use crate::state::State;
use core::cell::RefCell;
use embassy_usb_driver::{Direction, EndpointAddress, EndpointError, EndpointInfo, EndpointType};
use futures::future::poll_fn; // TODO: switch to core::future::poll_fn?
use log::trace;

pub struct EndpointIn<'d> {
    state: &'d RefCell<State>,
    pub(crate) info: EndpointInfo,
}

pub struct EndpointOut<'d> {
    state: &'d RefCell<State>,
    pub(crate) info: EndpointInfo,
}

impl<'d> EndpointIn<'d> {
    pub(crate) fn new(
        state: &'d RefCell<State>,
        index: usize,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Self {
        Self {
            state,
            info: EndpointInfo {
                addr: EndpointAddress::from_parts(index, Direction::In),
                ep_type,
                max_packet_size,
                interval_ms,
            },
        }
    }
}

impl<'d> EndpointOut<'d> {
    pub(crate) fn new(
        state: &'d RefCell<State>,
        index: usize,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Self {
        Self {
            state,
            info: EndpointInfo {
                addr: EndpointAddress::from_parts(index, Direction::Out),
                ep_type,
                max_packet_size,
                interval_ms,
            },
        }
    }
}

impl<'d> embassy_usb_driver::Endpoint for EndpointIn<'d> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        let ep_index = self.info.addr.index();
        poll_fn(|cx| self.state.borrow_mut().poll_in_ep_enabled(cx, ep_index)).await
    }
}

impl<'d> embassy_usb_driver::Endpoint for EndpointOut<'d> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        let ep_index = self.info.addr.index();
        poll_fn(|cx| self.state.borrow_mut().poll_out_ep_enabled(cx, ep_index)).await
    }
}

impl<'d> embassy_usb_driver::EndpointOut for EndpointOut<'d> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        let ep_index = self.info.addr.index();
        trace!("OUT EP{} read len={}", ep_index, buf.len());

        // Note: it looks like the embassy-usb code intends for EndpointOut::read() to be called
        // with at most max-packet-size at once.
        //
        // This seems unfortunate, since it will slow down read operations: we will consume only 1
        // packet at a time from the bus, NAKing subsequent packets until we have fetched the first
        // one from the RX FIFO, and forcing the host to wait until we clear the NAK flag after
        // each single packet read.
        //
        // It would be nice if this restriction were lifted in the embassy-usb API, but for now we
        // enforce this check here to avoid having to calculate the number of packets to read
        // in State::poll_out_ep_start_read().
        //
        // Allowing larger reads would complicate the API slightly: callers would need to always
        // read a multiple of the max packet size, so they can correctly detect a short packet that
        // signals the end of the transfer.  Things also get complicated if the read is aborted
        // partway through, after some packets were read into the caller's buffer but before the
        // whole transfer is complete.  (However, I might recommend just resetting the bus after a
        // timeout rather than trying to somehow recover and continue.)
        if buf.len() > self.info.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }

        // Start a read operation, then wait for it to complete.
        let mut read_op = crate::state::start_read_op(&self.state, ep_index, buf).await?;
        read_op.do_read().await
    }
}

impl<'d> embassy_usb_driver::EndpointIn for EndpointIn<'d> {
    async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        let ep_index = self.info.addr.index();
        trace!("IN EP{} write: {:?}", ep_index, buf);

        // The embassy-usb APIs unfortunately place the burden on the higher-level code to break
        // their writes up into max-packet-sized chunks.
        //
        // While this simplifies the driver code, this makes our writes less efficient, since we
        // have to give each packet to the HW in a separate transaction.  The Synopsys USB cores do
        // support accepting multiple packets worth of data at a time (i.e., in a single write to
        // the dieptsiz register, and then only generating a single xfer done interrupt when all
        // packets have been written).
        if buf.len() > self.info.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }

        // The driver currently always assigns each IN endpoint to the TX FIFO with the same
        // number.
        let fifo_index = ep_index;

        // This waits until the endpoint is free (in case an existing transfer is already in
        // progress) and for there to be enough room in the TX FIFO, and then transmits the packet.
        //
        // Note that if there a multiple outstanding calls to EndpointIn::write(),
        // they will both wait for the endpoint to be free and it is undefined which
        // one will win and get to go first.
        poll_fn(|cx| {
            self.state
                .borrow_mut()
                .poll_in_ep_write(cx, ep_index, fifo_index, buf)
        })
        .await
    }
}
