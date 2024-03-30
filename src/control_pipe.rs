use crate::endpoint::{EndpointIn, EndpointOut};
use crate::state::State;
use core::cell::RefCell;
use embassy_usb_driver::{EndpointError, EndpointIn as _, EndpointOut as _};
use futures::future::poll_fn;
use log::trace;

pub struct ControlPipe<'d> {
    state: &'d RefCell<State>,
    max_packet_size: u16,
    ep_in: EndpointIn<'d>,
    ep_out: EndpointOut<'d>,
}

impl<'d> ControlPipe<'d> {
    pub(crate) fn new(
        state: &'d RefCell<State>,
        max_packet_size: u16,
        ep_in: EndpointIn<'d>,
        ep_out: EndpointOut<'d>,
    ) -> Self {
        Self {
            state,
            max_packet_size,
            ep_in,
            ep_out,
        }
    }
}

impl<'d> embassy_usb_driver::ControlPipe for ControlPipe<'d> {
    fn max_packet_size(&self) -> usize {
        usize::from(self.max_packet_size)
    }

    async fn setup(&mut self) -> [u8; 8] {
        poll_fn(|cx| self.state.borrow_mut().poll_setup(cx)).await
    }

    async fn data_out(
        &mut self,
        buf: &mut [u8],
        _first: bool,
        _last: bool,
    ) -> Result<usize, EndpointError> {
        // TODO: we probably shouldn't just call ep_out.read() here: if we somehow get out of sync
        // with the host state (say due to a bug or a timeout), it may time out our transaction and
        // try to start a new one with a SETUP packet.  If a SETUP packet arrives while we are
        // waiting for ep_out.read() we should fail the operation and return an error.

        // TODO
        trace!("control: data_out");
        let len = self.ep_out.read(buf).await?;
        trace!("control: data_out read: {:?}", &buf[..len]);
        Ok(len)
    }

    async fn data_in(
        &mut self,
        data: &[u8],
        _first: bool,
        last: bool,
    ) -> Result<(), EndpointError> {
        // TODO: as in data_out(), we should probably be prepared for a SETUP packet to arrive at
        // any point, and fail the write attempt if we see a SETUP packet.
        trace!("control: data_in write: {:?}", data);
        self.ep_in.write(data).await?;

        // wait for status response from host after sending the last packet
        if last {
            trace!("control: data_in waiting for status");
            self.ep_out.read(&mut []).await?;
            trace!("control: complete");
        }

        Ok(())
    }

    async fn accept(&mut self) {
        trace!("control: accept");

        self.ep_in.write(&[]).await.ok();

        trace!("control: accept OK");
    }

    async fn reject(&mut self) {
        trace!("control: reject");

        // TODO
        /*
        // EP0 should not be controlled by `Bus` so this RMW does not need a critical section
        let regs = T::regs();
        regs.diepctl(self.ep_in.info.addr.index()).modify(|w| {
            w.set_stall(true);
        });
        regs.doepctl(self.ep_out.info.addr.index()).modify(|w| {
            w.set_stall(true);
        });
        */
    }

    async fn accept_set_address(&mut self, addr: u8) {
        trace!("address set to: {:#x}", addr);

        /*

        // The Synopsys docs indicate we should update DCFG first, before triggering the STATUS IN
        // response to the command (i.e., the accept() call).

        // Note: we only modify self.usb0.dcfg() from the main USB executor, and never from
        // interrupt context, so no critical section is needed here.
        self.usb0.dcfg().modify(|_, w| w.devaddr().set_bits(addr));

        // synopsys driver requires accept to be sent after changing address
        self.accept().await
        */
    }
}
