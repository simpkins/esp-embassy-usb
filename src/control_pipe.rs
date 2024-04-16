use crate::endpoint::{EndpointIn, EndpointOut};
use crate::fmt::*;
use crate::state::State;
use core::cell::RefCell;
use core::future::poll_fn;
use embassy_usb_driver::{EndpointError, EndpointIn as _, EndpointOut as _};

pub struct ControlPipe<'d> {
    ep_in: EndpointIn<'d>,
    ep_out: EndpointOut<'d>,
}

impl<'d> ControlPipe<'d> {
    pub(crate) fn new(ep_in: EndpointIn<'d>, ep_out: EndpointOut<'d>) -> Self {
        Self { ep_in, ep_out }
    }

    fn state(&self) -> &'d RefCell<State<'d>> {
        self.ep_in.state()
    }
}

impl<'d> embassy_usb_driver::ControlPipe for ControlPipe<'d> {
    fn max_packet_size(&self) -> usize {
        self.ep_in.max_packet_size() as usize
    }

    async fn setup(&mut self) -> [u8; 8] {
        poll_fn(|cx| self.state().borrow_mut().poll_setup(cx)).await
    }

    async fn data_out(
        &mut self,
        buf: &mut [u8],
        _first: bool,
        _last: bool,
    ) -> Result<usize, EndpointError> {
        trace!("EP0: data_out reading {} bytes", buf.len());
        let len = self.ep_out.read(buf).await?;
        Ok(len)
    }

    async fn data_in(
        &mut self,
        data: &[u8],
        _first: bool,
        last: bool,
    ) -> Result<(), EndpointError> {
        trace!("EP0: data_in write: {:?}", data);
        self.ep_in.write(data).await?;

        // wait for status response from host after sending the last packet
        if last {
            trace!("EP0: acknowledge IN");
            self.ep_out.read(&mut []).await?;
        }

        Ok(())
    }

    async fn accept(&mut self) {
        trace!("EP0: accept");
        self.ep_in.write(&[]).await.ok();
    }

    async fn reject(&mut self) {
        trace!("EP0: reject");
        let state = self.state().borrow_mut();
        state.in_ep_set_stalled(0, true);
        state.out_ep_set_stalled(0, true);
    }

    async fn accept_set_address(&mut self, addr: u8) {
        trace!("EP0: address set to: {:#x}", addr);

        // Note that the SETUP reply packet takes effect *after* the end of the SETUP transaction:
        // the final "accept" packet should still contain the original address.  In other USB
        // implementations you often have to send the IN reply packet first, before updating the
        // address in hardware.  However, Synopsys USB cores handle the address transition, and
        // they document that you should update the address before triggering the IN status packet.
        self.state().borrow_mut().set_address(addr);
        self.accept().await
    }
}
