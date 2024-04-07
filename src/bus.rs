use crate::fmt::trace;
use crate::state::State;
use core::cell::RefCell;
use core::future::poll_fn;
use embassy_usb_driver::{Direction, EndpointAddress, Event, Unsupported};

pub struct Bus<'d> {
    state: &'d RefCell<State<'d>>,
}

impl<'d> Bus<'d> {
    pub(crate) fn new(state: &'d RefCell<State<'d>>) -> Self {
        Self { state }
    }
}

impl<'d> embassy_usb_driver::Bus for Bus<'d> {
    async fn poll(&mut self) -> Event {
        poll_fn(|cx| self.state.borrow_mut().poll_bus(cx)).await
    }

    fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        trace!("endpoint_set_stalled ep={:?} en={}", ep_addr, stalled);
        match ep_addr.direction() {
            Direction::Out => self.state.borrow_mut().stall_out_ep(ep_addr.index()),
            Direction::In => self.state.borrow_mut().stall_in_ep(ep_addr.index()),
        }
    }

    fn endpoint_is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
        match ep_addr.direction() {
            Direction::Out => self.state.borrow().is_out_ep_stalled(ep_addr.index()),
            Direction::In => self.state.borrow().is_in_ep_stalled(ep_addr.index()),
        }
    }

    // TODO: endpoint_set_enabled() really needs to be turned into an async function.
    // Disabling an OUT endpoint requires
    // - enabling global OUT NAK mode
    // - waiting for the GNOKEFF interrupt
    // - disabling the endpoint
    // - waiting for the EPDISD interrupt
    //
    // we currently have to busy loop for some of these operations since this function
    // is not async.
    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        trace!("endpoint_set_enabled ep={:?} en={}", ep_addr, enabled);
        match (ep_addr.direction(), enabled) {
            (Direction::Out, true) => self.state.borrow_mut().enable_out_ep(ep_addr.index()),
            (Direction::Out, false) => self.state.borrow_mut().disable_out_ep(ep_addr.index()),
            (Direction::In, true) => self.state.borrow_mut().enable_in_ep(ep_addr.index()),
            (Direction::In, false) => self.state.borrow_mut().disable_in_ep(ep_addr.index()),
        }
    }

    async fn enable(&mut self) {
        trace!("bus enable");
        // enable() is called after VBUS power is detected.
        // We don't currently need to do anything in particular here, as we keep the data line
        // pull-ups always enabled.
    }

    async fn disable(&mut self) {
        trace!("disable");
        // disable() is called after VBUS power is lost.
        // This only matters for self-powered devices :-)
    }

    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        trace!("remote_wakeup");
        // TODO: set dctl.rmtwkupsig, then clear it again.
        // According to docs it must be set for at least 1ms, and be cleared within 15ms.
        // The tinyusb code also enables the SOF interrupt and uses SOF to detect bus resume
        // afterwards.
        Err(Unsupported)
    }
}
