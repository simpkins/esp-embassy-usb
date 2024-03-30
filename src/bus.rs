use crate::state::State;
use core::cell::RefCell;
use embassy_usb_driver::{EndpointAddress, Event, Unsupported};
use futures::future::poll_fn;
use log::trace;

pub struct Bus<'d> {
    state: &'d RefCell<State>,
}

impl<'d> Bus<'d> {
    pub(crate) fn new(state: &'d RefCell<State>) -> Self {
        Self { state }
    }
}

impl<'d> embassy_usb_driver::Bus for Bus<'d> {
    async fn poll(&mut self) -> Event {
        poll_fn(|cx| self.state.borrow_mut().poll_bus(cx)).await
    }

    fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        trace!("endpoint_set_stalled ep={:?} en={}", ep_addr, stalled);

        // TODO
        /*
        assert!(
            ep_addr.index() < T::ENDPOINT_COUNT,
            "endpoint_set_stalled index {} out of range",
            ep_addr.index()
        );

        let regs = T::regs();
        match ep_addr.direction() {
            Direction::Out => {
                critical_section::with(|_| {
                    regs.doepctl(ep_addr.index()).modify(|w| {
                        w.set_stall(stalled);
                    });
                });

                T::state().ep_out_wakers[ep_addr.index()].wake();
            }
            Direction::In => {
                critical_section::with(|_| {
                    regs.diepctl(ep_addr.index()).modify(|w| {
                        w.set_stall(stalled);
                    });
                });

                T::state().ep_in_wakers[ep_addr.index()].wake();
            }
        }
        */
    }

    fn endpoint_is_stalled(&mut self, _ep_addr: EndpointAddress) -> bool {
        // TODO
        /*
        assert!(
            ep_addr.index() < T::ENDPOINT_COUNT,
            "endpoint_is_stalled index {} out of range",
            ep_addr.index()
        );

        let regs = T::regs();

        match ep_addr.direction() {
            Direction::Out => regs.doepctl(ep_addr.index()).read().stall(),
            Direction::In => regs.diepctl(ep_addr.index()).read().stall(),
        }
        */
        false
    }

    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        trace!("endpoint_set_enabled ep={:?} en={}", ep_addr, enabled);

        // TODO
        /*
        assert!(
            ep_addr.index() < T::ENDPOINT_COUNT,
            "endpoint_set_enabled index {} out of range",
            ep_addr.index()
        );

        let r = T::regs();
        match ep_addr.direction() {
            Direction::Out => {
                critical_section::with(|_| {
                    // cancel transfer if active
                    if !enabled && r.doepctl(ep_addr.index()).read().epena() {
                        r.doepctl(ep_addr.index()).modify(|w| {
                            w.set_snak(true);
                            w.set_epdis(true);
                        })
                    }

                    r.doepctl(ep_addr.index()).modify(|w| {
                        w.set_usbaep(enabled);
                    });

                    // Flush tx fifo
                    r.grstctl().write(|w| {
                        w.set_txfflsh(true);
                        w.set_txfnum(ep_addr.index() as _);
                    });
                    loop {
                        let x = r.grstctl().read();
                        if !x.txfflsh() {
                            break;
                        }
                    }
                });

                // Wake `Endpoint::wait_enabled()`
                T::state().ep_out_wakers[ep_addr.index()].wake();
            }
            Direction::In => {
                critical_section::with(|_| {
                    // cancel transfer if active
                    if !enabled && r.diepctl(ep_addr.index()).read().epena() {
                        r.diepctl(ep_addr.index()).modify(|w| {
                            w.set_snak(true); // set NAK
                            w.set_epdis(true);
                        })
                    }

                    r.diepctl(ep_addr.index()).modify(|w| {
                        w.set_usbaep(enabled);
                        w.set_cnak(enabled); // clear NAK that might've been set by SNAK above.
                    })
                });

                // Wake `Endpoint::wait_enabled()`
                T::state().ep_in_wakers[ep_addr.index()].wake();
            }
        }
        */
    }

    async fn enable(&mut self) {
        trace!("enable");
        // TODO: the comments in the embassy-stm32 crate indicate that the semantics
        // of Bus::disable() aren't well defined yet.
    }

    async fn disable(&mut self) {
        trace!("disable");
        // TODO: the comments in the embassy-stm32 crate indicate that the semantics
        // of Bus::disable() aren't well defined yet.
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
