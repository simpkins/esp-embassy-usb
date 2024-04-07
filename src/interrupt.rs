use crate::fmt::trace;
use crate::state::BUS_WAKER;
use esp_hal::peripherals::USB0;
use esp_hal_procmacros::interrupt;

/// The USB interrupt handler.
///
/// We do as little work here as possible, and only signal the BUS_WAKER.  This allows us to avoid
/// needing any manual critical sections when accessing USB0 registers or our State object, as all
/// modifications (apart from clearing gintmsk) are always done from a normal task and never in
/// interrupt context.
///
/// That said, a downside of this approach is that it may add latency to USB event processing,
/// since we have to wait for our embassy-usb task to wake up and run in order to process the
/// events.  If we decide it is necessary for performance, we could move some processing into this
/// interrupt handler, at the expense of requiring critical sections around some of the USB
/// register access and around our own state data structures.
#[interrupt]
fn USB() {
    // Safety: this code assumes that the interrupt handler and main USB task code run on the same
    // CPU core, and therefore cannot both run concurrently.  The interrupt handler can interrupt
    // the main task code at any point in time, but the main task code cannot execute concurrently
    // with the interrupt handler.
    let usb0 = unsafe { &*USB0::PTR };
    trace!("USB interrupt: 0x{:08x}", usb0.gintsts().read().bits());

    // Mask out all interrupts so we won't be called again until Bus::poll() runs,
    // then wake Bus::poll().
    usb0.gintmsk().reset();
    BUS_WAKER.wake();
}
