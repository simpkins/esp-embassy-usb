use crate::state::BUS_WAKER;
use esp_hal::peripherals::USB0;
use esp_hal_procmacros::interrupt;
use log::trace;

/// The USB interrupt handler.
///
/// We do as little work here as possible, and only signal the BUS_WAKER.
/// Any state accesses inside the interrupt handler would require manual critical sections here and
/// in any other normal task code that accesses the same state.  Therefore we prefer to avoid doing
/// work here and only access state from a normal embassy task.
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
