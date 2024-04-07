#![no_std]

pub use crate::bus::Bus;
pub use crate::control_pipe::ControlPipe;
pub use crate::driver::Driver;
use core::cell::RefCell;
use esp_hal::gpio::{DriveStrength, OutputPin};
use esp_hal::peripheral::Peripheral;
use esp_hal::peripherals::{LPWR, USB0, USB_WRAP};

mod bus;
mod control_pipe;
mod driver;
mod endpoint;
mod fmt;
mod interrupt;
mod phy;
mod state;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PhyType {
    Internal,
    External,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Config {
    vbus_detection_pin: Option<()>, // TODO: should be an optional GPIO
    phy_type: PhyType,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            vbus_detection_pin: None,
            phy_type: PhyType::Internal,
        }
    }
}

/// A data structure to store all state needed by the USB device driver.
///
/// Note that the State object is immovable once you have called driver(), as the USB objects
/// retain a reference to the State for as long as they exist.  If desired, you can use something
/// like static_cell::make_static() to create a single static State object that exists for the
/// lifetime of your program.
pub struct State<'d> {
    inner: RefCell<crate::state::State<'d>>,
}

// TODO: implement Drop for State, and reset the USB device when it is dropped.

impl<'d> State<'d> {
    pub fn new<P, M>(
        usb0: impl Peripheral<P = USB0> + 'd,
        usb_wrap: impl Peripheral<P = USB_WRAP> + 'd,
        rtc: &LPWR,
        mut dp: impl Peripheral<P = P> + 'd,
        mut dm: impl Peripheral<P = M> + 'd,
        config: Config,
    ) -> Self
    where
        P: esp_hal::otg_fs::UsbDp + OutputPin + Send + Sync,
        M: esp_hal::otg_fs::UsbDm + OutputPin + Send + Sync,
    {
        let mut usb0 = usb0.into_ref();
        let usb_wrap = usb_wrap.into_ref();

        if let PhyType::Internal = config.phy_type {
            // The C ESP-IDF code sets the drive strength to the strongest level
            // when using the internal PHY.
            //
            // We have to use clone_unchecked() here since the USB::new() call requires
            // Peripheral objects, and does not accept PeripheralRef.  (This perhaps seems like an
            // API design bug that it can't accept moved PeripheralRef objects?)
            unsafe { dp.clone_unchecked() }.set_drive_strength(DriveStrength::I40mA);
            unsafe { dm.clone_unchecked() }.set_drive_strength(DriveStrength::I40mA);
        }

        // Initialize the USB peripheral.
        // Unfortunately PeripheralClockControl is private inside esp-hal, and the only way
        // to initialize the peripheral is by calling esp_hal::otg_fs::USB::new() to create and
        // then destroy a temporary USB object.
        {
            esp_hal::otg_fs::USB::new(&mut *usb0, dp, dm);
        }

        // Perform one-time initialization of the PHY and other registers.
        // We do this here so that we only need a temporary reference to rtc.
        //
        // We really only need access to rtc.usb_conf(), but esp-hal doesn't really expose the type
        // name for this variable without us needing to directly import the specific chip esp-pac
        // crate.
        crate::phy::init_phy(&config, &*usb_wrap, rtc);

        Self {
            inner: RefCell::new(crate::state::State::new(usb0, config)),
        }
    }

    /// Create a Driver, which can be passed to embassy_usb::Builder
    pub fn driver<'r>(&'r mut self) -> Driver<'r>
    where
        'r: 'd,
    {
        Driver::<'r>::new(&self.inner)
    }
}

pub enum In {}
pub enum Out {}
