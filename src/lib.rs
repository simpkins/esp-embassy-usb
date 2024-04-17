#![no_std]

pub use crate::bus::Bus;
pub use crate::control_pipe::ControlPipe;
pub use crate::driver::Driver;
use core::cell::RefCell;
use esp_hal::gpio::{
    AnyPin, DriveStrength, Floating, Input, InputOnlyPinType, InputOutputPinType, Output,
    OutputPin, PullDown, PushPull,
};
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

pub enum PhyType {
    Internal,
    External(ExtPhyConfig),
}

pub struct ExtPhyConfig {
    vp: AnyPin<Input<Floating>, InputOnlyPinType>,
    vm: AnyPin<Input<Floating>, InputOnlyPinType>,
    rcv: AnyPin<Input<Floating>, InputOnlyPinType>,
    oen: AnyPin<Output<PushPull>, InputOutputPinType>,
    vpo: AnyPin<Output<PushPull>, InputOutputPinType>,
    vmo: AnyPin<Output<PushPull>, InputOutputPinType>,
}

pub struct Config {
    pub vbus_detection_pin: Option<AnyPin<Input<PullDown>, InputOnlyPinType>>,
    pub phy_type: PhyType,
}

impl Config {
    /// Create a config for a bus-powered device using the internal PHY.
    ///
    /// Use this if your device is powered by the USB connection.  A vbus monitoring pin is not
    /// required in this case, since you can assume the bus always has power whenever the device is
    /// on.
    pub fn bus_powered() -> Self {
        Self {
            vbus_detection_pin: None,
            phy_type: PhyType::Internal,
        }
    }

    /// Create a Config object for a self-powered device using the internal PHY.
    ///
    /// The vbus_detection_pin is required to detect when power is present on the bus.  It should
    /// be connected to the USB VBUS pin using resistors to divide the 5V USB power signal into a
    /// 3.3V signal for the GPIO input.
    ///
    /// To call this with a specific pin, you normally will want to call
    /// `pin.into_pull_down_input().degrade().into_input_type()` in order to get an appropriate
    /// AnyPin type.
    pub fn self_powered(vbus_detection_pin: AnyPin<Input<PullDown>, InputOnlyPinType>) -> Self {
        Self {
            vbus_detection_pin: Some(vbus_detection_pin),
            phy_type: PhyType::Internal,
        }
    }

    /// Create a Config object for use with an external PHY.
    ///
    /// Note: Section 3.10 of the ESP32-S3 datasheet documents the following pins for the external
    /// PHY:
    /// - VP  gpio42
    /// - VM  gpio41
    /// - RCV gpio21
    /// - OEN gpio40
    /// - VPO gpio39
    /// - VMO gpio38
    /// That said, there are comments in the ESP-IDF that any GPIOs can be used and that some of
    /// these fixed pin definitions should be removed in IDF v6.0.  (See
    /// components/soc/esp32s3/include/soc/usb_pins.h)
    ///
    pub fn external_phy(
        vp: AnyPin<Input<Floating>, InputOnlyPinType>,
        vm: AnyPin<Input<Floating>, InputOnlyPinType>,
        rcv: AnyPin<Input<Floating>, InputOnlyPinType>,
        oen: AnyPin<Output<PushPull>, InputOutputPinType>,
        vpo: AnyPin<Output<PushPull>, InputOutputPinType>,
        vmo: AnyPin<Output<PushPull>, InputOutputPinType>,
        vbus_detection_pin: Option<AnyPin<Input<PullDown>, InputOnlyPinType>>,
    ) -> Self {
        Self {
            vbus_detection_pin: vbus_detection_pin,
            phy_type: PhyType::External(ExtPhyConfig {
                vp,
                vm,
                rcv,
                oen,
                vpo,
                vmo,
            }),
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
        mut config: Config,
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
        crate::phy::init_phy(&mut config, &*usb_wrap, rtc);

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
