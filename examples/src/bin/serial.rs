#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)] // needed by esp_hal_procmacros::main

use embassy_executor::Spawner;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State as CdcAcmState};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use esp_backtrace as _;
use esp_embassy_usb::{Config as UsbConfig, Driver, State as UsbState};
use esp_hal::prelude::*;
use esp_hal::{clock::ClockControl, gpio::IO, peripherals::Peripherals, timer::TimerGroup};
use example_utils::{init_logging, init_serial};
use log::info;

#[main]
async fn main(_spawner: Spawner) {
    init_logging(log::LevelFilter::Trace);
    info!("Starting USB serial example...");

    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();
    let timg0 = TimerGroup::new_async(peripherals.TIMG0, &clocks);
    esp_hal::embassy::init(&clocks, timg0);

    let io = IO::new_with_priority(
        peripherals.GPIO,
        peripherals.IO_MUX,
        esp_hal::interrupt::Priority::Priority1,
    );

    let config = UsbConfig::bus_powered();
    let mut usb_state = UsbState::new(
        peripherals.USB0,
        peripherals.USB_WRAP,
        &peripherals.LPWR,
        io.pins.gpio19,
        io.pins.gpio20,
        config,
    );

    let mut config = embassy_usb::Config::new(0x6666, 0x6667);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB Serial Example");
    let serial = init_serial();
    config.serial_number = Some(core::str::from_utf8(&serial).unwrap());

    let mut cdc_acm_state = CdcAcmState::new();

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];
    let mut builder = Builder::new(
        usb_state.driver(),
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // Initialize the CDC-ACM class
    let mut class = CdcAcmClass::new(&mut builder, &mut cdc_acm_state, 64);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    // Do stuff with the class!
    let echo_fut = async {
        loop {
            class.wait_connection().await;
            info!("Connected");
            let _ = echo(&mut class).await;
            info!("Disconnected");
        }
    };

    // Run everything concurrently.
    embassy_futures::select::select(usb_fut, echo_fut).await;
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

struct DebugBuf<'a>(&'a [u8]);
impl<'a> core::fmt::LowerHex for DebugBuf<'a> {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
        for (n, c) in self.0.iter().enumerate() {
            if n > 0 {
                formatter.write_str(" ")?;
            }
            core::fmt::LowerHex::fmt(c, formatter)?;
        }
        Ok(())
    }
}

async fn echo<'d>(class: &mut CdcAcmClass<'d, Driver<'d>>) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];
        info!("data: {:x}", DebugBuf(data));
        class.write_packet(data).await?;
    }
}
