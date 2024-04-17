#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)] // needed by esp_hal_procmacros::main

use core::sync::atomic::{AtomicBool, Ordering};
use embassy_executor::Spawner;
use embassy_usb::class::hid::{HidReaderWriter, ReportId, RequestHandler, State as HidState};
use embassy_usb::control::OutResponse;
use embassy_usb::{Builder, Handler};
use esp_backtrace as _;
use esp_embassy_usb::{Config as UsbConfig, State as UsbState};
use esp_hal::gpio;
use esp_hal::timer::TimerGroup;
use esp_hal::{clock::ClockControl, peripherals::Peripherals, prelude::*};
use esp_hal_procmacros::main;
use example_utils::{init_logging, init_serial};
use usbd_hid::descriptor::{KeyboardReport, SerializedDescriptor};

#[main]
async fn main(_spawner: Spawner) {
    init_logging(log::LevelFilter::Debug);
    log::info!("Starting HID keyboard example...");

    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();
    let timg0 = TimerGroup::new_async(peripherals.TIMG0, &clocks);
    esp_hal::embassy::init(&clocks, timg0);

    let io = gpio::IO::new_with_priority(
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

    log::info!("driver created");

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0x6666, 0x6666);
    config.manufacturer = Some("Embassy");
    config.product = Some("HID Keyboard Example");
    let serial = init_serial();
    config.serial_number = Some(core::str::from_utf8(&serial).unwrap());

    let request_handler = MyRequestHandler {};
    let mut device_handler = MyDeviceHandler::new();
    let mut hid_state = HidState::new();

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    // You can also add a Microsoft OS descriptor.
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];
    let mut builder = Builder::new(
        usb_state.driver(),
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    builder.handler(&mut device_handler);

    // Initialize the HID class
    let config = embassy_usb::class::hid::Config {
        report_descriptor: KeyboardReport::desc(),
        request_handler: Some(&request_handler),
        poll_ms: 60,
        max_packet_size: 8,
    };
    let hid = HidReaderWriter::<_, 1, 8>::new(&mut builder, &mut hid_state, config);

    // Build the builder.
    let mut usb = builder.build();

    let (reader, mut writer) = hid.split();

    // GPIO 0 is connected to the boot button on ESP32 dev kits.
    // It is normally pulled high, and pressing the boot switch pulls it low.
    let mut button = io.pins.gpio0.into_floating_input();

    // Do stuff with the class!
    let in_fut = async {
        loop {
            let _ = button.wait_for_falling_edge().await;
            log::info!("Button pressed!");
            // Create a report with the A key pressed. (no shift modifier)
            let report = KeyboardReport {
                keycodes: [4, 0, 0, 0, 0, 0],
                leds: 0,
                modifier: 0,
                reserved: 0,
            };
            // Send the report.
            match writer.write_serialize(&report).await {
                Ok(()) => {}
                Err(e) => log::warn!("Failed to send report: {:?}", e),
            };

            let _ = button.wait_for_rising_edge().await;
            log::info!("Button released!");
            let report = KeyboardReport {
                keycodes: [0, 0, 0, 0, 0, 0],
                leds: 0,
                modifier: 0,
                reserved: 0,
            };
            match writer.write_serialize(&report).await {
                Ok(()) => {}
                Err(e) => log::warn!("Failed to send report: {:?}", e),
            };
        }
    };

    let out_fut = async {
        reader.run(false, &request_handler).await;
    };

    // Run everything concurrently.
    let usb_fut = usb.run();
    embassy_futures::select::select3(usb_fut, in_fut, out_fut).await;
}

struct MyRequestHandler {}

impl RequestHandler for MyRequestHandler {
    fn get_report(&self, id: ReportId, _buf: &mut [u8]) -> Option<usize> {
        log::info!("Get report for {:?}", id);
        None
    }

    fn set_report(&self, id: ReportId, data: &[u8]) -> OutResponse {
        log::info!("Set report for {:?}: {:?}", id, data);
        OutResponse::Accepted
    }

    fn set_idle_ms(&self, id: Option<ReportId>, dur: u32) {
        log::info!("Set idle rate for {:?} to {:?}", id, dur);
        // Note: a real keyboard example should honor the idle rate,
        // and resend this report at the specified idle frequency, even when there is no activity.
    }

    fn get_idle_ms(&self, id: Option<ReportId>) -> Option<u32> {
        log::info!("Get idle rate for {:?}", id);
        None
    }
}

struct MyDeviceHandler {
    configured: AtomicBool,
}

impl MyDeviceHandler {
    fn new() -> Self {
        MyDeviceHandler {
            configured: AtomicBool::new(false),
        }
    }
}

impl Handler for MyDeviceHandler {
    fn enabled(&mut self, enabled: bool) {
        self.configured.store(false, Ordering::Relaxed);
        if enabled {
            log::info!("Device enabled");
        } else {
            log::info!("Device disabled");
        }
    }

    fn reset(&mut self) {
        self.configured.store(false, Ordering::Relaxed);
        log::info!("Bus reset, the Vbus current limit is 100mA");
    }

    fn addressed(&mut self, addr: u8) {
        self.configured.store(false, Ordering::Relaxed);
        log::info!("USB address set to: {}", addr);
    }

    fn configured(&mut self, configured: bool) {
        self.configured.store(configured, Ordering::Relaxed);
        if configured {
            log::info!(
                "Device configured, it may now draw up to the configured current limit from Vbus."
            )
        } else {
            log::info!("Device is no longer configured, the Vbus current limit is 100mA.");
        }
    }
}
