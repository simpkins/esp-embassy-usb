#![no_std]

use esp_hal::efuse::Efuse;
use log::LevelFilter;

pub fn init_logging(level: LevelFilter) {
    unsafe {
        log::set_logger_racy(&TimestampLogger).unwrap();
        log::set_max_level_racy(level);
    }
}

pub fn init_serial() -> [u8; 12] {
    let mac_addr = Efuse::get_mac_address();
    let mut serial: [u8; 12] = [0; 12];

    let nibble_to_hex = |n| {
        if n < 10 {
            b'0' + n
        } else {
            b'A' + (n - 10)
        }
    };

    // Just do a simple conversion of the MAC address to a hex string.
    // Note that USB mass storage places some requirements on the serial number, namely that it
    // consist only of hexadecimal digits, and that it have at least 12 digits.
    for n in 0..mac_addr.len() {
        serial[2 * n] = nibble_to_hex((mac_addr[n] >> 4) & 0xf);
        serial[(2 * n) + 1] = nibble_to_hex(mac_addr[n] & 0xf);
    }

    serial
}

pub struct TimestampLogger;

impl log::Log for TimestampLogger {
    fn enabled(&self, _metadata: &log::Metadata) -> bool {
        true
    }

    fn log(&self, record: &log::Record) {
        use esp_println::println;

        const RESET: &str = "\u{001B}[0m";
        const RED: &str = "\u{001B}[31m";
        const GREEN: &str = "\u{001B}[32m";
        const YELLOW: &str = "\u{001B}[33m";
        const BLUE: &str = "\u{001B}[34m";
        const CYAN: &str = "\u{001B}[35m";

        let color = match record.level() {
            log::Level::Error => RED,
            log::Level::Warn => YELLOW,
            log::Level::Info => GREEN,
            log::Level::Debug => BLUE,
            log::Level::Trace => CYAN,
        };

        let timestamp = embassy_time::Instant::now();
        let s = timestamp.as_secs();
        let ms = timestamp.as_millis() % 1000;
        println!(
            "{}{}.{:03}: {} - {}{}",
            color,
            s,
            ms,
            record.level(),
            record.args(),
            RESET
        );
    }

    fn flush(&self) {}
}
