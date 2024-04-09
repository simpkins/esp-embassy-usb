#![no_std]

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
