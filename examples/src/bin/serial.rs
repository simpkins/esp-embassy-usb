#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)] // needed by esp_hal_procmacros::main

use embassy_executor::Spawner;
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State as CdcAcmState};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use esp_backtrace as _;
use esp_embassy_usb::{Config as UsbConfig, Driver, State as UsbState};
use esp_hal::prelude::*;
use esp_hal::{clock::ClockControl, gpio::IO, peripherals::Peripherals, timer::TimerGroup};
use example_utils::{init_logging, init_serial};
use log::{debug, info};

#[main]
async fn main(_spawner: Spawner) {
    init_logging(log::LevelFilter::Debug);
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
    let cdc_acm = CdcAcmClass::new(&mut builder, &mut cdc_acm_state, 64);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    // Do stuff with the class!
    let (mut sender, mut receiver) = cdc_acm.split();
    let readline_fut = async {
        loop {
            receiver.wait_connection().await;
            info!("Connected");
            let mut buf = [0; 128];
            let mut loop_count = 0;
            loop {
                // Use an empty prompt on the first loop.  If we send data first, many terminals
                // will echo this data back to us, causing us to get in an echo loop with the host.
                // Waiting to let the host send data first appears to avoid this.
                let prompt = if loop_count == 0 { "" } else { "test> " };
                loop_count += 1;

                match readline(prompt, &mut buf, &mut sender, &mut receiver).await {
                    Ok(data) => {
                        info!("read line: {}", PrintableBuf(data))
                    }
                    Err(ReadlineError::Overflow) => {
                        info!("line buffer overflow: {}", PrintableBuf(&buf));
                        break;
                    }
                    Err(ReadlineError::Disconnected) => {
                        info!("Disconnected");
                        break;
                    }
                }
            }
        }
    };

    // Run everything concurrently.
    embassy_futures::select::select(usb_fut, readline_fut).await;
}

#[derive(Debug)]
enum ReadlineError {
    Disconnected,
    Overflow,
}

impl From<EndpointError> for ReadlineError {
    fn from(val: EndpointError) -> Self {
        match val {
            // Note that BufferOverflow here does not translate to ReadlineError::Overflow.
            // ReadlineError::Overflow indicates the readline buffer overflowed.  An endpoint
            // buffer overflow shouldn't happen unless we have a bug around max packet handling.
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => ReadlineError::Disconnected {},
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

struct PrintableBuf<'a>(&'a [u8]);
impl<'a> core::fmt::Display for PrintableBuf<'a> {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
        use core::fmt::Write;

        for c in self.0 {
            if *c >= 0x20 && *c <= 0x7e {
                formatter.write_char(*c as char)?;
            } else {
                formatter.write_char('\\')?;
                let nibble_char = |n| (if n < 10 { b'0' + n } else { b'a' + n - 10 }) as char;
                formatter.write_char(nibble_char(*c >> 4))?;
                formatter.write_char(nibble_char(*c & 0xf))?;
            }
        }
        Ok(())
    }
}

struct OutBuf<'a, 'd> {
    buf: [u8; 64],
    index: usize,
    sender: &'a mut Sender<'d, Driver<'d>>,
    need_flush: bool,
}

impl<'a, 'd> OutBuf<'a, 'd> {
    fn new(sender: &'a mut Sender<'d, Driver<'d>>) -> Self {
        Self {
            buf: [0; 64],
            index: 0,
            sender,
            need_flush: false,
        }
    }

    async fn putc(&mut self, c: u8) -> Result<(), EndpointError> {
        self.buf[self.index] = c;
        self.index += 1;
        if self.index >= self.buf.len() {
            self.send_now().await?;
            // After sending a full packet we need a zero-length packet
            // to indicate the end of the transaction.
            self.need_flush = true;
        }
        Ok(())
    }

    async fn put_bytes(&mut self, data: &[u8]) -> Result<(), EndpointError> {
        let mut data_idx = 0;
        while data_idx < data.len() {
            let data_len = data.len() - data_idx;
            let space_avail = self.buf.len() - self.index;
            let chunk_len = data_len.min(space_avail);
            self.buf[self.index..(self.index + chunk_len)]
                .copy_from_slice(&data[data_idx..(data_idx + chunk_len)]);
            data_idx += chunk_len;
            self.index += chunk_len;

            if self.index >= self.buf.len() {
                self.send_now().await?;
                self.need_flush = true;
            }
        }
        Ok(())
    }

    async fn puts(&mut self, s: &str) -> Result<(), EndpointError> {
        self.put_bytes(s.as_bytes()).await
    }

    async fn flush(&mut self) -> Result<(), EndpointError> {
        if self.index > 0 || self.need_flush {
            self.send_now().await?;
            self.need_flush = false;
        }
        Ok(())
    }

    async fn send_now(&mut self) -> Result<(), EndpointError> {
        self.sender.write_packet(&self.buf[0..self.index]).await?;
        self.index = 0;
        Ok(())
    }
}

async fn readline<'a, 'b, 'd>(
    prompt: &str,
    result_buf: &'b mut [u8],
    sender: &'a mut Sender<'d, Driver<'d>>,
    receiver: &'a mut Receiver<'d, Driver<'d>>,
) -> Result<&'b [u8], ReadlineError> {
    let mut in_packet = [0; 64];
    let mut out = OutBuf::new(sender);
    let mut result_idx = 0;

    if result_buf.len() == 0 {
        return Err(ReadlineError::Overflow);
    }

    out.puts(prompt).await?;
    out.flush().await?;

    const BACKSPACE: u8 = 0x08;
    const CTRL_U: u8 = 0x15;
    const DELETE: u8 = 0x7f;

    loop {
        let n = receiver.read_packet(&mut in_packet).await?;
        let in_data = &in_packet[..n];
        debug!("data: {:x}", DebugBuf(in_data));

        for c in in_data {
            match *c {
                b'\r' | b'\n' => {
                    // Receiving a newline is uncommon; terminals will typically send \r
                    // instead when enter is pressed.
                    out.put_bytes(b"\r\n").await?;
                    out.flush().await?;
                    return Ok(&result_buf[0..result_idx]);
                }
                BACKSPACE | DELETE => {
                    // backspace
                    if result_idx > 0 {
                        // Backspace, write a space over the character, then backspace again.
                        out.put_bytes(b"\x08 \x08").await?;
                        result_idx -= 1;
                    }
                }
                CTRL_U => {
                    // Clear the line
                    // Move cursor backwards by 999 columns, then clear to end of line
                    out.put_bytes(b"\x1b[999D\x1b[K").await?;
                    out.puts(prompt).await?;
                    result_idx = 0;
                }
                _ => {
                    out.putc(*c).await?;
                    result_buf[result_idx] = *c;
                    result_idx += 1;
                    if result_idx >= result_buf.len() {
                        return Err(ReadlineError::Overflow);
                    }
                }
            }
        }
        out.flush().await?;
    }
}
