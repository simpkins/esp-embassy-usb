use crate::driver::FifoSizes;
use crate::fmt::{error, trace, warn};
use crate::Config;
use core::cell::RefCell;
use core::future::poll_fn;
use core::task::Poll;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver::{EndpointAllocError, EndpointError, EndpointType, Event};
use esp_hal::peripheral::PeripheralRef;
use esp_hal::peripherals::{Interrupt, USB0};

pub const NUM_IN_ENDPOINTS: usize = 7;
pub const NUM_OUT_ENDPOINTS: usize = 7;

// The BUS_WAKER, used by the interrupt handler to wake the main task to perform work.
// This has to be global so it can be accessed by the USB interrupt handler.
pub(crate) static BUS_WAKER: AtomicWaker = AtomicWaker::new();

// This structure keeps track of common shared state needed by the Bus, ControlPipe, and endpoints.
pub(crate) struct State<'d> {
    usb0: PeripheralRef<'d, USB0>,

    config: Config,
    ep0_mps_bits: u8,
    bus_event_flags: u8,
    ep0_setup_ready: bool,
    ep0_setup_data: [u32; 2],

    ep_in_config: [Option<InEndpointConfig>; NUM_IN_ENDPOINTS],
    ep_out_config: [Option<OutEndpointConfig>; NUM_OUT_ENDPOINTS],

    ep_in_waker: [AtomicWaker; NUM_IN_ENDPOINTS],
    ep_out_waker: [AtomicWaker; NUM_OUT_ENDPOINTS],

    // A list of current read buffers.
    // These are populated only while a read is actively occurring on an endpoint
    ep_out_readers: [ReadBuf; NUM_OUT_ENDPOINTS],
}

// TODO: Despite being intended for async use, this code does have busy loops in a few places,
// waiting for things like AHB idle, for FIFO flushes to complete, or for endpoint disables to
// become effective.  flush_fifos_on_reset() is probably the worst offender.
//
// These operations should generally be relatively fast, and only occur rarely in situations like
// initialization or bus reset.  However, it would be nice to eliminate them.
//
// With some minor effort we probably could refactor the code a bit so that we yield back to the
// executor in between each check, and immediately signal readiness so that it will poll us again
// on the next executor loop.  This would give other tasks a chance to run while we are performing
// these checks, rather than starving other tasks while we loop.  We would need to track some
// additional state so that poll_bus() could remember the pending operation and resume where it
// left off the next time it is called.

#[derive(Debug)]
pub(crate) struct InvalidControlMaxPacketSize;

impl<'d> State<'d> {
    // Safety: it would be more correct for this API to accept a (Peripheral<P = USB0> + 'd) as an
    // argument, and then have the State lifetime be associated with the lifetime of the USB0
    // peripheral reference we were given.
    //
    // However, due to the fact that the embassy-usb API doesn't provide us with a location to
    // store the State, we have to store the State statically.  From a safety perspective, this
    // should be okay since the only things that use the State (Driver, ControlPipe, EndpointIn,
    // EndpointOut) each have their own lifetimes constrained to the lifetime of the USB0
    // peripheral.
    //
    // It would sort of be nice if we could reference count users of the State and automatically
    // reset the peripheral when all references were dropped.  That said, in practice most users
    // will likely keep the USB Driver for the duration of their program.
    //
    // TODO: this should probably accept in 'impl Peripheral', but at the moment we have to convert
    // the Peripheral to a PeripheralRef first in order to let esp_hal::otg_fs::USB initialize
    // the USB peripheral clock.
    pub fn new(usb0: PeripheralRef<'d, USB0>, config: Config) -> Self {
        // If we do not have a VBUS detection pin, start with a PowerDetected event recorded
        // so that we will return this the very first time poll_bus() is called.
        let bus_event_flags = if let None = config.vbus_detection_pin {
            bus_event_flag::POWER_DETECTED
        } else {
            0
        };

        let mut state = Self {
            usb0: usb0,
            config,
            ep0_mps_bits: 0,
            bus_event_flags,
            ep0_setup_data: [0; 2],
            ep0_setup_ready: false,
            ep_in_config: [None; NUM_IN_ENDPOINTS],
            ep_out_config: [None; NUM_OUT_ENDPOINTS],
            ep_in_waker: core::array::from_fn(|_| AtomicWaker::new()),
            ep_out_waker: core::array::from_fn(|_| AtomicWaker::new()),
            ep_out_readers: core::array::from_fn(|_| ReadBuf::new()),
        };

        // Endpoint 0 always exists.
        // We set max_packet_size to 64 here, but this may be updated later in Driver::start() with
        // a call to set_ep0_max_packet_size().
        state.ep_in_config[0] = Some(InEndpointConfig {
            ep_type: EndpointType::Control,
            tx_fifo: 0,
            max_packet_size: 64,
        });
        state.ep_out_config[0] = Some(OutEndpointConfig {
            ep_type: EndpointType::Control,
            max_packet_size: 64,
        });

        state
    }

    /// The main USB worker function.
    ///
    /// This services any interrupts that have occurred.
    ///
    /// Currently we run this any time Bus::poll() is called, or any time a method is called on
    /// endpoint 0 (either the IN or OUT sides).  In the future it would be nicer if there was a
    /// separate embassy task that just ran a `State::run()` function which continually await'ed
    /// a poll_fn() calling this in a loop.  For now we are relying on explicit knowledge of how
    /// the higher-level UsbDevice::run() function works, and the fact that it should always either
    /// be attempting to poll the bus or endpoint 0.
    ///
    /// See https://github.com/embassy-rs/embassy/issues/2751 for some discussion of this.
    pub fn poll_usb(&mut self, cx: &mut core::task::Context) {
        // Re-register to be woken again.
        BUS_WAKER.register(cx.waker());
        trace!("poll_usb starting");

        // Call process_interrupts() to process interrupts from the gintsts register.
        // This will clear each interrupt in gintsts as it processes them, so we won't get
        // notified again about ones that have been processed.  It may return an event before
        // processing all interrupts, in which case remaining ones will still be set in gintsts
        // so the interrupt handler should fire again immediately to re-process these once we
        // re-enable the interrupt mask below.
        self.process_interrupts();

        // Re-enable the interrupt mask
        self.enable_intmask();
    }

    /// This performs the main logic for Bus::poll()
    pub fn poll_bus(&mut self, cx: &mut core::task::Context) -> Poll<Event> {
        trace!("poll_bus starting");
        self.poll_usb(cx);

        if self.bus_event_flags == 0 {
            // common case
            return Poll::Pending;
        }
        // Multiple flags may be set, so just return the first one we find.
        for (flag, event) in [
            (bus_event_flag::POWER_DETECTED, Event::PowerDetected),
            (bus_event_flag::POWER_REMOVED, Event::PowerRemoved),
            (bus_event_flag::RESET, Event::Reset),
            (bus_event_flag::SUSPEND, Event::Suspend),
            (bus_event_flag::RESUME, Event::Resume),
        ] {
            if 0 != (self.bus_event_flags & flag) {
                self.bus_event_flags &= !flag;
                return Poll::Ready(event);
            }
        }
        // We should only reach here if there is a bug.
        Poll::Pending
    }

    pub fn set_ep0_max_packet_size(
        &mut self,
        max_packet_size: u16,
    ) -> Result<(), InvalidControlMaxPacketSize> {
        // Get the bits to set in the diepctl0 and doepctl0 registers to indicate
        // the maximum packet size for the control endpoint.
        self.ep0_mps_bits = match max_packet_size {
            64 => 0,
            32 => 1,
            16 => 2,
            8 => 3,
            _ => {
                return Err(InvalidControlMaxPacketSize);
            }
        };
        if let Some(config) = &mut self.ep_in_config[0] {
            config.max_packet_size = max_packet_size;
        }
        if let Some(config) = &mut self.ep_out_config[0] {
            config.max_packet_size = max_packet_size;
        }
        Ok(())
    }

    pub fn set_address(&mut self, address: u8) {
        self.usb0.dcfg().modify(|_, w| w.devaddr().bits(address));
    }

    pub fn init_bus(&mut self, fifo_settings: &FifoSizes) {
        // Ensure the data line pull-up is disabled as we perform initialization.
        self.usb0.dctl().modify(|_, w| w.sftdiscon().set_bit());

        // Set the speed.  Also configure the device to send a stall on receipt of any
        // unexpected non-zero length OUT packet during the status phase of a control transfer.
        self.usb0.dcfg().modify(|_, w| {
            unsafe { w.bits(devspd::FULL_48MHZ) }
                .nzstsouthshk()
                .set_bit()
        });

        // TODO: do we need to do anything extra for VBUS sense setup, or should all that
        // have already been done in init_phy()?  STMicro chips document a gcctl register
        // to configure with VBUS sense settings, but this is not listed in the esp32 SVD.

        // Wait for AHB idle.
        // Note that the tinyusb esp32 implementation doesn't do this, but STMicro docs appear to
        // recommend this.
        while self.usb0.grstctl().read().ahbidle().bit_is_clear() {}

        // Configure AHB interrupts
        self.usb0.gahbcfg().modify(|_, w| {
            // Turn on the global interrupt enable flag
            let w = w.glbllntrmsk().set_bit();
            // Enable periodic TxFIFO empty interval (recommended by synopsys USB core docs for
            // STM32 chips)
            let w = w.ptxfemplvl().set_bit();
            // Configure the non-periodic TxFIFO empty interrupt to fire when completely empty
            // rather than half empty
            w.nptxfemplvl().set_bit()
        });

        // USB configuration
        self.usb0.gusbcfg().modify(|_, w| {
            // Force device mode
            let w = w.forcedevmode().set_bit();
            // We don't currently support HNP or SRP
            let w = w.hnpcap().clear_bit().srpcap().clear_bit();
            // Set the timeout calibration to the maximum value
            let w = unsafe { w.toutcal().bits(0x03) };
            // Set the USB turnaround time to 5, which is the recommended value from the docs.
            // The include/soc/usb_reg.h header in ESP-IDF documents that the values for this field
            // should be
            // - 5 when the MAC interface is 16-bit UTMI+
            // - 9 when the MAC interface is 8-bit UTMI+
            // (maybe we would need to change this when using an external PHY, depending on the PHY?)
            unsafe { w.usbtrdtim().bits(5) }
        });

        // Clear overrides in the OTG configuration
        self.usb0.gotgctl().modify(|_, w| {
            w.bvalidovval()
                .clear_bit()
                .bvalidoven()
                .clear_bit()
                .vbvalidovval()
                .clear_bit()
        });

        // Set the NAK flag on all out endpoints
        self.nak_all_out_endpoints();

        self.init_fifos(&fifo_settings);

        // Mask all interrupts
        self.usb0.gintmsk().reset();
        // Clear any pending OTG interrupts
        self.usb0.gotgint().write(|w| unsafe { w.bits(0xffffffff) });
        // Clear any pending interrupts
        self.usb0.gintsts().write(|w| unsafe { w.bits(0xffffffff) });

        // Enable the interrupts we care about
        self.enable_intmask();

        // Enable the interrupt handler
        esp_hal::interrupt::enable(Interrupt::USB, esp_hal::interrupt::Priority::Priority1)
            .expect("failed to enable USB interrupt");

        // Enable the data line pull-up to connect the bus
        self.usb0.dctl().modify(|_, w| w.sftdiscon().clear_bit());
    }

    pub(crate) fn alloc_in_endpoint(
        &mut self,
        config: InEndpointConfig,
    ) -> Result<usize, EndpointAllocError> {
        let ep_index = self.find_free_endpoint_slot(&self.ep_in_config)?;
        self.ep_in_config[ep_index] = Some(config);
        Ok(ep_index)
    }

    pub(crate) fn alloc_out_endpoint(
        &mut self,
        config: OutEndpointConfig,
    ) -> Result<usize, EndpointAllocError> {
        let ep_index = self.find_free_endpoint_slot(&self.ep_out_config)?;
        self.ep_out_config[ep_index] = Some(config);
        Ok(ep_index)
    }

    fn find_free_endpoint_slot<Slot>(
        &self,
        slots: &[Option<Slot>],
    ) -> Result<usize, EndpointAllocError> {
        for (index, ep) in slots.iter().enumerate() {
            if ep.is_none() {
                trace!("allocating endpoint {}", index);
                return Ok(index);
            }
        }
        error!("No free endpoints available");
        return Err(EndpointAllocError);
    }

    fn init_fifos(&mut self, sizes: &FifoSizes) {
        // Sanity check that things aren't currently in use
        assert!(
            {
                let r = self.usb0.daintmsk().read();
                r.outepmsk0().bit_is_clear() && r.inepmsk0().bit_is_clear()
            },
            "init_fifos() called after EP0 has already been configured"
        );
        trace!("init_fifos: {:?}", sizes);

        // Note: during initialization we assume that the USB bus has not been used yet,
        // so the USB core should not be currently accessing the RX or TX FIFOs.
        // We could call self.flush_fifos_on_reset() here if we wanted to handle the case where
        // someone else has already used the USB peripheral before they gave ownership of it to us.

        // Configure the RX FIFO size
        self.usb0
            .grxfsiz()
            .write(|w| unsafe { w.rxfdep().bits(sizes.rx_num_words) });

        let mut offset_words = sizes.rx_num_words;

        // Configure TX FIFOs
        // TX FIFO 0 is configured with usb0.gnptxfsiz()
        self.usb0.gnptxfsiz().write(|w| unsafe {
            w.nptxfstaddr()
                .bits(offset_words)
                .nptxfdep()
                .bits(sizes.tx_num_words[0])
        });
        offset_words += sizes.tx_num_words[0];

        // TX FIFOs 1-4 are configured with usb0.dieptxfN
        for n in 0..4 {
            self.usb0.dieptxf(n).write(|w| unsafe {
                w.inep1txfstaddr()
                    .bits(offset_words)
                    .inep1txfdep()
                    .bits(sizes.tx_num_words[n + 1])
            });
            offset_words = offset_words + sizes.tx_num_words[0];
        }
    }

    pub fn poll_setup(&mut self, cx: &mut core::task::Context) -> Poll<[u8; 8]> {
        // Re-register to be woken again.
        self.ep_out_waker[0].register(cx.waker());

        if !self.ep0_setup_ready {
            trace!("SETUP poll waiting");
            return Poll::Pending;
        }

        // Update doeptsiz0 to reset the setup count and indicate that we can receive up to
        // 3 more back-to-back SETUP packets.
        self.usb0
            .out_ep0()
            .doeptsiz()
            .modify(|_, w| unsafe { w.supcnt().bits(3) });

        trace!(
            "SETUP ready: {:?}",
            bytemuck::cast_slice::<u32, u8>(&self.ep0_setup_data)
        );
        self.ep0_setup_ready = false;
        Poll::Ready(bytemuck::cast(self.ep0_setup_data))
    }

    pub fn poll_out_ep_enabled(
        &mut self,
        cx: &mut core::task::Context,
        ep_index: usize,
    ) -> Poll<()> {
        trace!("OUT EP{} polling for enabled", ep_index);
        if ep_index == 0 {
            // https://github.com/embassy-rs/embassy/issues/2751
            self.poll_usb(cx);

            // Endpoint 0 is always enabled.
            return Poll::Ready(());
        }

        self.ep_out_waker[ep_index].register(cx.waker());
        let doepctl = self.usb0.out_ep(ep_index - 1).doepctl().read();
        if doepctl.usbactep().bit_is_set() {
            trace!("OUT EP{} poll enabled returning ready", ep_index);
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }

    fn poll_out_ep_start_read(
        &mut self,
        cx: &mut core::task::Context,
        ep_index: usize,
        buf: &mut [u8],
    ) -> Poll<Result<(), EndpointError>> {
        trace!("OUT EP{} RX: polling for start read", ep_index);
        if ep_index == 0 {
            // https://github.com/embassy-rs/embassy/issues/2751
            self.poll_usb(cx);
        }

        let ep_config = match self.ep_out_config[ep_index] {
            Some(config) => config,
            None => {
                return Poll::Ready(Err(EndpointError::Disabled));
            }
        };
        if let Some(err) = self.check_out_read_error(ep_index, false) {
            return Poll::Ready(Err(err));
        }
        self.ep_out_waker[ep_index].register(cx.waker());

        // Try to register as the current reader for this endpoint
        if self.ep_out_readers[ep_index].set_reader(buf).is_err() {
            // Another task is already reading on this endpoint.  Wait until it finishes.
            trace!(
                "OUT EP{} RX: waiting for in-progress read to complete",
                ep_index
            );
            return Poll::Pending;
        }

        trace!("OUT EP{} RX: starting read", ep_index);

        // Specify the number of packets to read and total length.
        // Currently the higher-level embassy APIs only read at most max_packet_size at once,
        // and we verify this in EndpointOut::read()
        const NUM_PACKETS: u8 = 1;
        // We always ask for max_packet_size at once, even if the reader asked for less.
        // If the host sends more data in the transfer than the user requested, we want to know and
        // generate an error.
        let read_size = ep_config.max_packet_size;
        assert!(read_size as usize >= buf.len());
        if ep_index == 0 {
            let out_ep = self.usb0.out_ep0();
            // Endpoint 0 only supports reading up to 1 OUT packet at a time
            out_ep
                .doeptsiz()
                .modify(|_, w| w.xfersize().bits(read_size as u8).pktcnt().set_bit());
            // Clear the NAK flag and enable endpoint to allow the hardware to start reading.
            out_ep
                .doepctl()
                .modify(|_, w| w.cnak().set_bit().epena().set_bit());
        } else {
            let out_ep = self.usb0.out_ep(ep_index - 1);
            out_ep.doeptsiz().modify(|_, w| {
                w.xfersize()
                    .bits(read_size as u32)
                    .pktcnt()
                    .bits(NUM_PACKETS as u16)
            });
            // Clear the NAK flag and enable endpoint to allow the hardware to start reading.
            //
            // TODO: For isochronous endpoints with interval == 1 I believe we need to manually set
            // doepctl.setd0pid or doepctl.setd1pid.
            out_ep
                .doepctl()
                .modify(|_, w| w.cnak().set_bit().epena().set_bit());
        }

        Poll::Ready(Ok(()))
    }

    fn poll_out_ep_read_complete(
        &mut self,
        cx: &mut core::task::Context,
        ep_index: usize,
    ) -> Poll<Result<usize, EndpointError>> {
        trace!("OUT EP{} RX: polling for read complete", ep_index);
        if ep_index == 0 {
            // https://github.com/embassy-rs/embassy/issues/2751
            self.poll_usb(cx);
        }

        if let Some(result) = self.ep_out_readers[ep_index].check_read_complete() {
            return Poll::Ready(result);
        }
        if let Some(err) = self.check_out_read_error(ep_index, true) {
            return Poll::Ready(Err(err));
        }

        self.ep_out_waker[ep_index].register(cx.waker());
        Poll::Pending
    }

    fn check_out_read_error(
        &mut self,
        ep_index: usize,
        already_reading: bool,
    ) -> Option<EndpointError> {
        if ep_index == 0 {
            if already_reading {
                // If the bus is reset while we are reading, endpoint 0 doesn't get disabled
                // but the NAK flag will be set on it.  Check to see if the NAK flag has been
                // disabled since we started our read attempt.
                let out_ep = self.usb0.out_ep0();
                let doepctl = out_ep.doepctl().read();
                trace!("  EP0 doepctl: 0x{:08x}", doepctl.bits());
                if doepctl.naksts().bit_is_set() {
                    warn!("bus reset while attempting to read from EP0");
                    return Some(EndpointError::Disabled);
                }
            }

            // If we are trying to read from endpoint 0 and we see a SETUP packet instead of an OUT
            // packet, we should fail the read attempt.  This may happen if we get out of sync with
            // the host somehow (e.g., due to a bug in our control transfer handling, or perhaps
            // due to a buggy host).
            if ep_index == 0 && self.ep0_setup_ready {
                error!("received SETUP packet on EP0 while expecting an OUT packet");
                // There unfortunately isn't a good EndpointError code for us to return here.
                return Some(EndpointError::Disabled);
            }
        } else {
            // If the endpoint is disabled, fail the read.
            // This can happen if the bus is reset while we are attempting to read.
            let out_ep = self.usb0.out_ep(ep_index - 1);
            let doepctl = out_ep.doepctl().read();
            if doepctl.usbactep().bit_is_clear() {
                trace!("OUT EP{} RX: endpoint has been disabled", ep_index);
                return Some(EndpointError::Disabled);
            }
        }

        None
    }

    fn read_operation_dropped(&mut self, ep_index: usize, buf: &mut [u8]) {
        // If this is the current reader, stop the read.
        // (It is possible that this is an old ReadOperation that already finished,
        // and the caller started a new read before dropping their old ReadOperation variable.)
        let is_current = self.ep_out_readers[ep_index].read_operation_dropped(buf);
        if is_current {
            // Ask the hardware to NAK new OUT packets and disable active transfers.
            // Note: this is not immediately effective, and the hardware may still place another packet
            // for us in the RX FIFO until this takes effect.
            if ep_index == 0 {
                // Cannot set the EPDIS bit on endpoint 0
                self.usb0
                    .out_ep0()
                    .doepctl()
                    .modify(|_, w| w.snak().set_bit());
            } else {
                self.usb0
                    .out_ep(ep_index - 1)
                    .doepctl()
                    .modify(|_, w| w.epdis().set_bit().snak().set_bit());
            }
        }
    }

    pub fn poll_in_ep_enabled(
        &mut self,
        cx: &mut core::task::Context,
        ep_index: usize,
    ) -> Poll<()> {
        trace!("IN EP{} polling for enabled", ep_index);
        if ep_index == 0 {
            // https://github.com/embassy-rs/embassy/issues/2751
            self.poll_usb(cx);
            // Endpoint 0 is always enabled.
            return Poll::Ready(());
        }

        self.ep_in_waker[ep_index].register(cx.waker());
        let diepctl = self.usb0.in_ep(ep_index - 1).diepctl().read();
        if diepctl.usbactep().bit_is_set() {
            trace!("IN EP{} poll enabled returning ready", ep_index);
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }

    /// Attempt to write a single packet on an IN endpoint to the hardware buffers.
    ///
    /// Returns Pending if the IN endpoint is currently busy.
    ///
    /// Otherwise, writes the packet to the hardware buffers and makes it available to the hardware
    /// to transmit the next time the host sends an IN token, and then returns Ready<Ok>.  (This
    /// does not wait for the hardware to actually transmit the data on the bus.)
    pub fn poll_in_ep_write(
        &mut self,
        cx: &mut core::task::Context,
        ep_index: usize,
        buf: &[u8],
    ) -> Poll<Result<(), EndpointError>> {
        trace!("IN EP{} TX: polling for write", ep_index);
        if ep_index == 0 {
            // https://github.com/embassy-rs/embassy/issues/2751
            self.poll_usb(cx);
        }

        self.ep_in_waker[ep_index].register(cx.waker());

        let fifo_index = match self.ep_in_config[ep_index] {
            Some(cfg) => cfg.tx_fifo,
            None => {
                // shouldn't happen unless there is a bug.
                return Poll::Ready(Err(EndpointError::Disabled));
            }
        };

        let dtxfsts = if ep_index == 0 {
            let ep_regs = self.usb0.in_ep0();
            let diepctl = ep_regs.diepctl().read();
            // Don't bother checking usbactep for endpoint 0, as this bit is always set to 1.
            if diepctl.epena().bit_is_set() {
                trace!("IN EP{} TX: endpoint is busy", ep_index);
                return Poll::Pending;
            }
            ep_regs.dtxfsts()
        } else {
            let ep_regs = self.usb0.in_ep(ep_index - 1);
            let diepctl = ep_regs.diepctl().read();
            if diepctl.usbactep().bit_is_clear() {
                trace!("IN EP{} TX: endpoint is disabled", ep_index);
                return Poll::Ready(Err(EndpointError::Disabled));
            }
            if diepctl.epena().bit_is_set() {
                trace!("IN EP{} TX: endpoint is busy", ep_index);
                return Poll::Pending;
            }
            ep_regs.dtxfsts()
        };

        // Check for available TX FIFO space.
        // (I suspect that it shouldn't be possible for this check to fail, since we only ever
        // write a single packet at a time and we wait for the endpoint to become idle before
        // starting the next write.)
        if buf.len() > 0 {
            // Note that the STMicro docs indicate that we must not read dtxfsts if we are
            // transmitting a 0-length packet.
            let fifo_space_avail = dtxfsts.read().ineptxfspcavail().bits();
            let size_words = (buf.len() + 3) / 4;
            if size_words > (fifo_space_avail as usize) {
                // Not enough space in the FIFO
                // Enable the TX FIFO empty interrupt
                trace!("IN EP{} TX: waiting for TX FIFO space", ep_index);
                self.usb0.diepempmsk().modify(|r, w| {
                    let new_bits = r.d_ineptxfempmsk().bits() | (1 << ep_index);
                    unsafe { w.d_ineptxfempmsk().bits(new_bits) }
                });
                return Poll::Pending;
            }
        }

        // Everything is ready, we can proceed to transmit the packet
        trace!("IN EP{} TX: writing {} bytes", ep_index, buf.len());

        // The embassy-usb APIs require callers never send more than max packet size at once, and
        // the EndpointIn::write() code has already verified that buf.len() is within this limit.
        // (TODO: we should support writing multiple packets at once in the future.  Note that
        // endpoint supports only up to 3 packets at a time, while other endpoints support more.)
        const NUM_PACKETS: u8 = 1;

        // Update dieptsiz to inform the core how much data (and how many packets) to send,
        // and then update diepctl to clear the NAK flag and enable writing.
        if ep_index == 0 {
            let ep_regs = self.usb0.in_ep0();
            ep_regs.dieptsiz().write(|w| {
                w.pktcnt()
                    .bits(NUM_PACKETS)
                    .xfersize()
                    .bits(buf.len() as u8)
            });
            ep_regs
                .diepctl()
                .modify(|_, w| w.cnak().set_bit().epena().set_bit());
        } else {
            let ep_regs = self.usb0.in_ep(ep_index - 1);
            ep_regs.dieptsiz().write(|w| {
                w.pktcnt()
                    .bits(NUM_PACKETS as u16)
                    .xfersize()
                    .bits(buf.len() as u32)
            });
            // TODO: For isochronous endpoints with interval == 1 I believe we need to manually set
            // diepctl.setd0pid or diepctl.setd1pid.
            ep_regs
                .diepctl()
                .modify(|_, w| w.cnak().set_bit().epena().set_bit());
        }

        // Finally write all data to the FIFO
        self.write_to_fifo(buf, fifo_index);
        Poll::Ready(Ok(()))
    }

    #[inline(always)]
    fn enable_intmask(&mut self) {
        self.usb0.gintmsk().write(|w| {
            w.modemismsk()
                .set_bit()
                .otgintmsk()
                .set_bit()
                .rxflvimsk()
                .set_bit()
                .erlysuspmsk()
                .set_bit()
                .usbsuspmsk()
                .set_bit()
                .usbrstmsk()
                .set_bit()
                .resetdetmsk()
                .set_bit()
                .enumdonemsk()
                .set_bit()
                .sessreqintmsk()
                .set_bit()
                .iepintmsk()
                .set_bit()
                .oepintmsk()
                .set_bit()
        });
    }

    /// Record a bus event that has happened,
    /// so we can return it the next time Bus::poll() is called.
    fn record_bus_event(&mut self, event: Event) {
        self.bus_event_flags |= match event {
            Event::PowerDetected => bus_event_flag::POWER_DETECTED,
            Event::PowerRemoved => bus_event_flag::POWER_REMOVED,
            Event::Reset => bus_event_flag::RESET,
            Event::Suspend => bus_event_flag::SUSPEND,
            Event::Resume => bus_event_flag::RESUME,
        };
    }

    fn process_interrupts(&mut self) {
        let ints = self.usb0.gintsts().read();
        trace!("USB poll bus: ints=0x{:08x}", ints.bits());

        // Fast path check for when there is nothing to do.
        // Embassy can unfortunately call poll() more often than it needs to, even when our waker
        // was never asked to wake up.  (The embassy select* APIs poll all pending futures on all
        // branches any time any future is woken up.)  Therefore we want to make the no-op path
        // fast.  The compiler should be smart enough to optimize this to a single u32 comparison.
        if !(ints.rxflvi().bit_is_set()
            || ints.modemis().bit_is_set()
            || ints.sessreqint().bit_is_set()
            || ints.erlysusp().bit_is_set()
            || ints.otgint().bit_is_set()
            || ints.usbrst().bit_is_set()
            || ints.resetdet().bit_is_set()
            || ints.enumdone().bit_is_set()
            || ints.usbsusp().bit_is_set()
            || ints.wkupint().bit_is_set()
            || ints.oepint().bit_is_set()
            || ints.iepint().bit_is_set())
        {
            return;
        }

        // Process RX FIFO events first, before bus reset events.
        // When first connecting to a device, Linux hosts tend to request the device descriptor,
        // then immediately reset the bus and reset the descriptor again after a reset.  On this
        // first reset we often get the status acknowledgement of the device desctiptor response
        // together with an interrupt resetting the bus.  Process the info from the RX FIFO first,
        // just so we go through the normal SETUP transaction completion logic rather than first
        // processing the reset and warn about the transaction being aborted partway through.
        //
        // I wonder if RXFLVI is a typo in the esp32s2/s3 SVD?  This is normally called RXFLVL.
        if ints.rxflvi().bit() {
            self.process_rx_fifo();
        }

        if ints.modemis().bit() {
            error!("USB mode mismatch");
            self.usb0.gintsts().write(|w| w.modemis().set_bit());
        }

        if ints.sessreqint().bit() {
            trace!("vbus detected");
            self.usb0.gintsts().write(|w| w.sessreqint().set_bit());
            if self.config.vbus_detection_pin.is_some() {
                self.record_bus_event(Event::PowerDetected);
            }
        }

        if ints.erlysusp().bit() {
            trace!("early suspend");
            self.usb0.gintsts().write(|w| w.erlysusp().set_bit());
            if self.usb0.dsts().read().errticerr().bit() {
                // According to some of the STMicro docs, if an erratic error is detected, the
                // dsts.errticerr bit is set, the OTG_FS core is suspsended, and an erlysusp
                // interrupt is generated.  We need to perform a soft disconnect to recover
                //
                // (Note: I haven't actually tested that this code works to recover the device
                // state after an erratic error in practice.)
                trace!("erratic error recovery");
                self.usb0.dctl().modify(|_, w| w.sftdiscon().set_bit());
                self.usb0.dctl().modify(|_, w| w.sftdiscon().clear_bit());
            }
        }

        if ints.otgint().bit() {
            let otgints = self.usb0.gotgint().read();
            trace!("OTG interrupt: {:#x}", otgints.bits());
            // Clear all OTG interrupts in gotgint.  No need to update gintsts
            self.usb0
                .gotgint()
                .write(|w| unsafe { w.bits(otgints.bits()) });

            if otgints.sesenddet().bit() {
                trace!("vbus removed");
                if self.config.vbus_detection_pin.is_some() {
                    self.disable_all_endpoints_on_vbus_removed();
                    self.record_bus_event(Event::PowerRemoved);
                }
            }
        }

        if ints.usbrst().bit() || ints.resetdet().bit() {
            // usbrst indicates a reset.
            // resetdet indicates a reset detected while in suspend mode.
            // These two tend to be asserted together when first initializing the bus.
            trace!("USB reset");
            self.usb0
                .gintsts()
                .write(|w| w.usbrst().set_bit().resetdet().set_bit());
            self.process_reset()
        }

        if ints.enumdone().bit() {
            trace!("USB enumeration done");
            self.usb0.gintsts().write(|w| w.enumdone().set_bit());
            self.process_enum_done();
            self.record_bus_event(Event::Reset);
        }

        if ints.usbsusp().bit() {
            trace!("USB suspend");
            self.usb0.gintsts().write(|w| w.usbsusp().set_bit());
            self.record_bus_event(Event::Suspend);
        }

        if ints.wkupint().bit() {
            trace!("USB resume");
            self.usb0.gintsts().write(|w| w.wkupint().set_bit());
            self.record_bus_event(Event::Resume);
        }

        if ints.oepint().bit() {
            self.process_out_ep_interrupts();
        }

        if ints.iepint().bit() {
            self.process_in_ep_interrupts();
        }
    }

    fn process_reset(&mut self) {
        self.nak_all_out_endpoints();

        // Clear the device address
        self.usb0.dcfg().modify(|_, w| w.devaddr().bits(0));

        // Disable all endpoint interrupts
        self.usb0.daintmsk().write(|w| unsafe { w.bits(0) });
        self.usb0.doepmsk().write(|w| unsafe { w.bits(0) });
        self.usb0.diepmsk().write(|w| unsafe { w.bits(0) });

        self.flush_fifos_on_reset();

        // Wake any endpoints that may have been waiting on transactions
        for waker in &self.ep_in_waker {
            waker.wake();
        }
        for waker in &self.ep_out_waker {
            waker.wake();
        }

        // Note: we don't change the FIFO configuration (grxfsiz, gnptxfsiz, dieptxf1, etc.)
        // This is set initially during Driver::start(), and since embassy-usb doesn't allow
        // changing endpoint config after start() we never need to update the FIFO configuration.

        // Re-enable receipt of SETUP and XFER complete endpoint interrupts
        self.usb0
            .daintmsk()
            .modify(|_, w| w.outepmsk0().set_bit().inepmsk0().set_bit());
        self.usb0
            .doepmsk()
            .modify(|_, w| w.setupmsk().set_bit().xfercomplmsk().set_bit());
        self.usb0
            .diepmsk()
            .modify(|_, w| w.timeoutmsk().set_bit().di_xfercomplmsk().set_bit());
        self.usb0
            .out_ep0()
            .doeptsiz()
            .modify(|_, w| unsafe { w.supcnt().bits(3) });
    }

    fn process_enum_done(&mut self) {
        // self.usb0.dsts().enumspd() will indicate the device speed
        // We always expect this to be devspd::FULL_48MHZ

        // Note: usb0.gusbcfg().trdt() should be updated once the bus speed has
        // been determined.  However, we don't do this here, and always leave it set to 5,
        // which we set it to during init_bus().  We expect the bus speed to always be
        // devspd::FULL_48MHZ, so we shouldn't need to change it here.

        // allocate_tx_fifo() always uses TX FIFO 0 for the control endpoint
        const EP0_TX_FIFO_NUM: u8 = 0;

        // Setting the max packet size in in_ep0 updates both IN and OUT directions,
        // according to comments in the tinyusb code.
        self.usb0.in_ep0().diepctl().modify(|_, w| {
            unsafe {
                w.mps()
                    .bits(self.ep0_mps_bits)
                    .txfnum()
                    .bits(EP0_TX_FIFO_NUM)
            }
            .cnak()
            .set_bit()
            .stall()
            .clear_bit()
        });
    }

    fn process_rx_fifo(&mut self) {
        trace!("RX FIFO data ready");
        // Process entries from the FIFO until the FIFO is empty.
        loop {
            self.process_one_rx_entry();

            // If the rxflvi bit is clear, the FIFO is empty and there is nothing left to process.
            if self.usb0.gintsts().read().rxflvi().bit_is_clear() {
                return;
            }
        }
    }

    fn process_one_rx_entry(&mut self) -> u8 {
        let ctrl_word = self.usb0.grxstsp().read();

        match ctrl_word.pktsts().bits() {
            pktsts::OUT_PKT_RECEIVED => {
                let ep_index = ctrl_word.chnum().bits();
                let byte_count = ctrl_word.bcnt().bits();
                trace!(
                    "USB RX: OUT packet received on EP{}: size={}",
                    ep_index,
                    byte_count
                );
                self.receive_packet(ep_index, byte_count);
            }
            pktsts::OUT_XFER_COMPLETE => {
                let ep_index = ctrl_word.chnum().bits() as usize;
                trace!("OUT xfer complete on EP{}", ep_index);
                // Given that we only ever read a single packet at a time, we
                // currently expect to receive OUT_XFER_COMPLETE after every single
                // OUT_PKT_RECEIVED.
                self.ep_out_readers[ep_index].read_complete();
                self.ep_out_waker[ep_index].wake();
            }
            pktsts::SETUP_DONE => {
                trace!("SETUP done");
                // This entry indicates that the host has sent an IN or OUT token to start the
                // DATA phase of a control transfer.  We know that the most recently seen SETUP
                // packet is valid for this transfer, and is not going to be superseded by
                // another retransmitted SETUP.
                //
                // When we pop the SETUP_DONE packet the USB core will automatically generate a
                // stuppktrcvd OUT endpoint interrupt.  We don't do any other processing here,
                // and we wait to deliver the SETUP packet to the embassy-usb code until we
                // process the stuppktrcvd interrupt.  This is the behavior recommended by the
                // STMicro docs.
            }
            pktsts::SETUP_RECEIVED => {
                let ep_index = ctrl_word.chnum().bits();
                let byte_count = ctrl_word.bcnt().bits();
                // SETUP packets are always 8 bytes long.
                assert!(byte_count == 8);

                // Read the SETUP packet into self.ep0_setup_data.
                // We don't do anything with it yet: we wait until receiving SETUP_DONE
                // before attempting to process this data.
                //
                // The SETUP_DONE event is used to avoid processing retransmitted SETUP
                // packets.  In the case of SETUP retransmission, we may get multiple
                // SETUP_RECEIVED events before we get a SETUP_DONE.  The core
                // waits to see the next IN or OUT token after the SETUP before it sends us the
                // SETUP_DONE event.
                let rx_fifo = self.usb0.fifo(0);
                if ep_index == 0 {
                    self.ep0_setup_data[0] = rx_fifo.read().word().bits();
                    self.ep0_setup_data[1] = rx_fifo.read().word().bits();
                    trace!(
                        "SETUP received: {:?} from rx_fifo address {:p}",
                        bytemuck::cast_slice::<u32, u8>(&self.ep0_setup_data),
                        rx_fifo
                    );
                } else {
                    // We only ever configure EP0 as a control endpoint, so we don't expect to
                    // receive SETUP packets on any other endpoint.  Handle this case just in
                    // case there is a buggy host implementation that sends us garbage SETUP
                    // packets on other endpoints.  Pop the data from the FIFO and discard it.
                    warn!(
                        "received SETUP packet on unexpected endpoint EP{}",
                        ep_index
                    );
                    let _ = rx_fifo.read();
                    let _ = rx_fifo.read();
                }
            }
            pktsts::GLOBAL_OUT_NAK => {
                // Nothing to do here.
                // set_global_out_nak() will handle this when we return it.
            }
            _ => {
                // DATA_TOGGLE_ERROR and CHANNEL_HALTED should only be received in Host mode.
                // There are no other valid values.
                warn!(
                    "unexpected data in RX FIFO: {:#x}",
                    ctrl_word.pktsts().bits()
                );
            }
        }

        ctrl_word.pktsts().bits()
    }

    fn receive_packet(&mut self, ep_index: u8, byte_count: u16) {
        // Safety: the caller's buffer is valid for as long as the ReadOperation,
        // and the ReadOperation will clear self.ep_out_readers[ep_index] when it is dropped.
        // We know the buffer is valid if get_buffer() returns Some(buffer) buffer here
        let maybe_read_buf = unsafe { self.ep_out_readers[ep_index as usize].get_buffer() };
        if let Some(buf) = maybe_read_buf {
            if byte_count as usize > buf.len() {
                self.discard_rx_fifo_data(byte_count);
                self.ep_out_readers[ep_index as usize].record_buffer_overflow(byte_count as usize);
            } else {
                self.read_rx_fifo_data(&mut buf[0..byte_count as usize]);
                self.ep_out_readers[ep_index as usize].append_bytes_read(byte_count as usize);
            }
        } else {
            // This can happen if the caller aborted their read at roughly the same time as a
            // packet arrived.  This can happen if poll_bus() did not get a chance to run after a
            // packet arrived and before the ReadOperation was dropped.  Setting the NAK flag in
            // ReadOperation::drop() also takes a short period to take effect, so some read packets
            // may come in briefly after we set the NAK flag.
            //
            // Read the packet and drop it on the floor.
            //
            // TODO: it would perhaps be nice to have a separate buffer of max_packet_size
            // per OUT endpoint, so that we had a place to put this data, and could store it until
            // the next read() call and return it then.  However, even then it seems tricky for
            // callers to ensure that no data ever gets lost in the presence of timed out and
            // dropped read operations.  It's not clear to me if it's worth trying to guarantee no
            // data loss after a dropped read.
            self.discard_rx_fifo_data(byte_count);
        }
    }

    fn discard_rx_fifo_data(&mut self, byte_count: u16) {
        let mut bytes_read: u16 = 0;
        let rx_fifo = self.usb0.fifo(0);
        while bytes_read < byte_count {
            let _ = rx_fifo.read();
            bytes_read += 4;
        }
    }

    fn read_rx_fifo_data(&mut self, buf: &mut [u8]) {
        let rx_fifo = self.usb0.fifo(0);
        let whole_words = buf.len() / 4;
        if whole_words > 0 {
            if ((buf.as_ptr() as usize) & 0x3) == 0 {
                // If the buffer is word-aligned, we can read directly into it
                let buf_words: &mut [u32] = bytemuck::cast_slice_mut(&mut buf[0..whole_words * 4]);
                for word in buf_words {
                    *word = rx_fifo.read().bits();
                }
            } else {
                for chunk in buf.chunks_exact_mut(4) {
                    let word: u32 = rx_fifo.read().bits();
                    chunk.copy_from_slice(bytemuck::bytes_of(&word));
                }
            }
        }
        let remainder = buf.len() - whole_words * 4;
        if remainder > 0 {
            let word: u32 = rx_fifo.read().bits();
            buf[whole_words * 4..].copy_from_slice(&bytemuck::bytes_of(&word)[0..remainder]);
        }
    }

    fn write_to_fifo(&mut self, buf: &[u8], fifo_index: u8) {
        // TODO: it would be nicer to use DMA for reads and writes, but I haven't seen much
        // documentation on how to do DMA transfers to the USB core.   I've seen posts in the
        // Espressif forum that they plan to implement DMA transfers in the tinyusb code at some
        // point, so we could look at their implementation once that is complete.
        let fifo = self.usb0.fifo(fifo_index as usize);
        let whole_words = buf.len() / 4;
        if whole_words > 0 {
            // We could check buf.as_ptr().is_aligned_to(4), but this is nightly-only
            if ((buf.as_ptr() as usize) & 0x3) == 0 {
                // For aligned buffers, we can just use bytemuck::cast() to read the values as u32
                let buf_words: &[u32] = bytemuck::cast_slice(&buf[0..whole_words * 4]);
                for word in buf_words {
                    fifo.write(|w| w.set(*word));
                }
            } else {
                for chunk in buf.chunks_exact(4) {
                    let mut word = [0u8; 4];
                    word[0..4].copy_from_slice(chunk);
                    fifo.write(|w| w.set(bytemuck::cast(word)));
                }
            }
        }
        let remainder = buf.len() - whole_words * 4;
        if remainder > 0 {
            // Copy the final partial word
            let mut word = [0u8; 4];
            word[0..remainder].copy_from_slice(&buf[whole_words * 4..]);
            fifo.write(|w| w.set(bytemuck::cast(word)));
        }
    }

    fn process_out_ep_interrupts(&mut self) {
        let daint = self.usb0.daint().read().bits() >> 16;

        for ep_index in 0..NUM_OUT_ENDPOINTS {
            if 0 != daint & (1 << ep_index) {
                self.service_out_endpoint(ep_index);
            }
        }
    }

    fn service_out_endpoint(&mut self, ep_index: usize) {
        let doepint = if ep_index == 0 {
            self.usb0.out_ep0().doepint()
        } else {
            self.usb0.out_ep(ep_index - 1).doepint()
        };
        let ints = doepint.read();
        trace!("interrupt on OUT EP{}: {:#x}", ep_index, ints.bits());

        if ints.setup().bit() {
            trace!("process setup phase done");
            doepint.write(|w| w.setup().set_bit());
            // We only configure EP0 as a control endpoint, so we don't expect to get SETUP packets
            // on any other endpoint, unless maybe the host is buggy?
            if ep_index == 0 {
                self.ep0_setup_ready = true;
                self.ep_out_waker[0].wake();
            }
        }

        if ints.xfercompl().bit_is_set() {
            trace!("RX complete on EP{}", ep_index);
            doepint.write(|w| w.xfercompl().set_bit());
            // Note: at the moment we don't do any processing of the xfercompl interrupt,
            // but we will need to perform processing here in the future if we want to support
            // reading more than 1 packet at a time.
        }
    }

    fn process_in_ep_interrupts(&mut self) {
        let daint = self.usb0.daint().read().bits();
        trace!("process in EP interrupts: daint={:#x}", daint);

        for ep_index in 0..NUM_OUT_ENDPOINTS {
            if 0 != daint & (1 << ep_index) {
                self.service_in_endpoint(ep_index);
            }
        }
    }

    fn service_in_endpoint(&mut self, ep_index: usize) {
        let diepint = if ep_index == 0 {
            self.usb0.in_ep0().diepint()
        } else {
            self.usb0.in_ep(ep_index - 1).diepint()
        };
        let ints = diepint.read();
        trace!("interrupt on IN EP{}: {:#x}", ep_index, ints.bits());

        // If the TX FIFO is empty, disable this interrupt for this endpoint and then wake
        // any endpoint handling code that may be pending.  (In particular, poll_in_ep_transmit()
        // cares about this if it is pending.)
        if ints.txfemp().bit_is_set() {
            trace!("TX FIFO empty on EP{}", ep_index);
            // Note that we don't need to clear the txfemp bit in diepint.
            // This bit is read-only, and is always asserted when the TX FIFO is empty.
            self.usb0.diepempmsk().modify(|r, w| {
                let new_bits = r.d_ineptxfempmsk().bits() & !(1 << ep_index);
                unsafe { w.d_ineptxfempmsk().bits(new_bits) }
            });
        }

        // The timeout interrupt is only generated for control endpoints,
        // if a timeout occurs attempting to send an IN token.
        if ints.timeout().bit_is_set() {
            warn!("IN transaction timeout on EP{}", ep_index);
            diepint.write(|w| w.timeout().set_bit());

            // TODO: we probably should flush the TX FIFO after a timeout.
            // We would need to:
            // - Set the NAK flag for this endpoint
            // - Wait for the NAK flag to take effect
            // - Wait for grstctl.ahbidle
            // - Modify grstctl to set txfnum to ep_index and set the txfflsh bit
            // - Wait for the txflsh bit to clear.
            // - Clear the NAK flag for the endpoint

            // TODO: the ControlPipe task will presumably be waiting in endpoint.write() (to try
            // and write another packet) or endpoint.read() (to read acknowledgement from the
            // host).  We ideally should store some sort of flag somewhere to inform it that the
            // control transfer has failed, so it can notice this when it polls when we wake it up.
        }
        if ints.xfercompl().bit_is_set() {
            trace!("TX complete on EP{}", ep_index);
            diepint.write(|w| w.xfercompl().set_bit());
            // Note: at the moment we don't do any processing of the xfercompl interrupt,
            // but we will need to perform processing here in the future if we want to support
            // writing more than 1 packet at a time.
        }

        // Wake the ep_in_waker on any endpoint IN interrupt.
        // - poll_in_ep_transmit() cares about waking on d_txfemp (more room available in the TX
        //   FIFO) and d_xfercompl (a transfer has finished and a new one can now be started).
        self.ep_in_waker[ep_index].wake();
    }

    fn disable_all_endpoints_on_vbus_removed(&mut self) {
        self.flush_fifos_on_reset();

        for ep_index in 1..NUM_IN_ENDPOINTS {
            self.usb0
                .in_ep(ep_index - 1)
                .diepctl()
                .modify(|_, w| w.usbactep().clear_bit());
        }
        for ep_index in 1..NUM_OUT_ENDPOINTS {
            self.usb0
                .out_ep(ep_index - 1)
                .doepctl()
                .modify(|_, w| w.usbactep().clear_bit());
        }
    }

    fn nak_all_out_endpoints(&mut self) {
        self.usb0
            .out_ep0()
            .doepctl()
            .modify(|_, w| w.snak().set_bit());
        for n in 1..7 {
            self.usb0
                .out_ep(n - 1)
                .doepctl()
                .modify(|_, w| w.snak().set_bit());
        }
    }

    fn flush_fifos_on_reset(&mut self) {
        self.stop_all_in_endpoints();
        self.stop_all_out_endpoints();
    }

    fn stop_all_in_endpoints(&mut self) {
        trace!("stop_all_in_endpoints");

        // This is based on documentation for the Synopsys USB cores in STMicro chips.
        // See "Transfer Stop Programming for IN endpoints" in section 34.17.6 of the STM32F405
        // reference manual.

        // AHB must be idle before we can disable actively writing IN endpoints
        while self.usb0.grstctl().read().ahbidle().bit_is_clear() {}

        // Set the EPDIS flag on all endpoints, to stop any in-progress writes
        self.usb0
            .in_ep0()
            .diepctl()
            .modify(|_, w| w.epdis().set_bit().snak().set_bit());
        for n in 1..NUM_IN_ENDPOINTS {
            self.usb0
                .in_ep(n - 1)
                .diepctl()
                .modify(|_, w| w.epdis().set_bit().snak().set_bit());
        }

        // Wait for the disable to take effect
        // (It's unfortunate that we need to busy wait here)
        while self
            .usb0
            .in_ep0()
            .diepint()
            .read()
            .epdisbld()
            .bit_is_clear()
        {}
        for n in 1..NUM_IN_ENDPOINTS {
            while self
                .usb0
                .in_ep(n - 1)
                .diepint()
                .read()
                .epdisbld()
                .bit_is_clear()
            {}
        }

        // Flush all TX FIFOs.  Flushing with fifo number 0x10 flushes all FIFOs.
        self.flush_tx_fifo(0x10);
    }

    fn flush_tx_fifo(&mut self, fifo_number: u8) {
        self.usb0
            .grstctl()
            .modify(|_, w| unsafe { w.txfnum().bits(fifo_number) }.txfflsh().set_bit());
        // Busy loop for the flush to complete.
        while self.usb0.grstctl().read().txfflsh().bit_is_set() {}
    }

    fn stop_all_out_endpoints(&mut self) {
        trace!("stop_all_out_endpoints");
        // This is based on documentation for the Synopsys USB cores in STMicro chips.
        // See "Transfer Stop Programming for OUT Endpoints" in section 34.17.5 of the STM32F405
        // reference manual.

        // 1. Enable reads on all OUT endpoints
        self.usb0
            .out_ep0()
            .doepctl()
            .modify(|_, w| w.epena().set_bit());
        for n in 1..NUM_OUT_ENDPOINTS {
            self.usb0
                .out_ep(n - 1)
                .doepctl()
                .modify(|_, w| w.epena().set_bit());
        }

        // 2. Flush the RX FIFO:
        // 2a. Wait for grstctl.ahbidle to be set
        //
        // (It seems a little surprising to me that the STMicro docs recommend flushing the RX FIFO
        // before setting the global OUT NAK flag?)
        while self.usb0.grstctl().read().ahbidle().bit_is_clear() {}

        // 2b. Set grstctl.rxfflsh
        self.usb0.grstctl().modify(|_, w| w.rxfflsh().set_bit());
        // 2c. poll grstctl.rxfflsh until it is clear.  If it isn't set after 10ms, retry 2b once.
        if !self.wait_for_rxfflsh() {
            self.usb0.grstctl().modify(|_, w| w.rxfflsh().set_bit());
            if !self.wait_for_rxfflsh() {
                error!("timed out waiting for RX FIFO flush");
                // continue anyway here?
            }
        }

        // 3. Set global OUT NAK (sgonak)
        // 4. Confirm that the gnokeff bit is set in gintsts
        self.set_global_out_nak();

        // 5. Stop reads on all OUT endpoints by setting epdis and snak in doepctlN
        //
        // Hmm, the esp32s3 SVD lists the EPDIS bits as readable but not writable.
        // The ESP32 tinyusb code does set them though.  Let's do the same.
        // This unfortunately means we need to manually set these bits.
        const EPDIS: u32 = 1 << 30;
        const SNAK: u32 = 1 << 27;
        for n in 1..NUM_OUT_ENDPOINTS {
            self.usb0
                .out_ep(n - 1)
                .doepctl()
                .modify(|r, w| unsafe { w.bits(r.bits() | EPDIS | SNAK) });
        }
        // Endpoint 0 cannot be disabled, but set the NAK flag for it.
        self.usb0
            .out_ep0()
            .doepctl()
            .modify(|_, w| w.snak().set_bit());

        // 6. wait for the epdis interrupt in doepintN for all endpoints
        for n in 1..NUM_OUT_ENDPOINTS {
            while self
                .usb0
                .out_ep(n - 1)
                .doepint()
                .read()
                .epdisbld()
                .bit_is_clear()
            {}
        }

        // Exit global OUT NAK mode
        self.usb0.dctl().modify(|_, w| w.cgoutnak().set_bit());
    }

    fn set_global_out_nak(&mut self) {
        trace!("set global OUT NAK");

        // Set the global OUT NAK bit
        self.usb0.dctl().modify(|_, w| w.sgoutnak().set_bit());

        // Process entries from the RX FIFO until we see the GLOBAL_OUT_NAK effective control word.
        loop {
            while self.usb0.gintsts().read().rxflvi().bit_is_clear() {}
            let ctrl_word = self.process_one_rx_entry();
            if ctrl_word == pktsts::GLOBAL_OUT_NAK {
                break;
            }
        }

        // Confirm that the global OUT NAK is now in effect
        while self.usb0.gintsts().read().goutnakeff().bit_is_clear() {}
    }

    fn wait_for_rxfflsh(&mut self) -> bool {
        use embassy_time::{Duration, Instant};
        let start = Instant::now();
        const TIMEOUT: Duration = Duration::from_millis(10);
        loop {
            if self.usb0.grstctl().read().rxfflsh().bit_is_clear() {
                return true;
            }
            let now = Instant::now();
            if now - start > TIMEOUT {
                return false;
            }
        }
    }

    pub fn enable_in_ep(&mut self, ep_index: usize) {
        trace!("enabling IN EP{}", ep_index);
        assert_ne!(ep_index, 0); // EP0 is always enabled
        assert!(ep_index < NUM_IN_ENDPOINTS);
        let ep_in = self.usb0.in_ep(ep_index - 1);

        let config = match self.ep_in_config[ep_index] {
            None => {
                // This endpoint isn't configured.
                // It would be nice if we could return an error here.
                return;
            }
            Some(cfg) => cfg,
        };

        ep_in.diepctl().modify(|_, w| {
            unsafe {
                w.mps()
                    .bits(config.max_packet_size)
                    .eptype()
                    .bits(config.ep_type as u8)
                    .txfnum()
                    .bits(config.tx_fifo)
            }
            .usbactep()
            .set_bit();
            if config.ep_type != EndpointType::Isochronous {
                w.setd0pid().set_bit();
            }
            w
        });

        // Enable interrupts for this endpoint
        self.usb0.daintmsk().modify(|r, w| {
            let bits = r.bits() | (1 << ep_index);
            unsafe { w.bits(bits) }
        });

        // Wake the endpoint, in case some task is waiting in poll_in_ep_enabled()
        self.ep_in_waker[ep_index].wake();
    }

    pub fn enable_out_ep(&mut self, ep_index: usize) {
        trace!("enabling OUT EP{}", ep_index);
        assert_ne!(ep_index, 0); // EP0 is always enabled
        assert!(ep_index < NUM_OUT_ENDPOINTS);
        let ep_out = self.usb0.out_ep(ep_index - 1);

        let config = match self.ep_out_config[ep_index] {
            None => {
                // This endpoint isn't configured.
                // It would be nice if we could return an error here.
                return;
            }
            Some(cfg) => cfg,
        };

        trace!("enabling IN EP{}", ep_index);
        ep_out.doepctl().modify(|_, w| {
            w.eptype()
                .bits(config.ep_type as u8)
                .mps()
                .bits(config.max_packet_size)
                .usbactep()
                .set_bit();
            if config.ep_type != EndpointType::Isochronous {
                w.setd0pid().set_bit();
            }
            w
        });

        // Enable interrupts for this endpoint
        self.usb0.daintmsk().modify(|r, w| {
            let bits = r.bits() | (1 << (16 + ep_index));
            unsafe { w.bits(bits) }
        });

        // Wake the endpoint, in case some task is waiting in poll_out_ep_enabled()
        self.ep_out_waker[ep_index].wake();
    }

    pub fn disable_in_ep(&mut self, ep_index: usize) {
        assert_ne!(ep_index, 0); // EP0 is always enabled
        assert!(ep_index < NUM_IN_ENDPOINTS);

        // Note: this method unfortunately has several busy loops.  It would be nicer if
        // Bus::endpoint_set_enabled() were an async method (although we would require some more
        // complicated state management to deal with this).

        // Clear the usbactep bit, and set the NAK flag
        let ep_in = self.usb0.in_ep(ep_index - 1);
        let mut write_in_progress = false;
        ep_in.diepctl().modify(|r, w| {
            write_in_progress = r.epena().bit_is_set();
            w.usbactep().clear_bit().snak().set_bit()
        });

        // If there was a write in progress on the endpoint, we have to abort it.
        // If not, we are done.
        if !write_in_progress {
            return;
        }

        // This is based on documentation for the Synopsys USB cores in STMicro chips.
        // See "IN endpoint disable" in section 34.17.6 of the STM32F405 reference manual.

        // AHB must be idle before we can disable IN endpoints
        while self.usb0.grstctl().read().ahbidle().bit_is_clear() {}

        // Wait for the NAK flag to take effect on this endpoint.
        while ep_in.diepint().read().inepnakeff().bit_is_clear() {}

        // It seems like it might be okay to return early here if diepctl.epena is no longer set
        // at this point, if the endpoint finished the write before the NAK flag took effect?
        // For now we unconditionally continue setting the EPDIS flag.

        // Set the EPDIS and SNAK flags.
        ep_in
            .diepctl()
            .modify(|_, w| w.epdis().set_bit().snak().set_bit());
        // Wait for the disable to take effect
        while ep_in.diepint().read().epdisbld().bit_is_clear() {}

        // Flush any remaining data in the TX FIFO
        let fifo_number = match self.ep_in_config[ep_index] {
            Some(cfg) => cfg.tx_fifo,
            None => return, // shouldn't happen for valid endpoints that were actively writing
        };
        self.flush_tx_fifo(fifo_number);
    }

    pub fn disable_out_ep(&mut self, ep_index: usize) {
        assert_ne!(ep_index, 0); // EP0 is always enabled
        assert!(ep_index < NUM_IN_ENDPOINTS);

        let ep_out = self.usb0.out_ep(ep_index - 1);
        let mut read_in_progress = false;
        ep_out.doepctl().modify(|r, w| {
            read_in_progress = r.epena().bit_is_set();
            w.usbactep().clear_bit().snak().set_bit()
        });

        if !read_in_progress {
            return;
        }

        // This is based on documentation for the Synopsys USB cores in STMicro chips.
        // See "Disabling an OUT endpoint" in section 34.17.6 of the STM32F405 reference manual.

        // The global OUT NAK flag must be set before disabling any OUT endpoint.
        self.set_global_out_nak();
        assert_ne!(ep_index, 0); // EP0 is always enabled
        assert!(ep_index < NUM_OUT_ENDPOINTS);

        // Set the EPDIS and SNAK flags
        let ep_out = self.usb0.out_ep(ep_index - 1);
        ep_out
            .doepctl()
            .modify(|_, w| w.epdis().set_bit().snak().set_bit());

        // Wait for an epdisbld interrupt to confirm that the disable has taken effect.
        while ep_out.doepint().read().epdisbld().bit_is_clear() {}

        // Exit global OUT NAK mode
        self.usb0.dctl().modify(|_, w| w.cgoutnak().set_bit());
    }

    pub fn stall_in_ep(&self, ep_index: usize) {
        if ep_index == 0 {
            self.usb0
                .in_ep0()
                .diepctl()
                .modify(|_, w| w.stall().set_bit());
        } else {
            assert!(ep_index < NUM_IN_ENDPOINTS);
            self.usb0
                .in_ep(ep_index - 1)
                .diepctl()
                .modify(|_, w| w.stall().set_bit());
        }

        // Note: the embassy-stm32 implementation wakes the endpoint waker here,
        // but this doesn't seem necessary here to me?  I don't see any place where polling code
        // would care about the stall flag changing.
    }

    pub fn stall_out_ep(&self, ep_index: usize) {
        if ep_index == 0 {
            self.usb0
                .out_ep0()
                .doepctl()
                .modify(|_, w| w.stall().set_bit());
        } else {
            assert!(ep_index < NUM_OUT_ENDPOINTS);
            self.usb0
                .out_ep(ep_index - 1)
                .doepctl()
                .modify(|_, w| w.stall().set_bit());
        }

        // Note: the embassy-stm32 implementation wakes the endpoint waker here,
        // but this doesn't seem necessary here to me?  I don't see any place where polling code
        // would care about the stall flag changing.
    }

    pub fn is_in_ep_stalled(&self, ep_index: usize) -> bool {
        if ep_index == 0 {
            self.usb0.in_ep0().diepctl().read().stall().bit_is_set()
        } else {
            assert!(ep_index < NUM_IN_ENDPOINTS);
            self.usb0
                .in_ep(ep_index - 1)
                .diepctl()
                .read()
                .stall()
                .bit_is_set()
        }
    }

    pub fn is_out_ep_stalled(&self, ep_index: usize) -> bool {
        if ep_index == 0 {
            self.usb0.out_ep0().doepctl().read().stall().bit_is_set()
        } else {
            assert!(ep_index < NUM_OUT_ENDPOINTS);
            self.usb0
                .out_ep(ep_index)
                .doepctl()
                .read()
                .stall()
                .bit_is_set()
        }
    }
}

pub(crate) async fn start_read_op<'b, 'd>(
    state: &'d RefCell<State<'d>>,
    ep_index: usize,
    buf: &'b mut [u8],
) -> Result<ReadOperation<'b, 'd>, EndpointError> {
    poll_fn(|cx| {
        let mut state = state.borrow_mut();
        state.poll_out_ep_start_read(cx, ep_index, buf)
    })
    .await?;

    Ok(ReadOperation {
        state,
        buf,
        ep_index,
    })
}

/// ReadOperation exists for the duration of an in-progress read attempt.
///
/// This allows us to be notified when it is dropped, and ask the hardware to stop accepting OUT
/// packets if our caller cancels the read.
pub(crate) struct ReadOperation<'b, 'd> {
    state: &'d RefCell<State<'d>>,
    buf: &'b mut [u8],
    ep_index: usize,
}

impl<'b, 'd> ReadOperation<'b, 'd> {
    pub(crate) async fn do_read(&mut self) -> Result<usize, EndpointError> {
        poll_fn(|cx| {
            let mut state = self.state.borrow_mut();
            state.poll_out_ep_read_complete(cx, self.ep_index)
        })
        .await
    }
}

impl<'b, 'd> Drop for ReadOperation<'b, 'd> {
    fn drop(&mut self) {
        // ReadOperation objects are owned by higher level code, and should never be
        // dropped during any operation where we have already borrowed self.state
        self.state
            .borrow_mut()
            .read_operation_dropped(self.ep_index, self.buf);
    }
}

#[derive(Debug, Eq, PartialEq)]
enum ReadStatus {
    Idle,
    Pending,
    Complete,
    BufferOverflow,
}

struct ReadBuf {
    ptr: *mut u8,
    capacity: usize,
    // TODO: move bytes_read and status into a separate caller-owned struct,
    // so we can start a new read as soon as an old one completes, instead of having
    // to wait for the old ReadOperation to be dropped.
    bytes_read: usize,
    status: ReadStatus,
}

#[derive(Debug)]
struct EndpointBusy;

impl ReadBuf {
    fn new() -> Self {
        Self {
            ptr: core::ptr::null_mut(),
            capacity: 0,
            bytes_read: 0,
            status: ReadStatus::Idle,
        }
    }

    fn set_reader(&mut self, buf: &mut [u8]) -> Result<(), EndpointBusy> {
        if self.ptr != core::ptr::null_mut() {
            return Err(EndpointBusy);
        }
        self.ptr = buf.as_mut_ptr();
        self.capacity = buf.len();
        self.bytes_read = 0;
        self.status = ReadStatus::Pending;
        Ok(())
    }

    // Returns true if this ReadOperation was the current reader.
    fn read_operation_dropped(&mut self, buf: &mut [u8]) -> bool {
        if self.ptr != buf.as_mut_ptr() {
            // This is presumably an old ReadOperation that already finished.
            // We may have already started a new read, so don't change the current state.
            return false;
        }
        self.ptr = core::ptr::null_mut();
        self.capacity = 0;
        self.bytes_read = 0;
        self.status = ReadStatus::Idle;
        true
    }

    fn append_bytes_read(&mut self, bytes_read: usize) {
        assert_eq!(self.status, ReadStatus::Pending);
        self.bytes_read += bytes_read;
    }

    fn record_buffer_overflow(&mut self, bytes_read: usize) {
        assert_eq!(self.status, ReadStatus::Pending);
        warn!(
            "read buffer overflow: caller asked for {} bytes, but host sent {}",
            self.capacity,
            self.bytes_read + bytes_read
        );
        self.status = ReadStatus::BufferOverflow;
    }

    fn read_complete(&mut self) {
        assert_eq!(self.status, ReadStatus::Pending);
        match self.status {
            ReadStatus::Pending => {
                self.status = ReadStatus::Complete;
            }
            ReadStatus::BufferOverflow => {
                // Leave ReadStatus as BufferOverflow after an error
            }
            ReadStatus::Complete => {
                panic!("read complete called twice");
            }
            ReadStatus::Idle => {
                panic!("no pending read");
            }
        }
        self.status = ReadStatus::Complete;
    }

    fn check_read_complete(&mut self) -> Option<Result<usize, EndpointError>> {
        match self.status {
            ReadStatus::Pending => None,
            ReadStatus::Complete => Some(Ok(self.bytes_read)),
            ReadStatus::BufferOverflow => Some(Err(EndpointError::BufferOverflow)),
            ReadStatus::Idle => {
                panic!("no pending read");
            }
        }
    }

    // Safety: the lifetime of the returned buf reference isn't actually static;
    // It is the lifetime of the current ReadOperation object.  Our caller has to be
    // careful never to store this reference and always use it before yielding back to the
    // executor.
    unsafe fn get_buffer(&self) -> Option<&'static mut [u8]> {
        if self.ptr == core::ptr::null_mut() {
            return None;
        }
        Some(core::slice::from_raw_parts_mut(self.ptr, self.capacity))
    }
}

mod pktsts {
    // I believe 0 is returned if we attempt to read from the RX FIFO when it is empty.
    pub(crate) const GLOBAL_OUT_NAK: u8 = 1;
    pub(crate) const OUT_PKT_RECEIVED: u8 = 2;
    pub(crate) const OUT_XFER_COMPLETE: u8 = 3;
    pub(crate) const SETUP_DONE: u8 = 4;
    pub(crate) const _DATA_TOGGLE_ERROR: u8 = 5; // Host mode only
    pub(crate) const SETUP_RECEIVED: u8 = 6;
    pub(crate) const _CHANNEL_HALTED: u8 = 7; // Host mode only
}

// We could use the bitflags crate, but it didn't seem worth pulling in more dependencies.
mod bus_event_flag {
    pub(crate) const POWER_DETECTED: u8 = 0x01;
    pub(crate) const POWER_REMOVED: u8 = 0x02;
    pub(crate) const RESET: u8 = 0x04;
    pub(crate) const SUSPEND: u8 = 0x08;
    pub(crate) const RESUME: u8 = 0x10;
}

// Speed bits for use in the usb0.dcfg register.
// These are not documented in the SVD or ESP-IDF headers, but are used in the tinyusb code and
// discussed in documentation for other Synopsys USB cores.  The tinyusb code references issue
// IDF-1476 about the fact that ESP-IDF does not define these values.  However, comments in the
// tinyusb code also appear to indicate that the Espressif USB core may be always fixed to full
// speed.
mod devspd {
    // pub(crate) const _HIGH_30MHZ: u32 = 0;
    // pub(crate) const _FULL_30MHZ: u32 = 1;
    // pub(crate) const _LOW_6MHZ: u32 = 2;
    pub(crate) const FULL_48MHZ: u32 = 3;
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct InEndpointConfig {
    pub(crate) ep_type: EndpointType,
    pub(crate) tx_fifo: u8,
    pub(crate) max_packet_size: u16,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct OutEndpointConfig {
    pub(crate) ep_type: EndpointType,
    pub(crate) max_packet_size: u16,
}
