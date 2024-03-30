use crate::driver::FifoSettings;
use crate::regs::{
    get_usb0_register_block, Usb0RegisterBlock, NUM_IN_ENDPOINTS, NUM_OUT_ENDPOINTS,
};
use crate::Config;
use core::cell::RefCell;
use core::mem::MaybeUninit;
use core::task::Poll;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver::{EndpointError, Event};
use esp_hal::peripherals::Interrupt;
use log::{error, trace, warn};

// The global State singleton.
// It's unfortunate that this has to be global, but the embassy-usb API doesn't provide us with any
// place to store state that needs to be shared by the Bus, ControlPipe, and Endpoints.  (It would
// seem nicer if Driver::start() instead returned a single State object, instead of returning
// separate Bus and ControlPipe objects with no place for shared state.)
//
// We only ever access this STATE variable in Driver::start(), when it is first initialized.  This
// would make it easy to remove this static global variable in the future if the embassy-usb APIs
// were updated to allow returning a State object directly to users.
//
// We use a RefCell to validate that access from the Bus, ControlPipe, and Endpoints are safe.  The
// main rule is that we only borrow the RefCell within poll_fn() calls, so that it is only borrowed
// for the duration of a single poll check, and we always release it before yielding back to the
// executor.
//
// TODO: for optimization purposes, we could perhaps replace the RefCell with a no-op wrapper that
// doesn't actually do checks in release builds.
pub(crate) static mut STATE: MaybeUninit<RefCell<State>> = MaybeUninit::zeroed();

// The BUS_WAKER, used by the interrupt handler to wake the main task to perform work.
// This has to be global so it can be accessed by the USB interrupt handler.
pub(crate) static BUS_WAKER: AtomicWaker = AtomicWaker::new();

// This structure keeps track of common shared state needed by the Bus, ControlPipe, and endpoints.
pub(crate) struct State {
    // TODO: it would perhaps be nicer to remove the usb0 member variable to save space.
    // We shouldn't really need to store this pointer, and we should be able to just access
    // the pointer as a constant.  However, I don't believe esp_hal exposes the pointer value as a
    // constant.  We perhaps should just define the correct target-specific address in regs.rs
    usb0: &'static Usb0RegisterBlock,

    config: Config,
    ep0_mps_bits: u8,
    initialized: bool,

    ep0_setup_data: [u32; 2],
    ep0_setup_ready: bool,
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

impl State {
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
    pub fn new(config: Config) -> Self {
        Self {
            usb0: unsafe { get_usb0_register_block() },
            config,
            ep0_mps_bits: 0,
            initialized: false,
            ep0_setup_data: [0; 2],
            ep0_setup_ready: false,
            ep_in_waker: core::array::from_fn(|_| AtomicWaker::new()),
            ep_out_waker: core::array::from_fn(|_| AtomicWaker::new()),
            ep_out_readers: core::array::from_fn(|_| ReadBuf::new()),
        }
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
        Ok(())
    }

    pub fn init_bus(&mut self, fifo_settings: &FifoSettings) {
        // Ensure the data line pull-up is disabled as we perform initialization.
        self.usb0.dctl().modify(|_, w| w.sftdiscon().set_bit());

        // Set the speed, and also configure the device to send a stall on receipt of any
        // non-zero length OUT packet while we are still initializing the settings.
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

        // TODO: test enabling this code and see if it speeds up the enumeration process
        /*
        // Accept out packets.
        // (Unclear if we really need to do this: tinyusb doesn't, and the STmicro docs don't really
        // mention clearing it.)
        self.usb0.dcfg().modify(|_, w| w.nzstsouthshk().clear_bit());
        */

        // Enable the data line pull-up to connect the bus
        self.usb0.dctl().modify(|_, w| w.sftdiscon().clear_bit());
    }

    fn init_fifos(&mut self, settings: &FifoSettings) {
        // Sanity check that things aren't currently in use
        assert!(
            {
                let r = self.usb0.daintmsk().read();
                r.outepmsk0().bit_is_clear() && r.inepmsk0().bit_is_clear()
            },
            "init_fifos() called after EP0 has already been configured"
        );
        trace!("init_fifos: {:?}", settings);

        // Note: during initialization we assume that the USB bus has not been used yet,
        // so the USB core should not be currently accessing the RX or TX FIFOs.
        // We could call self.flush_fifos_on_reset() here if we wanted to handle the case where
        // someone else has already used the USB peripheral before they gave ownership of it to us.

        // Configure the RX FIFO size
        self.usb0
            .grxfsiz()
            .write(|w| unsafe { w.rxfdep().bits(settings.rx_num_words) });

        let mut offset_words = settings.rx_num_words;

        // Configure TX FIFOs
        // TX FIFO 0 is configured with usb0.gnptxfsiz()
        self.usb0.gnptxfsiz().write(|w| unsafe {
            w.nptxfstaddr()
                .bits(offset_words)
                .nptxfdep()
                .bits(settings.tx_num_words[0])
        });
        offset_words += settings.tx_num_words[0];

        // TX FIFOs 1-4 are configured with usb0.dieptxfN
        for n in 0..4 {
            self.usb0.dieptxf(n).write(|w| unsafe {
                w.inep1txfstaddr()
                    .bits(offset_words)
                    .inep1txfdep()
                    .bits(settings.tx_num_words[n + 1])
            });
            offset_words = offset_words + settings.tx_num_words[0];
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
            .modify(|_, w| unsafe { w.supcnt0().bits(3) });

        trace!(
            "SETUP ready: {:?}",
            bytemuck::cast_slice::<u32, u8>(&self.ep0_setup_data)
        );
        Poll::Ready(bytemuck::cast(self.ep0_setup_data))
    }

    pub fn poll_out_ep_enabled(
        &mut self,
        cx: &mut core::task::Context,
        ep_index: usize,
    ) -> Poll<()> {
        self.ep_out_waker[ep_index].register(cx.waker());
        let doepctl = self.usb0.out_ep(ep_index).doepctl().read();
        if doepctl.usbactep1().bit_is_set() {
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
        self.ep_out_waker[ep_index].register(cx.waker());

        let out_ep = self.usb0.out_ep(ep_index);
        let doepctl = out_ep.doepctl().read();
        if doepctl.usbactep1().bit_is_clear() {
            trace!("OUT EP{} RX: endpoint is disabled", ep_index);
            return Poll::Ready(Err(EndpointError::Disabled));
        }

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
        // We read max_packet_size from doepctl just to avoid the caller having to pass it in.
        let max_packet_size = out_ep.doepctl().read().mps1().bits();
        let read_size = max_packet_size;
        assert!(read_size as usize >= buf.len());
        out_ep.doeptsiz().modify(|r, w| {
            // Doh.  The DOEPTSIZ register definition in the esp-pacs crate is pretty broken,
            // and has wrong field widths for pktcnt and xfersize.  Manually construct the value
            // for now.
            // TODO: update this if https://github.com/esp-rs/esp-pacs/pull/213
            // is merged.
            let value = (r.bits() & 0xe0000000) | (read_size as u32) | ((NUM_PACKETS as u32) << 19);
            unsafe { w.bits(value) }
        });
        // Clear the NAK flag and enable endpoint to allow the hardware to start reading.
        out_ep
            .doepctl()
            .modify(|_, w| w.cnak1().set_bit().epena1().set_bit());

        Poll::Ready(Ok(()))
    }

    fn poll_out_ep_read_complete(
        &mut self,
        cx: &mut core::task::Context,
        ep_index: usize,
    ) -> Poll<Result<usize, EndpointError>> {
        trace!("OUT EP{} RX: polling for read complete", ep_index);
        self.ep_out_waker[ep_index].register(cx.waker());

        if let Some(result) = self.ep_out_readers[ep_index].check_read_complete() {
            Poll::Ready(result)
        } else {
            Poll::Pending
        }
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
            self.usb0.out_ep(ep_index).doepctl().modify(|r, w| {
                // TODO: update this if https://github.com/esp-rs/esp-pacs/pull/213
                // is merged.
                // w.epdis1().set_bit().snak1().set_bit()
                let value = r.bits() | 0x48000000;
                unsafe { w.bits(value) }
            });
        }
    }

    pub fn poll_in_ep_enabled(
        &mut self,
        cx: &mut core::task::Context,
        ep_index: usize,
    ) -> Poll<()> {
        self.ep_in_waker[ep_index].register(cx.waker());
        let diepctl = self.usb0.in_ep(ep_index).diepctl().read();
        if diepctl.d_usbactep1().bit_is_set() {
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
        fifo_index: usize,
        buf: &[u8],
    ) -> Poll<Result<(), EndpointError>> {
        self.ep_in_waker[ep_index].register(cx.waker());

        let ep_regs = self.usb0.in_ep(ep_index);
        let diepctl = ep_regs.diepctl().read();
        if diepctl.d_usbactep1().bit_is_clear() {
            trace!("IN EP{} TX: endpoint is disabled", ep_index);
            return Poll::Ready(Err(EndpointError::Disabled));
        }
        if diepctl.d_epena1().bit_is_set() {
            trace!("IN EP{} TX: endpoint is busy", ep_index);
            return Poll::Pending;
        }

        let dtxfsts = ep_regs.dtxfsts().read();
        trace!(
            "IN EP{} poll write: diepctl {:08x} dtxfsts {:08x}",
            ep_index,
            diepctl.bits(),
            dtxfsts.bits()
        );

        // Check for available TX FIFO space.
        // (I suspect that it shouldn't be possible for this check to fail, since we only ever
        // write a single packet at a time and we wait for the endpoint to become idle before
        // starting the next write.)
        if buf.len() > 0 {
            let size_words = (buf.len() + 3) / 4;

            let fifo_space = dtxfsts.d_ineptxfspcavail1().bits() as usize;
            if size_words > fifo_space {
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
        // the EndpointIn::write() cod has already verified that buf.len() is within this limit.
        const NUM_PACKETS: u8 = 1;

        // First inform the core of how much data we want to send, and how many packets.
        ep_regs.dieptsiz().write(|w| unsafe {
            w.d_pktcnt1().bits(NUM_PACKETS);
            w.d_xfersize1().bits(buf.len() as _)
        });
        // Now enable the endpoint to start writing, and clear the NAK bit.
        ep_regs
            .diepctl()
            .modify(|_, w| w.d_cnak1().set_bit().d_epena1().set_bit());

        // Finally write all data to the FIFO
        self.write_to_fifo(buf, fifo_index);
        Poll::Ready(Ok(()))
    }

    pub fn poll_bus(&mut self, cx: &mut core::task::Context) -> Poll<Event> {
        // Re-register to be woken again.
        BUS_WAKER.register(cx.waker());
        trace!("poll_bus starting");

        // Call process_interrupts() to process interrupts from the gintsts register.
        // This will clear each interrupt in gintsts as it processes them, so we won't get
        // notified again about ones that have been processed.  It may return an event before
        // processing all interrupts, in which case remaining ones will still be set in gintsts
        // so the interrupt handler should fire again immediately to re-process these once we
        // re-enable the interrupt mask below.
        let result = self.process_interrupts();

        // Re-enable the interrupt mask
        self.enable_intmask();
        trace!("USB poll returning {:?}", result);
        result
    }

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
            // .oepintmsk()
            // .set_bit()
        });
    }

    fn process_interrupts(&mut self) -> Poll<Event> {
        if !self.initialized {
            self.initialized = true;

            // If we don't have a dedicated pin to detect VBUS,
            // return a PowerDetected event immediately on startup.
            if let None = self.config.vbus_detection_pin {
                return Poll::Ready(Event::PowerDetected);
            }
        }

        let ints = self.usb0.gintsts().read();
        trace!("USB poll bus: ints={:#x}", ints.bits());

        if ints.modemis().bit() {
            error!("USB mode mismatch");
            self.usb0.gintsts().write(|w| w.modemis().set_bit());
        }

        if ints.sessreqint().bit() {
            trace!("vbus detected");
            self.usb0.gintsts().write(|w| w.sessreqint().set_bit());
            if self.config.vbus_detection_pin.is_some() {
                return Poll::Ready(Event::PowerDetected);
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
                    self.disable_all_endpoints();
                    return Poll::Ready(Event::PowerRemoved);
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
            return Poll::Ready(Event::Reset);
        }

        if ints.usbsusp().bit() {
            trace!("USB suspend");
            self.usb0.gintsts().write(|w| w.usbsusp().set_bit());
            return Poll::Ready(Event::Suspend);
        }

        if ints.wkupint().bit() {
            trace!("USB resume");
            self.usb0.gintsts().write(|w| w.wkupint().set_bit());
            return Poll::Ready(Event::Resume);
        }

        // I wonder if RXFLVI is a typo in the esp32s2/s3 SVD?  This is normally called RXFLVL.
        if ints.rxflvi().bit() {
            self.process_rx_fifo();
        }

        // We don't currently register for OUT endpoint interrupts.
        // Maybe we will need to in the future if we wanted to fix the busy loop in
        // stop_all_out_endpoints(), and wait for the endpoint disable interrupt.
        /*
        if ints.oepint().bit() {
            self.process_out_ep_interrupts();
        }
        */

        if ints.iepint().bit() {
            self.process_in_ep_interrupts();
        }

        Poll::Pending
    }

    fn process_reset(&mut self) {
        self.nak_all_out_endpoints();

        // Clear the device address
        self.usb0
            .dcfg()
            .modify(|_, w| unsafe { w.devaddr().bits(0) });

        // Disable all endpoint interrupts
        self.usb0.daintmsk().write(|w| unsafe { w.bits(0) });
        self.usb0.doepmsk().write(|w| unsafe { w.bits(0) });
        self.usb0.diepmsk().write(|w| unsafe { w.bits(0) });

        self.flush_fifos_on_reset();

        // TODO: Reset all software endpoint state.  Do we need to do anything here, or will
        // embassy-usb do this on the Reset event after the enumdone interrupt?

        // Note: we don't change the FIFO configuration (grxfsiz, gnptxfsiz, dieptxf1, etc.)
        // This is set initially during Driver::start(), and embassy-usb doesn't allow changing
        // endpoint config after start() we never need to update the FIFO configuration.

        // Re-enable receipt of SETUP and XFER complete endpoint interrupts

        // At the moment we don't care about any OUT endpoint interrupts,
        // so we don't set daintmsk.outepmsk0 or doepmsk.setupmsk and doepmsk.xfercomplmsk
        // Maybe xfercomplmsk would be useful purely for debug logging
        // TODO: clean this up after I'm sure I don't need it anymore
        /*
        self.usb0
            .daintmsk()
            .modify(|_, w| w.outepmsk0().set_bit().inepmsk0().set_bit());
        self.usb0
            .doepmsk()
            .modify(|_, w| w.setupmsk().set_bit().xfercomplmsk().set_bit());
            */
        self.usb0.daintmsk().modify(|_, w| w.inepmsk0().set_bit());
        self.usb0
            .diepmsk()
            .modify(|_, w| w.timeoutmsk().set_bit().di_xfercomplmsk().set_bit());
        self.usb0
            .out_ep0()
            .doeptsiz()
            .modify(|_, w| unsafe { w.supcnt0().bits(3) });
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
                w.d_mps0()
                    .bits(self.ep0_mps_bits)
                    .d_txfnum0()
                    .bits(EP0_TX_FIFO_NUM)
            }
            .d_stall0()
            .clear_bit()
        });
    }

    fn process_rx_fifo(&mut self) {
        trace!("RX FIFO data ready");
        // Process entries from the FIFO until the FIFO is empty.
        loop {
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
                    // DATA phase of a control transfer.  Process the most recently seen SETUP
                    // packet now, as we know this SETUP packet is valid for this transfer and is
                    // not going to be superceded by another retransmitted SETUP.
                    //
                    // Note: the STMicro docs recommend doing nothing here, and waiting until we
                    // get the stuppktrcvd OUT endpoint interrupt to process the packet.  It's not
                    // clear to me if there is any reason to wait until the interrupt.  Other
                    // driver implementations also appear to process the SETUP packet here with no
                    // repercussions.  Doing the processing here lets us avoid needing to enable
                    // OUT endpoint interrupts.
                    let ep_index = ctrl_word.chnum().bits();
                    if ep_index == 0 {
                        self.ep0_setup_ready = true;
                        self.ep_out_waker[0].wake();
                    }
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
                        self.ep0_setup_data[0] = rx_fifo.read();
                        self.ep0_setup_data[1] = rx_fifo.read();
                        trace!(
                            "SETUP received: {:?}",
                            bytemuck::cast_slice::<u32, u8>(&self.ep0_setup_data)
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
                    // We don't really expect to get this here.
                    // set_global_out_nak() will read and consume it on its own.
                    trace!("global OUT NAK effective");
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

            // If the rxflvi bit is clear, the FIFO is empty and there is nothing left to process.
            if self.usb0.gintsts().read().rxflvi().bit_is_clear() {
                return;
            }
        }
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
            // the next read() call and return it then.
            self.discard_rx_fifo_data(byte_count);
        }

        // TODO: look a bit more closely at the embassy-stm32 behavior, then delete this code.
        /*
        if state.ep_out_size[ep_num].load(Ordering::Acquire) == EP_OUT_BUFFER_EMPTY {
            // SAFETY: Buffer size is allocated to be equal to endpoint's maximum packet size
            // We trust the peripheral to not exceed its configured MPSIZ
            let buf = unsafe {
                core::slice::from_raw_parts_mut(*state.ep_out_buffers[ep_num].get(), len)
            };


            state.ep_out_size[ep_num].store(len as u16, Ordering::Release);
            state.ep_out_wakers[ep_num].wake();
        } else {
            error!("ep_out buffer overflow index={}", ep_num);

            // discard FIFO data
            let len_words = (len + 3) / 4;
            for _ in 0..len_words {
                r.fifo(0).read().data();
            }
        }
        */
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
        // TODO: do testing of both code paths here
        if whole_words > 0 {
            if ((buf.as_ptr() as usize) & 0x3) == 0 {
                // If the buffer is word-aligned, we can read directly into it
                let buf_words: &mut [u32] = bytemuck::cast_slice_mut(&mut buf[0..whole_words * 4]);
                for word in buf_words {
                    *word = rx_fifo.read();
                }
            } else {
                for chunk in buf.chunks_exact_mut(4) {
                    let word: u32 = rx_fifo.read();
                    chunk.copy_from_slice(bytemuck::bytes_of(&word));
                }
            }
        }
        let remainder = buf.len() - whole_words * 4;
        if remainder > 0 {
            let word: u32 = rx_fifo.read();
            buf[whole_words * 4..].copy_from_slice(&bytemuck::bytes_of(&word)[0..remainder]);
        }
    }

    fn write_to_fifo(&mut self, buf: &[u8], fifo_index: usize) {
        // TODO: it would be nicer to use DMA for reads and writes, but I haven't seen much
        // documentation on how to do DMA transfers to the USB core.   I've seen posts in the
        // Espressif forum that they plan to implement DMA transfers in the tinyusb code at some
        // point, so we could look at their implementation once that is complete.
        let fifo = self.usb0.fifo(fifo_index);
        let whole_words = buf.len() / 4;
        // TODO: do testing of both code paths here
        if whole_words > 0 {
            // We could check buf.as_ptr().is_aligned_to(4), but this is nightly-only
            if ((buf.as_ptr() as usize) & 0x3) == 0 {
                // For aligned buffers, we can just use bytemuck::cast() to read the values as u32
                let buf_words: &[u32] = bytemuck::cast_slice(&buf[0..whole_words * 4]);
                for word in buf_words {
                    fifo.write(*word);
                }
            } else {
                for chunk in buf.chunks_exact(4) {
                    let mut word = [0u8; 4];
                    word[0..4].copy_from_slice(chunk);
                    fifo.write(bytemuck::cast(word));
                }
            }
        }
        let remainder = buf.len() - whole_words * 4;
        if remainder > 0 {
            // Copy the final partial word
            let mut word = [0u8; 4];
            word[0..remainder].copy_from_slice(&buf[whole_words * 4..]);
            fifo.write(bytemuck::cast(word));
        }
    }

    /*
    fn process_out_ep_interrupts(&mut self) {
        let daint = (self.usb0.daint().read().bits() >> 16);

        for ep_index in 0..NUM_OUT_ENDPOINTS {
            if daint & (1 << ep_index) {
                self.service_out_endpoint(ep_index);
            }
        }
    }

    fn service_out_endpoint(&mut self, ep_index: usize) {
        let doepint = self.usb0.out_ep(ep_index).doepint();
        let ints = doepint.read();
        trace!("interrupt on OUT EP{}: {:#x}", ep_index, ints.bits());

        if ints.stuppktrcvd1().bit() {
            trace!("process setup received");
            // We only configure EP0 as a control endpoint, so we don't expect to get SETUP packets
            // on any other endpoint, unless maybe the host is buggy?
            if ep_index == 0 {
                self.ep0_setup_ready = true;
                self.ep_out_waker[0].wake();
            }
        }

        // TODO: actually process the interrupts
        doepint.write(|w| {
            w.setup1()
                .set_bit()
                .stuppktrcvd0()
                .set_bit()
                .xfercompl0()
                .set_bit()
        });
        // TODO
    }
    */

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
        let diepint = self.usb0.in_ep(ep_index).diepint();
        let ints = diepint.read();
        trace!("interrupt on IN EP{}: {:#x}", ep_index, ints.bits());

        // Clear all interrupts that we process
        // Note that txfemp is read-only, and will always be asserted when the FIFO is empty.
        diepint.write(|w| w.d_xfercompl1().set_bit().d_timeout1().set_bit());

        // If the TX FIFO is empty, disable this interrupt for this endpoint and then wake
        // any endpoint handling code that may be pending.  (In particular, poll_in_ep_transmit()
        // cares about this if it is pending.)
        if ints.d_txfemp1().bit_is_set() {
            self.usb0.diepempmsk().modify(|r, w| {
                let new_bits = r.d_ineptxfempmsk().bits() & !(1 << ep_index);
                unsafe { w.d_ineptxfempmsk().bits(new_bits) }
            });
        }

        // The timeout interrupt is only generated for control endpoints,
        // if a timeout occurs attempting to send an IN token.
        if ints.d_timeout1().bit_is_set() {
            warn!("IN transaction timeout on EP{}", ep_index);

            // TODO: we probably should flush the TX FIFO after a timeout.
            // We would need to:
            // - Set the NAK flag for this endpoint
            // - Wait for the NAK flag to take effect
            // - Wait for grstctl.ahbidle
            // - Modify grstctl to set txfnum to ep_index and set the txfflsh bit
            // - Wait for the txflsh bit to clear.
            // - Clear the NAK flag for the endpoint
        }
        if ints.d_xfercompl1().bit_is_set() {
            trace!("TX complete on EP{}", ep_index);
        }

        // Wake the ep_in_waker on any endpoint IN interrupt.
        // - poll_in_ep_transmit() cares about waking on d_txfemp (more room available in the TX
        //   FIFO) and d_xfercompl (a transfer has finished and a new one can now be started).
        self.ep_in_waker[ep_index].wake();
    }

    fn disable_all_endpoints(&mut self) {
        // TODO
        /*
        for i in 0..T::ENDPOINT_COUNT {
            self.endpoint_set_enabled(EndpointAddress::from_parts(i, Direction::In), false);
            self.endpoint_set_enabled(EndpointAddress::from_parts(i, Direction::Out), false);
        }
        */
    }

    fn nak_all_out_endpoints(&mut self) {
        self.usb0
            .out_ep0()
            .doepctl()
            .modify(|_, w| w.do_snak0().set_bit());
        for n in 1..7 {
            self.usb0
                .out_ep(n)
                .doepctl()
                .modify(|_, w| w.do_snak1().set_bit());
        }
    }

    fn flush_fifos_on_reset(&mut self) {
        self.stop_all_in_endpoints();
        self.stop_all_out_endpoints();
    }

    fn stop_all_in_endpoints(&mut self) {
        trace!("stop_all_in_endpoints");
        // Set the disable and NAK bits
        // Note: seems like an SVD typo that EP3 and EP5 use different register names?
        //
        // We do this for all endpoints except EP0, which is always enabled by hardware.
        self.usb0
            .in_ep0()
            .diepctl()
            .modify(|_, w| w.d_epdis0().set_bit().di_snak0().set_bit());
        for n in 1..NUM_IN_ENDPOINTS {
            self.usb0
                .in_ep(n)
                .diepctl()
                .modify(|_, w| w.d_epdis1().set_bit().di_snak1().set_bit());
        }

        // Wait for the disable to take effect
        // (It's unfortunate that we need to busy wait here)
        while self
            .usb0
            .in_ep0()
            .diepint()
            .read()
            .d_epdisbld0()
            .bit_is_clear()
        {}
        for n in 1..NUM_IN_ENDPOINTS {
            while self
                .usb0
                .in_ep(n)
                .diepint()
                .read()
                .d_epdisbld1()
                .bit_is_clear()
            {}
        }

        // Flush all TX FIFOs
        self.usb0
            .grstctl()
            .modify(|_, w| unsafe { w.txfnum().bits(0x10) }.txfflsh().set_bit());
        // Busy loop for the flush to complete.
        while self.usb0.grstctl().read().txfflsh().bit_is_set() {}
    }

    fn stop_all_out_endpoints(&mut self) {
        trace!("stop_all_out_endpoints");
        // This is based on documentation for the synopsys USB cores in STMicro chips.
        // See "Transfer Stop Programming for OUT Endpoints" in section 34.17.5 of the STM32F405
        // reference manual.

        // 1. Enable all OUT endpoints
        // TODO: should we perhaps also set the SNAK bit here, to ensure no new packets
        // get put in the RX FIFO between when we flush it and when we set the global OUT NAK flag?
        self.usb0
            .out_ep0()
            .doepctl()
            .modify(|_, w| w.epena0().set_bit());
        for n in 1..NUM_OUT_ENDPOINTS {
            self.usb0
                .out_ep(n)
                .doepctl()
                .modify(|_, w| w.epena1().set_bit());
        }

        // 2. Flush the RX FIFO:
        // 2a. Wait for grstctl.ahbidle to be set
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

        // 5. disable all OUT endpoints by setting epdis and snak in doepctlN
        //
        // Hmm, the esp32s3 SVD lists the EPDIS bits as readable but not writable.
        // The ESP32 tinyusb code does set them though.  Let's do the same.
        // This unfortunately means we need to manually set these bits.
        //
        // Note that we skip EP0, since it is always enabled by hardware.  (SETUP packets cannot be
        // NAKed).
        const EPDIS: u32 = 1 << 30;
        const SNAK: u32 = 1 << 27;
        for n in 1..NUM_OUT_ENDPOINTS {
            self.usb0
                .out_ep(n)
                .doepctl()
                .modify(|r, w| unsafe { w.bits(r.bits() | EPDIS | SNAK) });
        }

        // 6. wait for the epdis interrupt in doepintN for all endpoints
        for n in 1..NUM_OUT_ENDPOINTS {
            while self
                .usb0
                .out_ep(n)
                .doepint()
                .read()
                .epdisbld1()
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

        // Read from the RX FIFO until we see the GLOBAL_OUT_NAK effective control word
        // TODO: create a helper function to process one RX FIFO entry, and then share that with
        // the normal process_rx_fifo() code?
        loop {
            while self.usb0.gintsts().read().rxflvi().bit_is_clear() {}
            let ctrl_word = self.usb0.grxstsp().read();
            match ctrl_word.pktsts().bits() {
                pktsts::GLOBAL_OUT_NAK => {
                    break;
                }
                pktsts::SETUP_RECEIVED => {
                    // We may receive a SETUP packet if there was already one in the RX FIFO
                    // that we hadn't read.
                    trace!("received SETUP packet while waiting on global OUT NAK effective");
                    // Read and discard the setup packet.
                    // TODO: perhaps we should save it?
                    let rx_fifo = self.usb0.fifo(0);
                    let _ = rx_fifo.read();
                    let _ = rx_fifo.read();
                    break;
                }
                _ => {
                    // TODO: there may have been a valid OUT packet already present in the RX FIFO that
                    // we need to read and discard.
                    warn!(
                        "valid entry in RX FIFO before GLOBAL_OUT_NAK: {:#x}",
                        ctrl_word.pktsts().bits()
                    );
                }
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
}

pub(crate) async fn start_read_op<'b>(
    state: &'b RefCell<State>,
    ep_index: usize,
    buf: &'b mut [u8],
) -> Result<ReadOperation<'b>, EndpointError> {
    futures::future::poll_fn(|cx| {
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
pub(crate) struct ReadOperation<'b> {
    state: &'b RefCell<State>,
    buf: &'b mut [u8],
    ep_index: usize,
}

impl<'b> ReadOperation<'b> {
    pub(crate) async fn do_read(&mut self) -> Result<usize, EndpointError> {
        futures::future::poll_fn(|cx| {
            let mut state = self.state.borrow_mut();
            state.poll_out_ep_read_complete(cx, self.ep_index)
        })
        .await
    }
}

impl<'b> Drop for ReadOperation<'b> {
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
        self.status = ReadStatus::Complete;
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
