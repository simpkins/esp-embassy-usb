use crate::bus::Bus;
use crate::control_pipe::ControlPipe;
use crate::endpoint::{EndpointIn, EndpointOut};
use crate::endpoint::{InEndpointConfig, OutEndpointConfig};
use crate::phy::init_phy;
use crate::regs::{NUM_IN_ENDPOINTS, NUM_OUT_ENDPOINTS};
use crate::state::State;
use crate::{Config, PhyType};
use core::cell::RefCell;
use embassy_usb_driver::{EndpointAllocError, EndpointType};
use esp_hal::gpio::{DriveStrength, OutputPin};
use esp_hal::peripheral::Peripheral;
use esp_hal::peripherals;
use log::{error, trace};

pub struct Driver<'d> {
    state: &'d RefCell<State>,
    ep_in: [Option<InEndpointConfig>; NUM_IN_ENDPOINTS],
    ep_out: [Option<OutEndpointConfig>; NUM_OUT_ENDPOINTS],
    fifo_settings: FifoSettings,
}

// The ESP32-S3 and ESP32-S2 technical reference manuals both state the FIFO depth is 256.
// This matches the EP_FIFO_SIZE constant in the tinyusb code for ESP32, despite the code having
// comments that document the FIFO layout as either 320 or 1024 entries.
const FIFO_DEPTH_WORDS: u16 = 256;

// The number of words to allocate for the RX FIFO.
// The embassy-stm32 and synopsys-usb-otg crates use 30 words (120 bytes), and claim they came to
// this number empirically.  The tinyusb code uses 52 words (208 bytes).
//
// Based on the documentation in other STM manuals (e.g., for the STM32F405), the size must be
// large enough to hold:
// - 10 words for setup packets
//   - Each SETUP packet takes 3 words, 2 for the packet data and 1 for the pktsts::SETUP_RECEIVED
//     control word, and we need room to store up to 3 back-to-back SETUP packets
//   - Plus 1 word to store the pktsts::SETUP_COMPLETE control word
// - 2 words for status of the control OUT data packet
// - max packet size of the control endpoint
//   (16 words for full speed, 2 for low speed.  High speed would need 128, but ESP32 does not
//   support high speed).
// - possibly 1 word for global OUT NAK ready status, if we ever set the global OUT NAK flag.
//
// Comments in the tinyusb code recommend including space for 2 control packets rather than just 1.
//
// TODO: what happens for full speed isochronous transfers with max packet size larger than 64
// bytes? The STMicro docs just mention that GRXFSIZ must include space for the control endpoint
// max packet size, but I'm guessing that we presumably need space for max packet size of any
// OUT endpoint?
const RX_FIFO_SIZE_WORDS: u16 = 48;

#[derive(Debug)]
pub(crate) struct FifoSettings {
    pub(crate) rx_num_words: u16,
    pub(crate) tx_num_words: [u16; 5],
}

impl FifoSettings {
    fn new() -> Self {
        Self {
            rx_num_words: RX_FIFO_SIZE_WORDS,
            tx_num_words: [0; 5],
        }
    }

    fn allocated_num_words(&self) -> u16 {
        return self.rx_num_words + self.tx_num_words.iter().sum::<u16>();
    }

    /// Validate and record a TX FIFO assignment.
    ///
    /// This checks that the TX FIFO index is valid and that there is enough FIFO space for at
    /// least one maximum-sized-packet.
    fn allocate_tx_fifo(
        &mut self,
        fifo_index: usize,
        max_packet_size: u16,
    ) -> Result<(), EndpointAllocError> {
        // INEPTXFD requires minimum size of 16 words
        let minimum_fifo_size_words = u16::max((max_packet_size + 3) / 4, 16);
        if self.allocated_num_words() + minimum_fifo_size_words > FIFO_DEPTH_WORDS {
            // We do not have enough space in the FIFO
            error!("not enough FIFO space for new TX endpoint");
            return Err(EndpointAllocError);
        }

        // Check that the FIFO index is valid.
        // The caller picks the FIFO index from the endpoint number, but the chip supports more IN
        // endpoints than TX FIFOs.  The hardware allows users to dynamically change FIFO
        // assignments on the fly, say in response to a SET_CONFIGURATION or SET_INTERFACE call.
        // However, the embassy-usb APIs don't really seem intended to support this, and so all
        // FIFO assignments are pre-determined before Driver::start() is called.  Therefore we
        // can't actually ever use of all IN endpoints provided by the hardware.
        if fifo_index >= self.tx_num_words.len() {
            error!("too many IN endpoints: not enough TX FIFOs");
            return Err(EndpointAllocError);
        }

        // Confirm that the TX FIFO is currently unused.
        // Since we assign each TX FIFO to the same numbered endpoint, this should have already
        // prevented any conflicts.
        if self.tx_num_words[fifo_index] != 0 {
            error!("bug in IN endpoint allocation: TX FIFO already in use");
            return Err(EndpointAllocError);
        }

        // Assign the FIFO.
        self.tx_num_words[fifo_index] = minimum_fifo_size_words;
        Ok(())
    }

    fn redistribute_free_space(&mut self, _endpoints: &[Option<InEndpointConfig>]) {
        // This method is called after all endpoints have been defined.
        // This should update the FIFO sizes to make the best use of any remaining unallocated
        // space.
        //
        // We currently give all remaining space to the RX FIFO.
        //
        // It looks like the embassy-usb APIs unfortunately don't let us send more than a single TX
        // packet at a time to the device, so there is no point giving extra space to any TX FIFOs.
        // It appears that the embassy-usb APIs require that the high-level code split large
        // transfers up into multiple separate writes, each with at most max-packet-size data.
        // They don't currently allow giving larger writes to the driver.  (Other driver
        // implementations do not appear to support larger sized writes, even when the underlying
        // hardware supports doing packetization itself.)
        //
        // The embassy-usb RX code also breaks down reads into separate max-packet-sized chunks,
        // which seems unfortunate since this means the HW has to NAK subsequent OUT packets on any
        // given endpoint until we have consumed the previous packet from the FIFO, so it can only
        // have at most 1 packet per endpoint in the RX FIFO.

        let extra_space = FIFO_DEPTH_WORDS - self.allocated_num_words();
        self.rx_num_words += extra_space;
    }
}

impl<'d> Driver<'d> {
    pub fn new<P, M>(
        usb0: impl Peripheral<P = peripherals::USB0> + 'd,
        usb_wrap: impl Peripheral<P = peripherals::USB_WRAP> + 'd,
        rtc: &peripherals::LPWR,
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

        // It's a little unfortunate that we need to use global state.
        // It would be nicer if the Driver::start() could return a state object to the user,
        // so that the user could manage its lifetime rather only having separate Bus and
        // ControlPipe objects that implicitly depend on the same shared state.
        // The interrupt handler obviously requires some global state, but only needs minimal state
        // (just a pointer to the waker).  We could automatically uninstall the interrupt handler
        // when the main state was dropped if we had proper lifetime management of the main state.
        let state = unsafe { crate::state::STATE.write(RefCell::new(State::new(config))) };

        // Perform one-time initialization of the PHY and other registers.
        // We do this here so that we only need a temporary reference to rtc.
        //
        // We really only need access to rtc.usb_conf(), but esp-hal doesn't really expose the type
        // name for this variable without us needing to directly import the specific chip esp-pac
        // crate.
        init_phy(&config, &*usb_wrap, rtc);

        Self {
            state,
            ep_in: [None; NUM_IN_ENDPOINTS],
            ep_out: [None; NUM_OUT_ENDPOINTS],
            fifo_settings: FifoSettings::new(),
        }
    }

    fn find_free_endpoint_slot<Slot>(
        slots: &[Option<Slot>],
        ep_type: EndpointType,
    ) -> Result<usize, EndpointAllocError> {
        for (index, ep) in slots.iter().enumerate() {
            if index == 0 && ep_type != EndpointType::Control {
                // reserved for control pipe
                continue;
            }
            if ep.is_none() {
                trace!("allocating endpoint {}", index);
                return Ok(index);
            }
        }
        error!("No free endpoints available");
        return Err(EndpointAllocError);
    }
}

impl<'d> embassy_usb_driver::Driver<'d> for Driver<'d> {
    type EndpointOut = EndpointOut<'d>;
    type EndpointIn = EndpointIn<'d>;
    type ControlPipe = ControlPipe<'d>;
    type Bus = Bus<'d>;

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        trace!(
            "alloc_endpoint_in(type={:?}, mps={}, interval={}ms)",
            ep_type,
            max_packet_size,
            interval_ms,
        );

        let slot_index = Self::find_free_endpoint_slot(&mut self.ep_in, ep_type)?;
        if ep_type == EndpointType::Control && slot_index != 0 {
            // The embassy-usb API assumes there is only ever a single control pipe, and does not
            // expect SETUP packets on other endpoints.  The hardware and the USB spec both
            // technically support having multiple control message pipes, but this generally isn't
            // used or useful in practice.
            error!("only a single control endpoint is supported");
            return Err(EndpointAllocError);
        }

        // We always assign TX FIFO N to endpoint N.  The code in EndpointIn::write() currently
        // relies on this assumption, and would need to be updated if we ever change this.
        // Note that endpoint 0 must always use TX FIFO 0, but TX FIFOs 1-3 can be mapped to
        // arbitrary other endpoints.
        //
        // allocate_tx_fifo() will verify that this is a valid fifo number, and will fail if not.
        let fifo_index = slot_index;
        self.fifo_settings
            .allocate_tx_fifo(fifo_index, max_packet_size)?;

        // Now that we know allocation is successful, update the self.ep_in array
        self.ep_in[slot_index] = Some(InEndpointConfig {
            ep_type,
            max_packet_size,
        });
        Ok(EndpointIn::new(
            self.state,
            slot_index,
            ep_type,
            max_packet_size,
            interval_ms,
        ))
    }

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        trace!(
            "alloc_endpoint_out(type={:?}, mps={}, interval={}ms)",
            ep_type,
            max_packet_size,
            interval_ms,
        );

        let slot_index = Self::find_free_endpoint_slot(&mut self.ep_out, ep_type)?;
        if ep_type == EndpointType::Control && slot_index != 0 {
            // The embassy-usb API assumes there is only ever a single control pipe, and does not
            // expect SETUP packets on other endpoints.  The hardware and the USB spec both
            // technically support having multiple control message pipes, but this generally isn't
            // used or useful in practice.
            error!("only a single control endpoint is supported");
            return Err(EndpointAllocError);
        }

        self.ep_out[slot_index] = Some(OutEndpointConfig {
            ep_type,
            max_packet_size,
        });

        Ok(EndpointOut::new(
            self.state,
            slot_index,
            ep_type,
            max_packet_size,
            interval_ms,
        ))
    }

    fn start(mut self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        // It's sort of unfortunate that start() cannot return an error, and has to panic
        // on any failure.
        self.state
            .borrow_mut()
            .set_ep0_max_packet_size(control_max_packet_size)
            .expect("invalid max_packet_size for control endpoint");

        let ep_out = self
            .alloc_endpoint_out(EndpointType::Control, control_max_packet_size, 0)
            .unwrap();
        let ep_in = self
            .alloc_endpoint_in(EndpointType::Control, control_max_packet_size, 0)
            .unwrap();
        assert_eq!(ep_out.info.addr.index(), 0);
        assert_eq!(ep_in.info.addr.index(), 0);

        // Redistribute any remaining free FIFO space
        self.fifo_settings.redistribute_free_space(&self.ep_in);

        trace!("start");
        self.state.borrow_mut().init_bus(&self.fifo_settings);

        (
            Bus::new(self.state),
            ControlPipe::new(self.state, control_max_packet_size, ep_in, ep_out),
        )
    }
}
