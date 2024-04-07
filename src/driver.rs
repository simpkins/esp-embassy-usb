use crate::bus::Bus;
use crate::control_pipe::ControlPipe;
use crate::endpoint::{EndpointIn, EndpointOut};
use crate::state::{InEndpointConfig, OutEndpointConfig, State};
use core::cell::RefCell;
use embassy_usb_driver::{EndpointAllocError, EndpointType};
use log::{error, trace};

pub struct Driver<'d> {
    state: &'d RefCell<State<'d>>,
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
        // DIEPTXF requires a minimum size of 16 words
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
    /// Driver constructor.
    /// Users shouldn't use this directly: use State::new() to create a State object instead,
    /// then call State::driver().
    pub(crate) fn new(state: &'d RefCell<State<'d>>) -> Self {
        Self {
            state,
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

        let mut state = self.state.borrow_mut();
        let slot_index = Self::find_free_endpoint_slot(&mut state.ep_in_config, ep_type)?;
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

        // Now that we know allocation is successful, update the ep_in_config array
        state.ep_in_config[slot_index] = Some(InEndpointConfig {
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

        let mut state = self.state.borrow_mut();
        let slot_index = Self::find_free_endpoint_slot(&mut state.ep_out_config, ep_type)?;
        if ep_type == EndpointType::Control && slot_index != 0 {
            // The embassy-usb API assumes there is only ever a single control pipe, and does not
            // expect SETUP packets on other endpoints.  The hardware and the USB spec both
            // technically support having multiple control message pipes, but this generally isn't
            // used or useful in practice.
            error!("only a single control endpoint is supported");
            return Err(EndpointAllocError);
        }

        state.ep_out_config[slot_index] = Some(OutEndpointConfig {
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

        let ep0_out = self
            .alloc_endpoint_out(EndpointType::Control, control_max_packet_size, 0)
            .unwrap();
        let ep0_in = self
            .alloc_endpoint_in(EndpointType::Control, control_max_packet_size, 0)
            .unwrap();
        assert_eq!(ep0_out.index(), 0);
        assert_eq!(ep0_in.index(), 0);

        {
            let mut state = self.state.borrow_mut();
            state
                .set_ep0_max_packet_size(control_max_packet_size)
                .expect("invalid max_packet_size for control endpoint");

            // Redistribute any remaining free FIFO space
            self.fifo_settings
                .redistribute_free_space(&state.ep_in_config);

            trace!("start");
            state.init_bus(&self.fifo_settings);
        }

        (
            Bus::new(self.state),
            ControlPipe::new(ep0_in, ep0_out),
        )
    }
}
