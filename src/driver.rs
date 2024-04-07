use crate::bus::Bus;
use crate::control_pipe::ControlPipe;
use crate::endpoint::{EndpointIn, EndpointOut};
use crate::fmt::{error, trace};
use crate::state::{InEndpointConfig, OutEndpointConfig, State};
use core::cell::RefCell;
use embassy_usb_driver::{EndpointAllocError, EndpointType};

/// The Driver object is mainly responsible for endpoint allocation.
///
/// This object only exists briefly while the USB configuration is being defined.  Once all
/// endpoints have been allocated, Driver::start() is called, which creates the Bus and ControlPipe
/// and destroys the Driver object itself.
pub struct Driver<'d> {
    state: &'d RefCell<State<'d>>,
    fifo_settings: FifoSettings,
}

// The ESP32-S3 and ESP32-S2 technical reference manuals both state the FIFO depth is 256.
// This matches the EP_FIFO_SIZE constant in the tinyusb code for ESP32, despite the code having
// comments that document the FIFO layout as either 320 or 1024 entries.
const FIFO_DEPTH_WORDS: u16 = 256;

/// A structure for keeping track of allocated FIFO sizes while we
/// are still allocating endpoints.
#[derive(Debug)]
struct FifoSettings {
    largest_rx_packet: u16,
    num_out_endpoints: u8,
    tx_num_words: [u16; 5],
}

/// The finalized FIFO sizes after all endpoints have been allocated.
#[derive(Debug)]
pub(crate) struct FifoSizes {
    pub(crate) rx_num_words: u16,
    pub(crate) tx_num_words: [u16; 5],
}

impl FifoSettings {
    fn new(ep0_max_packet_size: u16) -> Self {
        let mut fs = Self {
            largest_rx_packet: ep0_max_packet_size,
            num_out_endpoints: 1, // Start out assuming at least EP0 exists
            tx_num_words: [0; 5],
        };
        fs.tx_num_words[0] = Self::minimum_tx_words(ep0_max_packet_size);
        fs
    }

    fn alloc_out_endpoint(&mut self, max_packet_size: u16) -> Result<(), EndpointAllocError> {
        self.num_out_endpoints += 1;
        let orig_largest_rx_packet = self.largest_rx_packet;
        self.largest_rx_packet = u16::max(self.largest_rx_packet, max_packet_size);

        if self.allocated_num_words() > FIFO_DEPTH_WORDS {
            self.num_out_endpoints -= 1;
            self.largest_rx_packet = orig_largest_rx_packet;
            Err(EndpointAllocError)
        } else {
            Ok(())
        }
    }

    fn alloc_in_endpoint(&mut self, max_packet_size: u16) -> Result<u8, EndpointAllocError> {
        let fifo_index = match self.find_free_tx_fifo() {
            Some(idx) => idx,
            None => return Err(EndpointAllocError),
        };

        // DIEPTXF requires a minimum size of 16 words
        let fifo_size = Self::minimum_tx_words(max_packet_size);
        self.tx_num_words[fifo_index] = fifo_size;
        if self.allocated_num_words() > FIFO_DEPTH_WORDS {
            self.tx_num_words[fifo_index] = 0;
            return Err(EndpointAllocError);
        }
        Ok(fifo_index as u8)
    }

    fn find_free_tx_fifo(&self) -> Option<usize> {
        for (fifo_index, fifo_num_words) in self.tx_num_words.iter().enumerate() {
            if *fifo_num_words == 0 {
                return Some(fifo_index);
            }
        }
        // All TX FIFOs are already in use
        None
    }

    #[inline(always)]
    fn minimum_tx_words(max_packet_size: u16) -> u16 {
        // DIEPTXF requires a minimum size of 16 words
        u16::max((max_packet_size + 3) / 4, 16)
    }

    fn get_min_rx_fifo_words(&self) -> u16 {
        // According to the documentation for STMicro manuals (such as for the STM32F405),
        // the size must be large enough to hold:
        //
        // - 10 words for SETUP packets
        //   - Up to 3 back-to-back setup packets, each of which requires 3 words
        //     - 2 words for the SETUP packet data, plus 1 word for pktsts::SETUP_RECEIVED
        //   - 1 word for pktsts::SETUP_COMPLETE
        // 1 for pktsts::GLOBAL_OUT_NAK
        // 1 for pktsts::OUT_XFER_COMPLETE for each OUT endpoint
        // 1 for pktsts::OUT_PKT_RECEIVED for 1 packet
        // Plus room for the largest possible OUT packet
        //
        // Ideally the RX FIFO should be sized large enough to hold at least 2 largest-size-packets
        // rather than just 1, so that there is room for the USB peripheral to start receiving a
        // subsequent packet while the CPU is processing the first packet.  In this function we
        // just return the minimum required size, and we will expand it later if possible in
        // finalize_fifo_sizes().
        10 + 1 + (self.num_out_endpoints as u16) + 1 + self.largest_rx_packet
    }

    fn allocated_num_words(&self) -> u16 {
        return self.get_min_rx_fifo_words() + self.tx_num_words.iter().sum::<u16>();
    }

    fn finalize_fifo_sizes(&self) -> FifoSizes {
        // This method is called after all endpoints have been defined.
        // This should update the FIFO sizes to make the best use of any remaining unallocated
        // space.
        //
        // We currently give all remaining space to the RX FIFO.
        //
        // The embassy-usb APIs currently unfortunately don't let us send more than a single TX
        // packet at a time to the device, so there is no point giving extra space to any TX FIFOs.
        // The embassy-usb APIs require that the high-level code split large transfers up into
        // multiple separate writes, each with at most max-packet-size data.  They don't currently
        // allow giving larger writes to the driver.  (Other driver implementations do not appear
        // to support larger sized writes, even when the underlying hardware supports doing
        // packetization itself.)
        //
        // The embassy-usb RX code also breaks down reads into separate max-packet-sized chunks,
        // which seems unfortunate since this means the HW has to NAK subsequent OUT packets on any
        // given endpoint until we have consumed the previous packet from the FIFO, so it can only
        // have at most 1 packet per endpoint in the RX FIFO.
        let tx_total_words = self.tx_num_words.iter().sum::<u16>();
        let rx_num_words = FIFO_DEPTH_WORDS - tx_total_words;
        FifoSizes {
            rx_num_words,
            tx_num_words: self.tx_num_words,
        }
    }
}

impl<'d> Driver<'d> {
    /// Driver constructor.
    /// Users shouldn't use this directly: use State::new() to create a State object instead,
    /// then call State::driver().
    pub(crate) fn new(state: &'d RefCell<State<'d>>) -> Self {
        // The endpoint 0 max packet size for using in our FIFO size calculations.
        // The EP0 max packet size can't ever be larger than 64 bytes.
        //
        // Since Driver::start() cannot return an error, we must make sure that the FIFO space
        // needed for EP0 is already taken into account when alloc_endpoint_in() and
        // alloc_endpoint_out() run, since those functions are our only opportunity to return an
        // error if we run out of FIFO space.
        let ep0_max_packet_size = 64;

        Self {
            state,
            fifo_settings: FifoSettings::new(ep0_max_packet_size),
        }
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

        if ep_type == EndpointType::Control {
            // The embassy-usb API assumes there is only ever a single control pipe, and does not
            // expect SETUP packets on other endpoints.  The hardware and the USB spec both
            // technically support having multiple control message pipes, but this generally isn't
            // used or useful in practice.
            error!("embassy-usb does not support control endpoints other than EP0");
            return Err(EndpointAllocError);
        }

        // Make sure we have enough FIFO space.
        let tx_fifo = self.fifo_settings.alloc_in_endpoint(max_packet_size)?;

        // Allocate the endpoint index
        let ep_index = self
            .state
            .borrow_mut()
            .alloc_in_endpoint(InEndpointConfig {
                ep_type,
                tx_fifo,
                max_packet_size,
            })?;

        Ok(EndpointIn::new(
            self.state,
            ep_index,
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

        if ep_type == EndpointType::Control {
            // The embassy-usb API assumes there is only ever a single control pipe, and does not
            // expect SETUP packets on other endpoints.  The hardware and the USB spec both
            // technically support having multiple control message pipes, but this generally isn't
            // used or useful in practice.
            error!("embassy-usb does not support control endpoints other than EP0");
            return Err(EndpointAllocError);
        }

        // Make sure we have enough FIFO space.
        self.fifo_settings.alloc_out_endpoint(max_packet_size)?;

        // Allocate the endpoint index
        let ep_index = self
            .state
            .borrow_mut()
            .alloc_out_endpoint(OutEndpointConfig {
                ep_type,
                max_packet_size,
            })?;

        Ok(EndpointOut::new(
            self.state,
            ep_index,
            ep_type,
            max_packet_size,
            interval_ms,
        ))
    }

    fn start(self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        // Create the EP0 endpoints.  We have already allocated slots for this endpoint
        // so we do not call alloc_endpoint_in() and alloc_endpoint_out() here.
        assert!(control_max_packet_size <= 64);
        let ep0_out = EndpointOut::new(
            self.state,
            0,
            EndpointType::Control,
            control_max_packet_size,
            0,
        );
        let ep0_in = EndpointIn::new(
            self.state,
            0,
            EndpointType::Control,
            control_max_packet_size,
            0,
        );

        {
            let mut state = self.state.borrow_mut();
            state
                .set_ep0_max_packet_size(control_max_packet_size)
                .expect("invalid max_packet_size for control endpoint");

            // Redistribute any remaining free FIFO space
            let fifo_sizes = self.fifo_settings.finalize_fifo_sizes();

            trace!("start");
            state.init_bus(&fifo_sizes);
        }

        (Bus::new(self.state), ControlPipe::new(ep0_in, ep0_out))
    }
}
