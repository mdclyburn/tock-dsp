use core::cell::Cell;

use cortexm0p::support;

use rp2040::dma;

use kernel::static_init;
use kernel::{ErrorCode, Kernel};
use kernel::utilities::cells::TakeCell;

use crate::{RaspberryPiPico, RP2040Chip};

const SAMPLING_RATE: usize = 44_100 * 4;
const BUFFER_LEN_MS: usize = 20;
const NO_SAMPLES: usize = SAMPLING_RATE * BUFFER_LEN_MS / 1000;

const SAMPLE_BUFFERS: usize = 2;

struct ASPK {
    samples: [TakeCell<'static, [usize]>; SAMPLE_BUFFERS],
    dma_next: Cell<usize>,
}

impl ASPK {
    unsafe fn run(&self,
                  tock_kernel: &'static Kernel,
                  board_resources: &'static RaspberryPiPico,
                  chip_resources: &'static RP2040Chip,
    ) -> !
    {
        // Start up the ADC.
        use kernel::hil::adc::Adc;
        let _ = board_resources.adc.sample_continuous(
            &rp2040::adc::Channel::Channel0,
            SAMPLING_RATE as u32)
            .unwrap();

        loop {
            support::wfe();
        }
    }

    fn start_sampling(&self, board_resources: &'static RaspberryPiPico) -> Result<usize, ErrorCode> {
        // Pick the next buffer.
        let buffer_idx = self.dma_next.get();
        if self.samples[buffer_idx].is_none() {
            // Have to get processed samples out quicker...
            Err(ErrorCode::BUSY)
        } else {
            let buffer = self.samples[buffer_idx].take()
                // We just checked this manually.
                .unwrap();
            self.dma_next.set(self.dma_next.get() + 1 % SAMPLE_BUFFERS);
            // Set up DMA to run off of the ADC samples.
            board_resources.dma.configure(
                None,
                buffer,
                &dma::ChannelOptions {
                // ADC result register, offset by +2 because we use half-word transfers
                // to move 12-bit data from the ADC.
                read_address: 0x4004_C004 + 2,
                write_address: core::ptr::addr_of!(buffer) as u32,
                transfer_count: buffer.len() as u32,
                transfer_size: dma::TransferSize::HalfWord,
                treq_signal: dma::TransferRequestSignal::ADC,
                ring_config: dma::TransferAddressWrap::None,
                increment_on_read: false,
                increment_on_write: true,
                high_priority: true,
                irq_line: Some(dma::InterruptLine::IRQ0),
            })
        }
    }
}

pub unsafe fn start(tock_kernel: &'static Kernel,
                    board_resources: &'static RaspberryPiPico,
                    chip_resources: &'static RP2040Chip,
) -> !
{
    let kernel = static_init!(ASPK, ASPK {
        samples: [TakeCell::new(kernel::static_buf!([usize; NO_SAMPLES]).initialize([0; NO_SAMPLES])),
                  TakeCell::new(kernel::static_buf!([usize; NO_SAMPLES]).initialize([0; NO_SAMPLES]))],
        dma_next: Cell::new(0),
    });

    kernel.run(tock_kernel, board_resources, chip_resources);
}
