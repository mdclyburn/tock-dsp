use core::cell::Cell;

use cortexm0p::support;

use rp2040::dma::DMA;

use kernel::static_init;
use kernel::Kernel;
use kernel::utilities::cells::TakeCell;

use crate::{RaspberryPiPico, RP2040Chip};

const SAMPLING_RATE: usize = 44_100 * 4;
const BUFFER_LEN_MS: usize = 20;
const NO_SAMPLES: usize = SAMPLING_RATE * BUFFER_LEN_MS / 1000;

const SAMPLE_BUFFERS: usize = 2;

struct ASPK {
    samples: [TakeCell<'static, [u16]>; SAMPLE_BUFFERS],
    dma_next: Cell<usize>,
}

impl ASPK {
    unsafe fn run(&self,
                  tock_kernel: &'static Kernel,
                  board_resources: &'static RaspberryPiPico,
                  chip_resources: &'static RP2040Chip,
    ) -> !
    {
        // Set up DMA to run off of the ADC samples.


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
}

pub unsafe fn start(tock_kernel: &'static Kernel,
                    board_resources: &'static RaspberryPiPico,
                    chip_resources: &'static RP2040Chip,
) -> !
{
    let kernel = static_init!(ASPK, ASPK {
        samples: [TakeCell::new(kernel::static_buf!([u16; NO_SAMPLES]).initialize([0; NO_SAMPLES])),
                  TakeCell::new(kernel::static_buf!([u16; NO_SAMPLES]).initialize([0; NO_SAMPLES]))],
        dma_next: Cell::new(0),
    });

    kernel.run(tock_kernel, board_resources, chip_resources);
}
