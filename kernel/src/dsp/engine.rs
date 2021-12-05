//! Signal processing core logic.

use core::cell::Cell;

use crate::config;
use crate::dsp::buffer::AudioBuffer;
use crate::dsp::component::{Collector, Processor, Player};
use crate::hil::dma::{self, DMA, DMAChannel};
use crate::platform::KernelResources;
use crate::platform::chip::Chip;

pub struct DSPEngine {
    /// Buffers for audio samples.
    sample_buffers: [AudioBuffer; config::SAMPLE_BUFFERS],
    /// Index of the currently collecting buffer.
    collecting_idx: Cell<usize>,
    /// Index of the currently-processing buffer.
    in_processing_idx: Cell<usize>,
    /// Index of the currently playing buffer.
    playback_idx: Cell<usize>,
}

impl DSPEngine {
    /// Create a new `DSPEngine` instance.
    pub unsafe fn new() -> DSPEngine {
        DSPEngine {
            sample_buffers: [AudioBuffer::new(),
                             AudioBuffer::new(),
                             AudioBuffer::new()],
            collecting_idx: Cell::new(0),
            in_processing_idx: Cell::new(0),
            playback_idx: Cell::new(0),
        }
    }

    /// Run the digital signal processing loop.
    ///
    /// Performs initial configuration and starts the DSP loop.
    /// Once configured, the DSP side of the kernel will respond to a short list of interrupts:
    /// SIO, for interprocessor messaging;
    /// DMA, for sample processing and output.
    pub fn run<C: Chip, R: KernelResources<C>>(
        &'static self,
        chip: &C,
        resources: &R,
        dma: &'static dyn DMA,
    ) -> !
    {
        // Configure DMA channels.
        let adc_dma_channel = allocate_adc_dma(dma);
        let adc_dma_channel_no = adc_dma_channel.channel_no();

        // Create buffer handler components.
        let (mut _collector, mut _processor, mut _player) =
            (Collector::new(self.sample_buffers.iter().cycle(), adc_dma_channel),
             Processor::new(self.sample_buffers.iter().cycle()),
             Player::new(self.sample_buffers.iter().cycle()));

        loop {
        }
    }
}

fn allocate_adc_dma(dma: &'static dyn DMA) -> &'static dyn DMAChannel {
    let params = dma::Parameters {
            kind: dma::TransferKind::PeripheralToMemory(dma::SourcePeripheral::ADC, 0),
            transfer_count: config::NO_SAMPLES,
            transfer_size: dma::TransferSize::Word, // need to optimize
            increment_on_read: false,
            increment_on_write: true,
            high_priority: true,
    };

    dma.configure(&params).unwrap()
}
