//! Signal processing core logic.

use core::cell::Cell;

use crate::config;
use crate::dsp::buffer::{AudioBuffer, BufferState};
use crate::dsp::component::{Collector, Processor, Player};
use crate::hil::dma::DMA;
use crate::static_buf;
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
    /// DMA, for sample processing.
    pub fn run<C: Chip, R: KernelResources<C>>(
        &self,
        chip: &C,
        resources: &R,
        dma: &'static dyn DMA,
    ) -> !
    {
        // Configure DMA channels.

        // Start the loop process by passing the first buffer to the DMA collecting samples from the ADC.

        loop {
        }
    }
}
