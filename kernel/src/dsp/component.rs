//! Sample data workers.

use core::iter::Cycle;
use core::slice::Iter as SliceIter;

use crate::dsp::buffer::AudioBuffer;
use crate::hil::dma::DMAChannel;

pub(super) type AudioBufferIter = Cycle<SliceIter<'static, AudioBuffer>>;

/// Audio sample collector.
///
/// Drives audio sample collection.
/// `Collector` makes use of a DMA channel to collect audio samples for processing.
pub struct Collector {
    buffer_it: AudioBufferIter,
    adc_dma_channel: &'static dyn DMAChannel,
    current_buffer: Option<&'static mut [usize]>,
}

impl Collector {
    /// Create a new `Collector`.
    pub(super) fn new(buffer_it: AudioBufferIter,
                      adc_dma_channel: &'static dyn DMAChannel) -> Collector {
        Collector {
            buffer_it,
            adc_dma_channel,
            current_buffer: None,
        }
    }
}

/// Audio sample processor.
pub struct Processor {
    buffer_it: AudioBufferIter,
}

impl Processor {
    /// Create a new `Processor`.
    pub(super) fn new(buffer_it: AudioBufferIter) -> Processor {
        Processor {
            buffer_it,
        }
    }
}

/// Audio sample player.
pub struct Player {
    buffer_it: AudioBufferIter,
}

impl Player {
    /// Create a new `Player`.
    pub(super) fn new(buffer_it: AudioBufferIter) -> Player {
        Player {
            buffer_it,
        }
    }
}
