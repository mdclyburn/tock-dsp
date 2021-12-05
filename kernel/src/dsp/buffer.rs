//! Audio sample buffer management.

use crate::config;
use crate::static_buf;
use crate::utilities::cells::TakeCell;

/// Current status of samples in an [`AudioBuffer`].
#[derive(Copy, Clone)]
pub enum BufferState {
    /// Buffer is ready to hold new samples.
    Free,
    /// Buffer is currently collecting samples.
    Collecting,
    /// Buffer holds unprocessed samples.
    Unprocessed,
    /// Filters are working with this buffer.
    Processing,
    /// Buffer contains processed, unplayed samples.
    Ready,
    /// Buffer samples are being output.
    Playing,
}

/// Container for audio samples.
pub struct AudioBuffer {
    samples: TakeCell<'static, [usize]>,
    current_state: BufferState,
}

impl AudioBuffer {
    /// Create an [`AudioBuffer`].
    pub unsafe fn new() -> AudioBuffer {
        AudioBuffer {
            samples: TakeCell::new(static_buf!([usize; config::NO_SAMPLES]).initialize([0; config::NO_SAMPLES])),
            current_state: BufferState::Free,
        }
    }

    /// Remove the sample buffer for use.
    pub fn take(&self) -> Option<&'static mut [usize]> {
        self.samples.take()
    }

    /// Replace the sample buffer.
    pub fn put(&self, buffer: &'static mut [usize]) {
        self.samples.put(Some(buffer))
    }

    /// Current state of samples in the buffer.
    pub fn state(&self) -> BufferState {
        self.current_state
    }
}
