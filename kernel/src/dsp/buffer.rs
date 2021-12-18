//! Audio sample buffer management.

use crate::config;
use crate::utilities::cells::{TakeCell, VolatileCell};

/// Current status of samples in an [`AudioBuffer`].
#[derive(Copy, Clone, Debug, PartialEq)]
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
    current_state: VolatileCell<BufferState>,
}

impl AudioBuffer {
    /// Create an [`AudioBuffer`].
    pub unsafe fn new() -> AudioBuffer {
        use crate::static_buf;

        AudioBuffer {
            samples: TakeCell::new(static_buf!([usize; config::NO_SAMPLES]).initialize([0; config::NO_SAMPLES])),
            current_state: VolatileCell::new(BufferState::Free),
        }
    }

    /// Remove the sample buffer for use.
    pub fn take(&self, new_state: BufferState) -> Option<&'static mut [usize]> {
        if self.samples.is_some() {
            self.current_state.set(new_state);
            self.samples.take()
        } else {
            None
        }
    }

    /// Replace the sample buffer.
    pub fn put(&self, buffer: &'static mut [usize], new_state: BufferState) {
        self.current_state.set(new_state);
        self.samples.put(Some(buffer))
    }

    /// Current state of samples in the buffer.
    pub fn state(&self) -> BufferState {
        self.current_state.get()
    }
}
