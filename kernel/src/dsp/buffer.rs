//! Audio sample buffer management.

use crate::config;
use crate::static_buf;
use crate::utilities::cells::{TakeCell, VolatileCell};

/// Current status of samples in an [`SampleContainer`].
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
pub struct SampleContainer {
    samples: TakeCell<'static, [usize]>,
    buffer_state: VolatileCell<BufferState>,
}

impl SampleContainer {
    /// Create an [`SampleContainer`].
    pub unsafe fn new() -> SampleContainer {
        let sample_buffer = static_buf!([usize; config::NO_BUFFER_ENTRIES])
            .initialize([0; config::NO_BUFFER_ENTRIES]);

        SampleContainer {
            samples: TakeCell::new(sample_buffer),
            buffer_state: VolatileCell::new(BufferState::Free),
        }
    }

    /// Remove the sample buffer for use.
    pub fn take(&self, new_state: BufferState) -> Option<&'static mut [usize]> {
        if self.samples.is_some() {
            self.buffer_state.set(new_state);
            self.samples.take()
        } else {
            None
        }
    }

    /// Replace the sample buffer.
    pub fn put(&self, buffer: &'static mut [usize], new_state: BufferState) {
        self.buffer_state.set(new_state);
        self.samples.put(Some(buffer))
    }

    /// Current state of samples in the buffer.
    pub fn state(&self) -> BufferState {
        self.buffer_state.get()
    }
}
