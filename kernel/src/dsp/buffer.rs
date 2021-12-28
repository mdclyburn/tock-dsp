//! Audio sample buffer management.

use core::mem::MaybeUninit;

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
    /// Create a [`SampleContainer`].
    pub unsafe fn new(samples_buffer: &'static mut [usize; config::NO_BUFFER_ENTRIES]) -> SampleContainer {
        SampleContainer {
            samples: TakeCell::new(samples_buffer),
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

    /// Memory address of the contained buffer, or None if buffer is not present.
    pub fn addr(&self) -> Option<usize> {
        if let Some(buffer) = self.samples.take() {
            let addr = buffer.as_ptr() as usize;
            self.samples.put(Some(buffer));
            Some(addr)
        } else {
            None
        }
    }
}
