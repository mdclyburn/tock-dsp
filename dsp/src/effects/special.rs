/*! Special-purpose processors.

Effects useful for purposes outside of typical audio signal processing are here.
These processors are oriented toward assisting with debugging and testing.
 */

use crate::signal::SignalProcessor;

/// Copy samples from the input buffer to the output buffer without additional processing.
///
/// `NoOp` is an effect useful for measuring the overhead of copying samples between buffers.
/// It should be the only effect in the chain during the benchmarking process.
/// With a 44.1kHz * 4 sampling rate and 20ms sample size
/// (each buffer is 3,528 * `sizeof(usize)` bytes = 3,528 * 4 = 11,412 bytes),
/// this processor takes ~227μs to run.
pub struct NoOp;

impl NoOp {
    pub fn new() -> NoOp { NoOp }
}

impl SignalProcessor for NoOp {
    fn process(&self, in_samples: &[i16], out_samples: &mut [i16]) {
        for (i, o) in in_samples.iter().zip(out_samples) {
            *o = *i;
        }
    }
}

/// Set all samples to zero.
///
/// A special-purpose effect processor for testing when the output is unimportant
/// or should not be visible
/// (e.g., inspecting output from an external DAC for noise).
pub struct Zero;

impl Zero {
    /// Create a new Zero processor.
    pub fn new() -> Zero { Zero }
}

impl SignalProcessor for Zero {
    fn process(&self, _in_samples: &[i16], out_samples: &mut [i16]) {
        for x in out_samples.iter_mut() {
            *x = 0;
        }
    }
}
