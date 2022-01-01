/*! Delay-based effects processors.
 */

use core::cell::Cell;

use kernel::dsp::engine;
use kernel::dsp::link::SignalProcessor;
use kernel::utilities::cells::MapCell;

/// Moving window state.
#[derive(Clone, Copy)]
enum Sweep {
    /// Leftward moving.
    Left(usize),
    /// Rightward moving.
    Right(usize),
}

impl Sweep {
    /// Returns the offset value.
    #[inline]
    fn offset(&self) -> usize {
        use Sweep::*;
        match self {
            Left(offset) => *offset,
            Right(offset) => *offset,
        }
    }
}

/// Mix a signal with itself with a small, varying delay.
///
/// Captures a varying amount of previously-encountered samples and layers it with a later sample.
/// The delay for flangers is typically very small,
/// on the order of a few milliseconds.
/// Thus, the maximum delay of this implementation is limited to the length of the sample buffer.
///
/// With a 44.1kHz sampling rate and 20ms sample size
/// (each buffer is 882 samples)
/// this processor takes 4.2 to 4.4 milliseconds to run.
pub struct Flange {
    /// Progression of the moving offset.
    cycle: Cell<Sweep>,
    /// Number of samples to advance in each processing cycle.
    stride: usize,
    /// Max number of samples to displace the source by.
    max_offset: usize,
    /// Samples held over from the previous processing cycle.
    overflow: MapCell<[i16; engine::buffer_len_samples()]>,
}

impl Flange {
    /// Create a new flanging processor.
    ///
    /// The maximum offset is determined by `max_offset_us`.
    /// A complete cycle from no offset to `max_offset_us` and back to no offset.
    /// This cycle will happen over about `cycle_len_ms` milliseconds.
    pub fn new(max_offset_us: usize, cycle_len_ms: usize) -> Flange {
        let max_offset =
            engine::sampling_rate()
            * max_offset_us / 1_000_000;
        let stride =
            engine::buffer_len_samples() * 2 // Samples in one cycle.
            / (cycle_len_ms / engine::buffer_len_ms()); // No. of times process() executes in cycle_len_ms.

        Flange {
            cycle: Cell::new(Sweep::Left(0)),
            stride,
            max_offset,
            overflow: MapCell::new([0; engine::buffer_len_samples()]),
        }
    }

    #[inline]
    fn mix(base: i16, add: i16) -> i16 {
        ((base / 3) * 2) + (add / 3)
    }
}

impl SignalProcessor for Flange {
    fn process(&self, in_samples: &[i16], out_samples: &mut [i16]) {
        let _ = self.overflow.map(|overflow| {
            let sweep = self.cycle.get();

            // Copy <current_offset> samples from in_samples to out_samples,
            // and mix these with the contents of the overflow buffer.
            // Sweep contains the number of samples we need from the overflow buffer.
            for idx in 0..sweep.offset() {
                out_samples[idx] = Self::mix(in_samples[idx], overflow[idx]);
            }

            // Copy the rest of the samples from in_samples to out_samples,
            // and mix these starting with the beginning of the in_samples.
            for (out_idx, mix_idx) in (sweep.offset()..out_samples.len()).zip(0..) {
                out_samples[out_idx] = Self::mix(in_samples[out_idx], in_samples[mix_idx])
            }
            // Next sample input index:
            let overflow_src_idx = in_samples.len() - sweep.offset();

            // If we increase the offset, then we extend the last sample to lengthen the overflow.
            // If we decrease the offset, then we do nothing and effectively drop samples in the next cycle.
            // Copy the rest of the samples from in_samples to the overflow buffer.
            match sweep {
                Sweep::Left(offset) => {
                    if offset + self.stride > self.max_offset {
                        // Limit reached.
                        // Switch sweep directions and set the new offset.
                        self.cycle.set(Sweep::Right(offset - self.stride));
                        // Changing sweep to a right direction means self.stride less samples.
                        // Use the overflow_src_idx + self.stride to exclude some samples.
                        for (of_idx, in_idx) in (0..).zip((overflow_src_idx+self.stride)..in_samples.len()) {
                            overflow[of_idx] = in_samples[in_idx];
                        }
                    } else {
                        // Increase the offset.
                        self.cycle.set(Sweep::Left(offset + self.stride));
                        // Insert the last sample repeated self.stride times to fillin the gaps.
                        for idx in 0..self.stride {
                            overflow[idx] = in_samples[overflow_src_idx-1];
                        }
                        // Copy remaining input samples to the overflow buffer.
                        for (of_idx, in_idx) in (self.stride..).zip(overflow_src_idx..in_samples.len()) {
                            overflow[of_idx] = in_samples[in_idx];
                        }
                    }
                },

                Sweep::Right(offset) => {
                    if offset < self.stride {
                        // Samples align; no overflow to copy.
                        self.cycle.set(Sweep::Left(0));
                    } else {
                        // Decrease the offset.
                        self.cycle.set(Sweep::Right(offset - self.stride));
                        // Copy remaining input to the overflow buffer.
                        // There are less samples to copy now.
                        // Use overflow_src_idx + self.stride to skip over some samples.
                        for (of_idx, in_idx) in (0..).zip((overflow_src_idx+self.stride)..in_samples.len()) {
                            overflow[of_idx] = in_samples[in_idx];
                        }
                    }
                },
            };
        });
    }
}
