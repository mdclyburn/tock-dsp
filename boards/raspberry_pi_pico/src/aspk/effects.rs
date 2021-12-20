//! Audio signal processing effects.

use kernel::dsp::link::SignalProcessor;

pub struct NoOp {  }

impl NoOp {
    pub fn new() -> NoOp {
        NoOp {  }
    }
}

impl SignalProcessor for NoOp {
    fn process(&self, _in_samples: &[usize], _out_samples: &mut [usize]) {  }
}

/// Copy samples from the input buffer to the output buffer without additional processing.
///
/// `NoOpCopy` is an effect useful for measuring the overhead of copying samples between buffers.
/// It should be the only effect in the chain during the benchmarking process.
/// With a 44.1kHz * 4 sampling rate and 20ms sample size
/// (each buffer is 3,528 * `sizeof(usize)` bytes = 3,528 * 4 = 11,412 bytes),
/// this processor takes ~227μs to run.
pub struct NoOpCopy;

impl NoOpCopy {
    pub fn new() -> NoOpCopy { NoOpCopy }
}

impl SignalProcessor for NoOpCopy {
    fn process(&self, in_samples: &[usize], out_samples: &mut [usize]) {
        for (i, o) in in_samples.iter().zip(out_samples) {
            *o = *i;
        }
    }
}

/// Scale
pub struct Scale {
    baseline: usize,
    nominator: u8,
    denominator: u8,
}

impl Scale {
    pub fn new(nominator: u8, denominator: u8) -> Scale {
        Scale {
            baseline: (2 << 12) / 2,
            nominator,
            denominator,
        }
    }
}

impl SignalProcessor for Scale {
    fn process(&self, in_samples: &[usize], out_samples: &mut [usize]) {
        let samples_it = in_samples.iter().zip(out_samples);
        for (in_sample, out_sample) in samples_it {
            *out_sample = if *in_sample < self.baseline {
                self.baseline - ((self.baseline - *in_sample)
                            * self.nominator as usize / self.denominator as usize)
            } else {
                self.baseline + ((*in_sample - self.baseline)
                            * self.nominator as usize / self.denominator as usize)
            };
        }
    }
}
