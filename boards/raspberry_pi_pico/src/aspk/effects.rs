//! Audio signal processing effects.

use kernel::dsp::link::SignalProcessor;

pub struct NoOp {  }

impl NoOp {
    pub fn new() -> NoOp {
        NoOp {  }
    }
}

impl SignalProcessor for NoOp {
    fn process(&self, _samples: &mut [usize]) {  }
}

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
    fn process(&self, samples: &mut [usize]) {
        for sample in samples {
            *sample = if *sample < self.baseline {
                self.baseline - ((self.baseline - *sample)
                            * self.nominator as usize / self.denominator as usize)
            } else {
                self.baseline + ((*sample - self.baseline)
                            * self.nominator as usize / self.denominator as usize)
            };
        }
    }
}
