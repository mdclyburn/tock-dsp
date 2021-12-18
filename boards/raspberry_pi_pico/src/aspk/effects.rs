//! Audio signal processing effects.

use kernel::dsp::link::SignalProcessor;

pub struct NoOp {  }

impl NoOp {
    pub fn new() -> NoOp {
        NoOp {  }
    }
}

impl SignalProcessor for NoOp {
    fn process(&self, samples: &'static mut [usize]) {  }
}
