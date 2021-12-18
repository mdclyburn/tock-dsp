//! Audio signal processing effects.

use kernel::dsp::link::SignalProcessor;

pub struct NoOp {  }

impl NoOp {
    pub fn new() -> NoOp {
        NoOp {  }
    }
}

impl SignalProcessor for NoOp {
    fn process(&self, input_buffer: &'static [usize], output_buffer: &'static mut [usize]) {  }
}
