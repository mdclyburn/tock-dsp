//! Audio signal processing effects.

use kernel::dsp::link::Link;

pub struct NoOp {  }

impl NoOp {
    pub fn new() -> NoOp {
        NoOp {  }
    }
}

impl Link for NoOp {
    fn next(&self) -> Option<&'static dyn Link> { None }

    fn process(&self, samples: &'static mut [usize]) {  }
}
