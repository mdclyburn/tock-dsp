//! Signal chain and link construction.

use core::cell::Cell;
use core::iter::{IntoIterator, Iterator};

use crate::utilities::cells::OptionalCell;

pub trait SignalProcessor {
    fn process(&self, samples: &'static mut [usize]);
}

pub struct Link {
    signal_processor: &'static dyn SignalProcessor,
    next: OptionalCell<&'static Link>,
}

impl Link {
    pub fn new(signal_processor: &'static dyn SignalProcessor) -> Link {
        Link {
            signal_processor,
            next: OptionalCell::empty(),
        }
    }
}

#[derive(Copy, Clone)]
pub struct Chain {
    link_head: &'static Link,
}

impl Chain {
    pub fn new(links: &[&'static Link]) -> Chain {
        let link_head = links[0];
        let mut prev_link = link_head;
        for link in &links[1..] {
            prev_link.next.set(link);
            prev_link = link;
        }

        Chain {
            link_head,
        }
    }
}

impl<'a> IntoIterator for &'a Chain {
    type Item = &'a Link;
    type IntoIter = ChainIt<'a>;

    fn into_iter(self) -> Self::IntoIter {
        ChainIt {
            current: Some(self.link_head),
        }
    }
}

pub struct ChainIt<'a> {
    current: Option<&'a Link>,
}

impl<'a> Iterator for ChainIt<'a> {
    type Item = &'a Link;

    fn next(&mut self) -> Option<Self::Item> {
        let out = self.current;
        if let Some(link) = self.current {
            self.current = link.next.extract();
        }

        out
    }
}
