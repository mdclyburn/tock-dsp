//! Support for custom sample processing.

use core::cell::Cell;
use core::iter::Iterator;

pub trait Link {
    fn next(&self) -> Option<&'static dyn Link>;

    fn process(&self, samples: &'static mut [usize]);
}

pub(super) struct LinkIt {
    current: &'static dyn Link,
}

impl LinkIt {
    pub(super) fn new(first_link: &'static dyn Link) -> LinkIt {
        LinkIt {
            current: first_link,
        }
    }
}

impl Iterator for LinkIt {
    type Item = &'static dyn Link;

    fn next(&mut self) -> Option<Self::Item> {
        let next_link = self.current.next();
        if let Some(link) = next_link {
            self.current = link;
        }

        next_link
    }
}
