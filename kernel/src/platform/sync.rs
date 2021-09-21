//! Hardware synchronization facilities

use crate::errorcode::ErrorCode;

/// Hardware spinlock.
pub trait Spinlock {
    fn claim(&self) {
        while !self.try_claim() {  }
    }

    fn try_claim(&self) -> bool;

    fn release(&self);

    fn free(&self);
}

/// Hardware supporting synchronization-related operations.
pub trait HardwareSync<'a> {
    fn get_spinlock(&'a self) -> Result<&'a dyn Spinlock, ErrorCode>;
}
