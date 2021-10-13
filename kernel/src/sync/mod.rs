//! Synchronization primitives.

mod mutex;

use crate::platform::sync::{
    RawSpinlock,
    UnmanagedSpinlock,
};

pub use mutex::{
    Mutex,
    MutexGuard,
};

/// A type that provides locking for exclusive use.
///
/// This trait presents a uniform interface for synchronization primitives to consume and provide their functionality.
pub trait Lockable {
    /// Attempt to lock the resource, returning true when successful.
    fn try_lock(&self) -> bool;

    /// Lock the resource and block execution until it is locked.
    fn lock(&self) {
        while !self.try_lock() {  }
    }

    /// Release the previously locked resource.
    ///
    /// If the resource was not locked by a prior call to `try_lock()` or `lock()`,
    /// then a call to this function will have no effect.
    fn release(&self);
}

impl Lockable for UnmanagedSpinlock {
    fn try_lock(&self) -> bool {
        RawSpinlock::try_claim(self)
    }

    fn lock(&self) {
        RawSpinlock::claim(self);
    }

    fn release(&self) {
        RawSpinlock::release(self);
    }
}
