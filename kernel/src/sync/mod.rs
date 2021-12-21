//! Synchronization primitives.

mod mutex;
mod semaphore;

use crate::platform::sync::{
    HardwareSpinlock,
    ManagedSpinlock,
    UnmanagedSpinlock,
};

pub use mutex::{
    Mutex,
    MutexGuard,
};
pub use semaphore::Semaphore;

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

impl Lockable for ManagedSpinlock {
    fn try_lock(&self) -> bool {
        self.try_claim()
    }

    fn lock(&self) {
        self.claim();
    }

    fn release(&self) {
        HardwareSpinlock::release(core::ops::Deref::deref(self));
    }
}

impl Lockable for UnmanagedSpinlock {
    fn try_lock(&self) -> bool {
        self.try_claim()
    }

    fn lock(&self) {
        self.claim();
    }

    fn release(&self) {
        HardwareSpinlock::release(core::ops::Deref::deref(self));
    }
}
