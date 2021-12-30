//! Synchronization primitives.

mod mutex;
mod semaphore;

pub use mutex::{
    Mutex,
    MutexGuard,
};
pub use semaphore::Semaphore;

/// A type that provides locking for exclusive use.
///
/// Types implementing this trait provide indefinite access to the holder of the lock.
/// until the holder calls [`Lockable::release`].
///
/// "Access" and "claim" to the lock are distinct.
/// "Access" means that code obtained the `HardwareSpinlock` type through some other code,
/// and the code is free to attempt to "claim" the lock at any point thereafter.
/// "Claim" means that code can attempt to change the state of the hardware backing the lock
/// (which may fail or succeed).
///
/// Once code successfully claims the lock through [`Lockable::lock`] or [`Lockable::try_lock`],
/// the claim holds until code calls [`Lockable::release`].
pub trait Lockable: 'static {
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
