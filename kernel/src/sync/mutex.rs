//! Mutex synchronization primitive.

use core::ops::Deref;

use crate::errorcode::ErrorCode;

use super::Lockable;

/// Result of a successful mutex lock operation.
///
/// [`Mutex`] creates a `MutexGuard` when a lock operation succeeds.
/// The type grants exclusive access to the resource that is protected by the `Mutex`.
/// When an instance falls out of scope, it automatically releases the lock on the spawning `Mutex`.
pub struct MutexGuard<'a, 'b, T> {
    scoped_lock: &'a dyn Lockable,
    resource_access: &'b T,
}

impl<'a, 'b, T> MutexGuard<'a, 'b, T> {
    fn new(scoped_lock: &'a dyn Lockable, resource_access: &'b T) -> MutexGuard<'a, 'b, T> {
        MutexGuard {
            scoped_lock,
            resource_access,
        }
    }
}

impl <'a, 'b, T> Deref for MutexGuard<'a, 'b, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        self.resource_access
    }
}

impl<'a, 'b, T> Drop for MutexGuard<'a, 'b, T> {
    fn drop(&mut self) {
        self.scoped_lock.release();
    }
}

/// Mutually exclusive access provider.
#[derive(Copy, Clone)] // TODO: allowing copies isn't quite right.
pub struct Mutex<L: Lockable, T> {
    lock: L,
    resource: T,
}

impl<L: Lockable, T> Mutex<L, T> {
    /// Create a new Mutex.
    pub fn new(lock: L, resource: T) -> Mutex<L, T> {
        Mutex {
            lock,
            resource,
        }
    }

    /// Attempt to gain access to the resource guarded by the Mutex.
    ///
    /// This function returns a `Result` that, if `Ok`, contains a `MutexGuard` the caller may use to access the resource.
    pub fn try_lock(&self) -> Result<MutexGuard<'_, '_, T>, ErrorCode> {
        if !self.lock.try_lock() {
            Err(ErrorCode::BUSY)
        } else {
            Ok(MutexGuard::new(&self.lock, &self.resource))
        }
    }

    /// Repeatedly attempt to access the resource until successful.
    pub fn lock(&self) -> MutexGuard<'_, '_, T> {
        while !self.lock.try_lock() {  }

        MutexGuard::new(&self.lock, &self.resource)
    }
}
