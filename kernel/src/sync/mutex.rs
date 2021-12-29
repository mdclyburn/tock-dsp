//! Mutex synchronization primitive.

use core::ops::{Deref, DerefMut};

use crate::errorcode::ErrorCode;

use super::Lockable;

/// Result of a successful mutex lock operation.
///
/// [`Mutex`] creates a `MutexGuard` when a lock operation succeeds.
/// The type grants exclusive access to the resource that is protected by the `Mutex`.
/// When an instance falls out of scope, it automatically releases the lock on the spawning `Mutex`.
pub struct MutexGuard<'a, T: 'static> {
    scoped_lock: &'a dyn Lockable,
    resource_access: &'static T,
}

impl<'a, T> MutexGuard<'a, T> {
    fn new(scoped_lock: &'a dyn Lockable, resource_access: &'static T) -> MutexGuard<'a, T> {
        MutexGuard {
            scoped_lock,
            resource_access,
        }
    }

    fn map<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut T) -> R,
    {
        // Mutex only accepts a static, mutable reference to T.
        // However, we cannot copy or clone &'static T (for good reason),
        // but we still want callers that properly lock this state to have
        // the ability to mutate it.
        // And thus...
        let r = unsafe {
            ((self.resource_access as *const T) as *mut T)
                .as_mut()
                .unwrap() // Bad if the pointer is a null pointer.
        };
        f(r)
    }
}

impl<'a, T> Deref for MutexGuard<'a, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        self.resource_access
    }
}

impl<'a, T> Drop for MutexGuard<'a, T> {
    fn drop(&mut self) {
        self.scoped_lock.release();
    }
}

/// Mutually exclusive access provider.
#[derive(Copy, Clone)]
pub struct Mutex<L: Lockable, T: 'static> {
    lock: L,
    resource: &'static T,
}

impl<L: Lockable, T> Mutex<L, T> {
    /// Create a new Mutex.
    pub fn new(lock: L, resource: &'static mut T) -> Mutex<L, T> {
        Mutex {
            lock,
            resource,
        }
    }

    /// Attempt to gain access to the resource guarded by the Mutex.
    ///
    /// This function returns a `Result` that, if `Ok`, contains a `MutexGuard` the caller may use to access the resource.
    pub fn try_lock(&self) -> Result<MutexGuard<'_, T>, ErrorCode> {
        if !self.lock.try_lock() {
            Err(ErrorCode::BUSY)
        } else {
            Ok(MutexGuard::new(&self.lock, self.resource))
        }
    }

    /// Repeatedly attempt to access the resource until successful.
    pub fn lock(&self) -> MutexGuard<'_, T> {
        while !self.lock.try_lock() {  }

        MutexGuard::new(&self.lock, self.resource)
    }

    /// Attempt to gain access to the resource to run an operation.
    ///
    /// The equivalent of [`Mutex::try_lock()`],
    /// but also accepts and executes a lambda on the contained resource.
    pub fn try_map<F, R>(&self, f: F) -> Result<R, ErrorCode>
    where
        F: FnOnce(&mut T) -> R,
    {
        self.try_lock().map(|guard| guard.map(f))
    }

    /// Repeatedly attempt to access the resource, running an operation when successful.
    pub fn map<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut T) -> R
    {
        let guard = self.lock();
        guard.map(f)
    }
}
