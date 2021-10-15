//! Hardware-backed synchronization facilities.
//!
//! Interfaces providing access to hardware that makes synchronization possible.

use core::ops::{Deref, Drop};

use crate::errorcode::ErrorCode;

/// Operations for a hardware spinlock.
///
/// Types implementing this trait provide indefinite access to the holder of the spinlock
/// until the holder calls [`HardwareSpinlock::free`].
///
/// "Access" and "claim" to the lock are distinct.
/// "Access" means that code obtained the `HardwareSpinlock` type through some other code,
/// and the code is free to attempt to "claim" the lock at any point thereafter.
/// "Claim" means that code can attempt to change the state of the hardware backing the spinlock
/// (which may fail or succeed).
///
/// Once code successfully claims the spinlock through [`HardwareSpinlock::claim`] or [`HardwareSpinlock::try_claim`],
/// the claim holds until code calls [`HardwareSpinlock::release`].
///
/// Any types that make use of the `HardwareSpinlock` must ensure that its [`HardwareSpinlock::free`] is eventually called.
/// Failure to do so means that the `HardwareSpinlock` is leaked.
/// [`ManagedSpinlock`] can wrap this type and free the spinlock when it falls out of scope.
pub trait HardwareSpinlock {
    /// Repeatedly attempt to claim the spinlock until it is successful.
    fn claim(&self) {
        while !self.try_claim() {  }
    }

    /// Attempt to claim the spinlock, returning `true` if successful.
    fn try_claim(&self) -> bool;

    /// Release the previously claimed spinlock.
    ///
    /// Return the spinlock to its unclaimed state.
    /// Calling this function when the caller did not previously claim the spinlock will have no effect.
    fn release(&self);

    /// Relinquish access to the spinlock.
    fn free(&self);
}

/// Type wrapper for `HardwareSpinlock` to provide deallocation on Drop.
///
/// The `Spinlock` provides the same interface that `HardwareSpinlock` provides,
/// but it automatically handles the responsibility of `free`ing the `HardwareSpinlock`,
/// calling [`HardwareSpinlock::free`] once `Spinlock` falls out of scope.
///
/// Create it with the `From<&'static Spinlock>` trait implementation.
pub struct ManagedSpinlock {
    raw_sl: &'static dyn HardwareSpinlock,
}

impl Deref for ManagedSpinlock {
    type Target = dyn HardwareSpinlock;

    fn deref(&self) -> &'static dyn HardwareSpinlock {
        self.raw_sl
    }
}

impl From<&'static dyn HardwareSpinlock> for ManagedSpinlock {
    fn from(raw_sl: &'static dyn HardwareSpinlock) -> ManagedSpinlock {
        ManagedSpinlock {
            raw_sl,
        }
    }
}

impl Drop for ManagedSpinlock {
    fn drop(&mut self) {
        self.raw_sl.free();
    }
}

/// Hardware supporting synchronization-related operations.
pub trait HardwareSync {
    /// Allocate an unmanaged spinlock.
    ///
    /// Returns a [`HardwareSpinlock`].
    /// The caller must deallocate the spinlock manually.
    ///
    /// Returns an `Ok(...)` on success.
    /// Returns an `Err(...)` on failure.
    fn get_spinlock(&'static self) -> Result<&'static dyn HardwareSpinlock, ErrorCode>;

    /// Returns the maximum number of spinlocks the platform supports.
    fn spinlocks_supported(&'static self) -> usize;

    /// Returns the number of spinlocks currently allocated.
    fn spinlocks_allocated(&'static self) -> usize;
}

/// Accessor for the [`HardwareSync`] interface.
///
/// Provides safe access to the hardware synchronization interface,
/// which may itself require locking to use.
pub trait HardwareSyncAccess {
    /// Provide access to the `HardwareSync` implementation.
    ///
    /// Locks access to hardware synchronization data and runs `f`,
    /// returning the result of `f` in a `Result::Ok`.
    ///
    /// # Returns
    /// - Result of the function `f` if HardwareSync was available.
    /// - `ErrorCode::BUSY` if `block` is false and `HardwareSync` was unavailable.
    /// - `ErrorCode::NODEVICE` if the functionality is not present.
    fn access<F, T>(&self, block: bool, f: F) -> Result<T, ErrorCode>
    where
        F: FnOnce(&'static dyn HardwareSync) -> T;
}

/// Implementation for hardware not supporting any kind of hardware synchronization.
impl HardwareSyncAccess for () {
    /// Always returns `ErrorCode::NODEVICE`.
    fn access<F, T>(&self, _block: bool, _f: F) -> Result<T, ErrorCode>
    where
        F: FnOnce(&'static dyn HardwareSync) -> T
    {
        Err(ErrorCode::NODEVICE)
    }
}
