//! Hardware-backed synchronization facilities.
//!
//! Interfaces providing access to hardware that makes synchronization possible.

use core::ops::Drop;

use crate::errorcode::ErrorCode;

/// Supported operations for a hardware-based spinlock.
///
/// Access to this trait represents access to a single hardware spinlock.
/// Types implementing this trait provide indefinite access to the holder of the spinlock until the holder calls [`HardwareSpinlock::free`].
///
/// ''Access'' and ''claim'' to the lock are distinct.
/// ''Access'' means that code obtained the `HardwareSpinlock` type through some other code,
/// and the code is free to attempt to ''claim'' the lock at any point thereafter.
/// ''Claim'' means that code can attempt to change the state of the hardware backing the spinlock
/// (which may fail or succeed).
///
/// Once code successfully claims the spinlock through [`HardwareSpinlock::claim`] or [`HardwareSpinlock::try_claim`],
/// the claim holds until code calls [`HardwareSpinlock::release`].
///
/// Any types that make use of the `HardwareSpinlock` must ensure that its [`HardwareSpinlock::free`] is eventually called.
/// Failure to do so means that the `HardwareSpinlock` is leaked.
/// This type is meant to be wrapped by the [`Spinlock`] type which will free the spinlock when it falls out of scope.
/// This way, it prevents accidental use of `HardwareSpinlock` after code has freed it.
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
    /// Calling this function when the spinlock is not claimed by the caller will have no effect.
    fn release(&self);

    /// Relinquish access to the spinlock.
    fn free(&self);
}

/// Wrapper for [`HardwareSpinlock`] to promote hygienic use.
///
/// The `Spinlock` provides the same interface that `HardwareSpinlock` provides,
/// but it automatically handles the responsibility of `free`ing the `HardwareSpinlock`.
/// Instead, [`HardwareSpinlock::free`] is called once `Spinlock` falls out of scope.
///
/// See [`HardwareSpinlock`] for a detailed description of how to use the spinlock implementation.
pub struct Spinlock {
    hw_spinlock: &'static dyn HardwareSpinlock,
}

impl Spinlock {
    /// Wrap a `HardwareSpinlock` in a new `Spinlock`.
    pub fn new(hw_spinlock: &'static dyn HardwareSpinlock) -> Spinlock {
        Spinlock {
            hw_spinlock,
        }
    }

    /// Attempt to claim the spinlock.
    pub fn try_claim(&self) -> bool {
        self.hw_spinlock.try_claim()
    }

    /// Attempt to claim the spinlock in a blocking manner.
    pub fn claim(&self) {
        self.hw_spinlock.claim();
    }

    /// Release the spinlock.
    pub fn release(&self) {
        self.hw_spinlock.release();
    }
}

impl Drop for Spinlock {
    fn drop(&mut self) {
        self.hw_spinlock.free();
    }
}

/// Hardware supporting synchronization-related operations.
pub trait HardwareSync {
    /// Allocate a hardware-based spinlock.
    ///
    /// Returns an `Ok(Spinlock)` on success.
    /// Returns an `Err(ErrorCode)` on failure.
    /// Failure most likely means that hardware resources backing `Spinlock`s are exhausted at the time of the call.
    fn get_spinlock(&'static self) -> Result<Spinlock, ErrorCode>;
}
