//! Hardware-backed synchronization facilities.
//!
//! Interfaces providing access to hardware that makes synchronization possible.

use crate::errorcode::ErrorCode;

/// Supported operations for a hardware-based spinlock.
///
/// Access to this trait represents access to a single hardware spinlock.
/// Types implementing this trait provide indefinite access to the holder of the spinlock until the holder calls [`Spinlock::free`].
///
/// ''Access'' and ''claim'' to the lock are distinct.
/// ''Access'' means that code obtained the `Spinlock` type through some other code,
/// and the code is free to attempt to ''claim'' the lock at any point thereafter.
/// ''Claim'' means that code can attempt to change the state of the hardware backing the spinlock
/// (which may fail or succeed).
///
/// Once code successfully claims the spinlock through [`Spinlock::claim`] or [`Spinlock::try_claim`],
/// the claim holds until code calls [`Spinlock::release`].
///
/// Any types that make use of the `Spinlock` must ensure that its [`Spinlock::free`] is eventually called.
/// Failure to do so means that the `Spinlock` is leaked.
pub trait Spinlock {
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

/// Hardware supporting synchronization-related operations.
pub trait HardwareSync<'a> {
    /// Allocate a hardware-based spinlock.
    ///
    /// Returns an `Ok(&'a dyn Spinlock)` on success.
    /// Returns an `Err(ErrorCode)` on failure.
    /// Failure most likely means that hardware resources backing `Spinlock`s are exhausted at the time of the call.
    fn get_spinlock(&'a self) -> Result<&'a dyn Spinlock, ErrorCode>;
}
