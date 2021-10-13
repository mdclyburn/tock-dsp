//! Hardware-backed synchronization facilities.
//!
//! Interfaces providing access to hardware that makes synchronization possible.

use core::ops::Drop;

use crate::errorcode::ErrorCode;

/// Operations for a spinlock.
///
/// A spinlock that additionally offers a manual deallocation method.
/// Types implementing this trait provide indefinite access to the holder of the spinlock
/// until the holder calls [`RawSpinlock::free`].
///
/// ''Access'' and ''claim'' to the lock are distinct.
/// ''Access'' means that code obtained the `RawSpinlock` type through some other code,
/// and the code is free to attempt to ''claim'' the lock at any point thereafter.
/// ''Claim'' means that code can attempt to change the state of the hardware backing the spinlock
/// (which may fail or succeed).
///
/// Once code successfully claims the spinlock through [`RawSpinlock::claim`] or [`RawSpinlock::try_claim`],
/// the claim holds until code calls [`RawSpinlock::release`].
///
/// Any types that make use of the `RawSpinlock` must ensure that its [`RawSpinlock::free`] is eventually called.
/// Failure to do so means that the `RawSpinlock` is leaked.
/// This type can be wrapped by the [`Spinlock`] type which will free the spinlock when it falls out of scope.
pub trait RawSpinlock {
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

/// Wrapper for the `RawSpinlock`.
///
/// Provides a _type_ to contain a [`RawSpinlock`].
/// Other than being a type and not a trait, it works the same way
/// (and leaks the same way, too!).
/// `UnmanagedSpinlock` allows other types to consume it as a type parameter in generics.
pub struct UnmanagedSpinlock {
    raw_sl: &'static dyn RawSpinlock,
}

impl RawSpinlock for UnmanagedSpinlock {
    fn try_claim(&self) -> bool {
        self.raw_sl.try_claim()
    }

    fn release(&self) {
        self.raw_sl.release();
    }

    fn free(&self) {
        self.raw_sl.free();
    }
}

impl From<&'static dyn RawSpinlock> for UnmanagedSpinlock {
    fn from(sl: &'static dyn RawSpinlock) -> UnmanagedSpinlock {
        UnmanagedSpinlock {
            raw_sl: sl,
        }
    }
}

/// Wrapper for `RawSpinlock` to promote hygienic use.
///
/// The `Spinlock` provides the same interface that `RawSpinlock` provides,
/// but it automatically handles the responsibility of `free`ing the `RawSpinlock`.
/// Instead, [`RawSpinlock::free`] is called once `Spinlock` falls out of scope.
///
/// Create a `Spinlock` with the `From<&'static RawSpinlock>` trait implementation.
///
/// See [`RawSpinlock`] for a detailed description of how to use the spinlock implementation.
pub struct Spinlock {
    raw_sl: &'static dyn RawSpinlock,
}

impl Spinlock {
    /// Attempt to claim the spinlock.
    pub fn try_claim(&self) -> bool {
        self.raw_sl.try_claim()
    }

    /// Attempt to claim the spinlock in a blocking manner.
    pub fn claim(&self) {
        self.raw_sl.claim();
    }

    /// Release the spinlock.
    pub fn release(&self) {
        self.raw_sl.release();
    }
}

impl From<&'static dyn RawSpinlock> for Spinlock {
    fn from(raw_sl: &'static dyn RawSpinlock) -> Spinlock {
        Spinlock {
            raw_sl,
        }
    }
}

impl Drop for Spinlock {
    fn drop(&mut self) {
        self.raw_sl.free();
    }
}

/// Hardware supporting synchronization-related operations.
pub trait HardwareSync {
    /// Allocate an unmanaged spinlock.
    ///
    /// Returns a [`RawSpinlock`] not wrapped by a `Spinlock`.
    /// This means that the caller must ensure that the spinlock is deallocated.
    /// Returns an `Ok(...)` on success.
    /// Returns an `Err(...)` on failure.
    fn get_raw_spinlock(&'static self) -> Result<&'static dyn RawSpinlock, ErrorCode>;

    /// Allocate a managed spinlock.
    ///
    /// The [`Spinlock`] this function returns will handle deallocating the contained `RawSpinlock`.
    ///
    /// Returns an `Ok(...)` on success.
    /// Returns an `Err(...)` on failure,
    /// which most likely means that hardware resources backing `RawSpinlock`s are exhausted at the time of the call.
    fn get_spinlock(&'static self) -> Result<Spinlock, ErrorCode> {
        self.get_raw_spinlock()
            .map(Spinlock::from)
    }

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
