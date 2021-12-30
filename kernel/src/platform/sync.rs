//! Hardware-backed synchronization facilities.
//!
//! Interfaces providing access to hardware that makes synchronization possible.

use core::ops::{Deref, Drop};

use crate::errorcode::ErrorCode;
use crate::sync::Lockable;

/// Operations for a hardware spinlock.
///

/// Hardware supporting synchronization-related operations.
pub trait HardwareSync {
    /// Allocate a lock.
    fn get_lock(&'static self) -> Result<&'static dyn Lockable, ErrorCode>;

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
