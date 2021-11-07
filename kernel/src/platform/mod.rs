//! Traits for implementing various layers and components in Tock.
//!
//! Implementations of these traits are used by the core kernel.

pub mod chip;
pub mod interprocessor;
pub mod mpu;
pub mod scheduler_timer;
pub mod sync;
pub mod watchdog;

pub(crate) mod platform;

pub use self::platform::KernelResources;
pub use self::platform::ProcessFault;
pub use self::platform::SyscallDriverLookup;
pub use self::platform::SyscallFilter;
