//! Runtime synchronization for the RP2040

use core::cell::Cell;

use cortexm0p::support;
use rp2040::gpio::SIO;

use kernel::static_init;
use kernel::errorcode::ErrorCode;
use kernel::platform::sync::{
    HardwareSync,
    HardwareSyncAccess,
    HardwareSpinlock,
};

/// Reference to the single instance of the hardware synchronization state.
///
/// Instance of the synchronization state used by `SIOSpinlock` to deallocate on Drop with minimal memory overhead.
static mut INSTANCE: Option<&'static HardwareSyncBlock> = None;

/// Spinlock implementated atop a hardware spinlock.
///
/// Each instance of SIOSpinlock represents exclusive access to one of the RP2040's
/// hardware spinlocks.
#[derive(Copy, Clone)]
struct SIOSpinlock(u8);

impl SIOSpinlock {
    /// Returns the index of the associated hardware spinlock.
    #[inline(always)]
    fn spinlock_no(&self) -> u8 { self.0 }

    /// Creates new spinlock states and tries to ensure they are in an unlocked state.
    unsafe fn enumerated() -> [SIOSpinlock; 32] {
        let sio = SIO::new();
        let mut spinlocks = [SIOSpinlock(0); 32];
        let mut idx = 0;
        while idx < spinlocks.len() {
            sio.release_spinlock(idx as u8);
            spinlocks[idx].0 = idx as u8;
            idx += 1;
        }

        spinlocks
    }
}

impl HardwareSpinlock for SIOSpinlock {
    fn try_claim(&self) -> bool {
        let sio = SIO::new();
        sio.claim_spinlock(self.spinlock_no())
    }

    fn release(&self) {
        let sio = SIO::new();
        sio.release_spinlock(self.spinlock_no());
    }

    fn free(&self) {
        // Call to free() implies SIOSpinlock was previously created by HardwareSyncBlock,
        // which implies that HardwareSyncBlock was previously created by HardwareSyncBlockAccess,
        // which implies that the instance exists.
        unsafe { INSTANCE.unwrap() }.deallocate(self.spinlock_no());
    }
}

// Use the first spinlock to mediate HSB access.
/// Hardware spinlock that module-internal code uses to make accessing the HSB safe.
const HSB_SPINLOCK_NO: u8 = 0;

/// Hardware-backed synchronization support.
///
/// Provides and manages access to the RP2040's hardware spinlocks.
/// This interface offers up to 31 spinlocks for use by software.
/// One spinlock is reserved to mediate access to the HSB itself between the two cores.
pub struct HardwareSyncBlock {
    spinlocks: [SIOSpinlock; 32],
    allocation_state: Cell<u32>,
}

impl HardwareSyncBlock {
    /// Create hardware synchronization tracking state.
    ///
    /// Code should not normally call this function.
    /// Instead, use [`HardwareSyncBlockAccess`] to gain access hardware synchronization resources.
    ///
    /// # Safety
    /// This function is unsafe because it tracks the state of hardware for which there is a single instance.
    /// Not only does creating this type with `new()` affect the hardware state (within [`SIOSpinlock::enumerated()`],
    /// but manipulating the hardware state with a second instance (or third, fourth...) will create inconsistencies that will result in unpredictable behavior.
    unsafe fn new() -> HardwareSyncBlock {
        let initial_allocation_state = 1 << HSB_SPINLOCK_NO;
        let hsb = HardwareSyncBlock {
            spinlocks: SIOSpinlock::enumerated(),
            allocation_state: Cell::new(initial_allocation_state),
        };

        hsb
    }

    /// Returns a bitmap representing the spinlocks that are allocated to consuming code.
    ///
    /// This function is useful for debugging purposes.
    /// The position of the bit indicates the spinlock number.
    /// A 0 means that no code is making use of the spinlock,
    /// and a 1 means that code is making use of the spinlock
    /// (e.g., `0b00010000` would mean code has allocated spinlock #4 for use).
    /// This is distinct from locking state.
    ///
    /// Note that at least one spinlock will always appear allocated as one is required to synchronize access to the `HardwareSyncBlock`.
    #[allow(dead_code)]
    pub fn allocation_state(&self) -> u32 {
        self.allocation_state.get()
    }

    /// Allocate a spinlock for use.
    fn allocate(&self, lock_no: u8) {
        let new_state = self.allocation_state.get() | (1 << lock_no);
        self.allocation_state.set(new_state);
    }

    /// Make a spinlock available for allocation.
    fn deallocate(&self, lock_no: u8) {
        let new_state = self.allocation_state.get() ^ (1 << lock_no);
        self.allocation_state.set(new_state);
    }
}

impl HardwareSync for HardwareSyncBlock {
    fn get_spinlock(&'static self) -> Result<&'static dyn HardwareSpinlock, ErrorCode> {
        let current_state = self.allocation_state.get();

        // Iterate through the bits until we find a free one.
        // The first is always reserved for access to the HSB itself.
        for i in 1u8..31 {
            let mask = 1u32 << i;
            if (mask & current_state) == 0 {
                self.allocate(i);
                return Ok(&self.spinlocks[i as usize]);
            }
        }

        Err(ErrorCode::FAIL)
    }

    fn spinlocks_supported(&'static self) -> usize { 32 }

    /// Returns the number of spinlocks that code has allocated.
    ///
    /// There will always be at least one spinlock allocated as one is used to synchronize access to the `HardwareSyncBlock`.
    fn spinlocks_allocated(&'static self) -> usize {
        self.allocation_state.get().count_ones() as usize
    }
}

/// Accessor for [`HardwareSyncBlock`].
///
/// Gateway to accessing to the [`HardwareSyncBlock`] instance.
/// On creation of the first instance, `HardwareSyncBlockAccess` will initialize the global hardware synchronization state.
/// Calls thereafter will only create a new instance of this (lightweight) type.
pub struct HardwareSyncBlockAccess { #[doc(hidden)] _private: () }

impl HardwareSyncBlockAccess {
    /// Create a new instance.
    ///
    /// # Safety
    /// Because the first call to this function will initialize the global synchronization state,
    /// the first call should be placed before code that is executing in a multicore environment
    /// (before core0 starts core1).
    pub unsafe fn new() -> HardwareSyncBlockAccess {
        if INSTANCE.is_none() {
            let new_instance: &'static _ = static_init!(HardwareSyncBlock, HardwareSyncBlock::new());
            INSTANCE = Some(new_instance);
        } else {
            INSTANCE.unwrap();
        }

        HardwareSyncBlockAccess { _private: () }
    }
}

impl HardwareSyncAccess for HardwareSyncBlockAccess {
    fn access<F, T>(&self, block: bool, f: F) -> Result<T, ErrorCode>
    where
        F: FnOnce(&'static dyn HardwareSync) -> T
    {
        let sio = SIO::new();
        loop {
            // Try at least once for the first try...
            if sio.claim_spinlock(HSB_SPINLOCK_NO) {
                let result = unsafe {
                    // Call to access() implies prior call to new(),
                    // prior call to new() implies instance exists.
                    let hsb = INSTANCE.unwrap();
                    let r = Ok(support::atomic(|| f(hsb)));
                    sio.release_spinlock(HSB_SPINLOCK_NO);
                    r
                };
                return result;
            } else if !block {
                // ... and if we're not blocking to get access and we failed,
                // we return with a BUSY code here...
                return Err(ErrorCode::BUSY);
            }
            // ... but if we're not blocking on this, then we just retry.
        }
    }
}
