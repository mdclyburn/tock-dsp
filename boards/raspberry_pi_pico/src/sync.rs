//! Runtime synchronization for the RP2040

use core::cell::Cell;
use core::mem::MaybeUninit;

use rp2040::gpio::SIO;

use kernel::errorcode::ErrorCode;
use kernel::platform::sync::{
    HardwareSync,
    HardwareSpinlock,
    Spinlock,
};

/// Hardware synchronization metadata.
#[used]
#[link_section = ".hardware_sync"]
static mut HARDWARE_SYNC_BLOCK: MaybeUninit<HardwareSyncBlock> = MaybeUninit::uninit();
/// Initialized single instance of the `HardwareSyncBlock`.
static mut INSTANCE: Option<&HardwareSyncBlock> = None;

/// Spinlock implementated atop a hardware spinlock.
///
/// Each instance of SIOSpinlock represents exclusive access to one of the RP2040's
/// hardware spinlocks.
struct SIOSpinlock(u8);

impl SIOSpinlock {
    /// Returns the index of the associated hardware spinlock.
    #[inline(always)]
    fn spinlock_no(&self) -> u8 { self.0 }
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
        with_hsb(|hsb| hsb.deallocate(self.spinlock_no()));
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
    /// Initialize the tracking state of hardware synchronization.
    ///
    /// # Safety
    /// This function is unsafe because it initializes global mutable state.
    /// It should only be called once during the startup sequence.
    ///
    /// It is not clear what happens to the spinlock claim state if a core is reset.
    /// Here, we write to all spinlocks to attempt to unlock them if they are locked.
    unsafe fn initialize(&'static mut self) {
        let sio = SIO::new();
        for (i, s) in (0..).zip(&mut self.spinlocks) {
            s.0 = i;
            // Try to release the spinlock of this is not from clean start.
            sio.release_spinlock(i);
        }

        let initial_allocation_state: u32 = 1 << HSB_SPINLOCK_NO;
        self.allocation_state = Cell::new(initial_allocation_state);
    }

    /// Returns a bitmap representing the spinlocks that are allocated to consuming code.
    ///
    /// The position of the bit indicates the spinlock number.
    /// A 0 means that no code is making use of the spinlock,
    /// and a 1 means that code is making use of the spinlock
    /// (e.g., `0b00010000` would mean code has allocated spinlock #4 for use).
    /// This is distinct from locking state.
    ///
    /// Note that at least one spinlock will always appear allocated as one is required to synchronize access to the `HardwareSyncBlock`.
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

impl<'a> HardwareSync<'a> for HardwareSyncBlock {
    fn get_spinlock(&'a self) -> Result<Spinlock<'a>, ErrorCode> {
        let current_state = self.allocation_state.get();

        // Iterate through the bits until we find a free one.
        // The first is always reserved for access to the HSB itself.
        for i in 1u8..31 {
            let mask = 1u32 << i;
            if (mask & current_state) == 0 {
                self.allocate(i);
                return Ok(Spinlock::new(&self.spinlocks[i as usize]));
            }
        }

        Err(ErrorCode::FAIL)
    }
}

/// Perform initialization of the HSB.
///
/// # Safety
/// This function is unsafe because it initializes global mutable state.
/// It should only be called once during the startup sequence.
///
/// # Panics
/// - When called more than once.
pub unsafe fn initialize_hsb() {
    if INSTANCE.is_some() {
        panic!("Attempted to initialize HSB more than once.");
    }

    (*HARDWARE_SYNC_BLOCK.as_mut_ptr()).initialize();
    INSTANCE = HARDWARE_SYNC_BLOCK.as_ptr().as_ref();
}

/// Returns the hardware synchronization interface.
///
/// Obtains access to the [`HardwareSyncBlock`] (or waits until it can do so)
/// and allows use of it by the provided closure.
///
/// # Panics
/// - When [`initialize_hsb`] has not previously been called.
pub fn with_hsb<F, T>(f: F) -> T
where
    F: FnOnce(&'static HardwareSyncBlock) -> T
{
    // Because the HSB is unavailable until we can claim it, we need to
    // use raw access to the hardware spinlock to try to claim the lock
    // set aside for HSB access.
    let sio = SIO::new();
    while !sio.claim_spinlock(HSB_SPINLOCK_NO) {  }

    let r = unsafe {
        match INSTANCE {
            Some(hsb) => f(hsb),
            None => panic!("Cannot use HSB before it has been initialized!"),
        }
    };

    sio.release_spinlock(HSB_SPINLOCK_NO);

    r
}
