//! Runtime synchronization

use core::cell::Cell;
use core::mem::MaybeUninit;

use rp2040::gpio::SIO;

use kernel::errorcode::ErrorCode;
use kernel::platform::sync::{
    HardwareSync,
    Spinlock,
};

#[used]
#[link_section = ".hardware_sync"]
static mut HARDWARE_SYNC_BLOCK: MaybeUninit<HardwareSyncBlock> = MaybeUninit::uninit();
static mut INSTANCE: Option<&HardwareSyncBlock> = None;

/// Spinlock implementated atop a hardware spinlock.
///
/// Each instance of SIOSpinlock represents exclusive access to one of the RP2040's
/// hardware spinlocks.
struct SIOSpinlock(u8);

impl SIOSpinlock {
    #[inline(always)]
    fn spinlock_no(&self) -> u8 { self.0 }
}

impl Spinlock for SIOSpinlock {
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
const HSB_SPINLOCK_NO: u8 = 0;

/// Hardware-backed synchronization support.
///
/// Provides and manages access to the RP2040's hardware spinlocks.
/// This interface offers up to 31 spinlocks for use by software.
/// One spinlock is reserved to mediate access to the HSB itself between the two cores.
pub struct HardwareSyncBlock {
    spinlocks: [SIOSpinlock; 32],
    allocation_state: Cell<u32>,
    sio: SIO,
}

impl HardwareSyncBlock {
    unsafe fn initialize(&'static mut self) {
        for (i, s) in (0..).zip(&mut self.spinlocks) { s.0 = i }

        let initial_allocation_state: u32 = 1 << HSB_SPINLOCK_NO;
        self.allocation_state = Cell::new(initial_allocation_state);
        self.sio = SIO::new();
    }

    pub fn allocation_state(&self) -> u32 {
        self.allocation_state.get()
    }

    fn allocate(&self, lock_no: u8) {
        let new_state = self.allocation_state.get() | (1 << lock_no);
        self.allocation_state.set(new_state);
    }

    fn deallocate(&self, lock_no: u8) {
        let new_state = self.allocation_state.get() ^ !(1 << lock_no);
        self.allocation_state.set(new_state);
    }
}

impl<'a> HardwareSync<'a> for HardwareSyncBlock {
    fn get_spinlock(&'a self) -> Result<&'a dyn Spinlock, ErrorCode> {
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
}

/// Perform initialization of the HSB.
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
