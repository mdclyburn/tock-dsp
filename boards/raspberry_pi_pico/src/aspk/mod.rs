pub(crate) mod cnc;
mod interrupt;
mod startup;

pub use interrupt::VECTORS;
pub use interrupt::IRQS;

pub use startup::launch;

// Core1 stack space.
#[no_mangle]
#[link_section = ".core1_stack_buffer"]
pub static mut CORE1_STACK_MEMORY: [u8; 0x1000] = [0; 0x1000];

extern "C" {
    /// Start marker for core1's stack.
    pub static _core1_sstack: u8;
    /// End marker for core1's stack.
    pub static _core1_estack: u8;
}
