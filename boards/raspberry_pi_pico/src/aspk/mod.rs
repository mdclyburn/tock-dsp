mod effects;
mod interrupt;
mod startup;

pub use interrupt::VECTORS;
pub use interrupt::IRQS;

pub use startup::launch;

// Core1 stack space.
// Slightly reduced for vector and IRQs, aligned to 256 bytes.
/// Core1 stack space.
#[no_mangle]
#[link_section = ".core1_stack_buffer"]
pub static mut CORE1_STACK_MEMORY: [u8; 0x1000 - 256] = [0; 0x1000 - 256];

#[no_mangle]
#[link_section = ".processing_stack_buffer"]
static mut PROCESSING_STACK_MEMORY: [u8; 0x0800] = [0; 0x0800];

extern "C" {
    /// Start marker for core1's stack.
    pub static _core1_sstack: u8;
    /// End marker for core1's stack.
    pub static _core1_estack: u8;
    /// Start marker for the processing stack.
    pub static _processing_sstack: u8;
    /// End marker for the processing stack.
    pub static _processing_estack: u8;
}
