#![allow(non_upper_case_globals)]

use rp2040;

// Core1 stack space.
// Slightly reduced for vector and IRQs, aligned to 256 bytes.
#[no_mangle]
#[link_section = ".core1_stack_buffer"]
pub static mut CORE1_STACK_MEMORY: [u8; 0x1000 - 256] = [0; 0x1000 - 256];

extern "C" {
    pub static _core1_sstack: u8;
    pub static _core1_estack: u8;
}

#[used]
#[link_section = ".core1_vectors"]
pub static mut CORE1_VECTORS: [usize; 16] = [0x0000_0000; 16];

#[used]
#[link_section = ".core1_irqs"]
pub static mut CORE1_IRQS: [usize; 32] = [0x0000_0000; 32];

#[no_mangle]
pub unsafe fn aspk_main() {
    let sio = rp2040::gpio::SIO::new();

    sio.write_fifo(0x41_53_50_4B);

    loop {
        asm!("wfe", options(nomem, preserves_flags));
        while sio.fifo_valid() {
            let _ = sio.read_fifo();
        }
    }
}
