#![allow(non_upper_case_globals)]

use rp2040;

pub const _estack_core1: *const u8 = 0x20042000 as *const u8;
pub const _sstack_core1: *const u8 = 0x20041000 as *const u8;

pub static mut CORE1_VECTORS: [usize; 16] = [
    0x0000_0000,

    0x0000_0000,
    0x0000_0000,
    0x0000_0000,
    0x0000_0000,
    0x0000_0000,
    0x0000_0000,
    0x0000_0000,
    0x0000_0000,
    0x0000_0000,
    0x0000_0000,
    0x0000_0000,
    0x0000_0000,
    0x0000_0000,
    0x0000_0000,
    0x0000_0000,
];

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
