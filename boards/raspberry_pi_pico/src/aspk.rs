#![allow(non_upper_case_globals)]

use kernel::{
    Kernel,
    debug,
};
use cortexm0p::nvic;
use rp2040::{
    self,
    gpio::SIO,
    interrupts,
};

use crate::{RP2040Chip, RaspberryPiPico};
use crate::sync;

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
    rp2040::init();

    // Initialize all vectors and IRQ handlers to something.
    // Skip the first two since that is the initial stack
    // pointer value and the reset vector.
    // for (no, vec) in (0..).zip(&mut CORE1_VECTORS[2..]) {
    //     *vec = match no {
    //         2 | 3 | 11 | 14 => fail_interrupt as *const fn() as usize,
    //         _ => ignored_interrupt as *const fn() as usize,
    //     };
    // };

    // for irq in &mut CORE1_IRQS {
    //     *irq = ignored_interrupt as usize;
    // }

    // let hw_sync_access = sync::HardwareSyncBlockAccess::new();
    // let _sl = {
    //     use kernel::platform::sync::HardwareSyncAccess;
    //     hw_sync_access.access(true, |hsb| hsb.get_spinlock())
    // };

    let sio = SIO::new();

    while !sio.fifo_ready() {  }
    sio.write_fifo(0x41_53_50_4B);
    debug!("ASPK launched.");

    // The first three words from the other side are the kernel, board, and chip resources.
    // let (kernel, board_resources, chip_resources) = receive_resources(&sio);
    // debug!("ASPK received resources!");

    let mut b = true;
    loop {
        if b {
            debug!("In the loop!");
            b = false;
        }
        // debug!("ASPK running...");
        // cortexm0p::support::wfe();
        // while sio.fifo_valid() {
        //     let _ = sio.read_fifo();
        // }
    }
}

#[inline(never)]
fn receive_resources(sio: &SIO) -> (&'static Kernel,
                                    &'static RaspberryPiPico,
                                    &'static RP2040Chip)
{
    let mut resources: [u32; 3] = [0; 3];
    for i in 0..3 {
        while !sio.fifo_ready() {  }
        resources[i] = sio.read_fifo();
    }

    unsafe {
        ((resources[0] as *const Kernel).as_ref::<'static>().unwrap(),
         (resources[1] as *const RaspberryPiPico).as_ref::<'static>().unwrap(),
         (resources[2] as *const RP2040Chip).as_ref::<'static>().unwrap())
    }
}

#[allow(dead_code)]
unsafe fn handle_interrupt() {
    let mut index: usize;
    asm!("mrs r0, ipsr",
         "and r0, #0xff",
         "sub r0, #16",
         "mov {0}, r0",
         out(reg) index);
}

/// "Handler" for interrupts that are not being explicitly handled by ASPK.
#[allow(dead_code)]
fn ignored_interrupt() {
    // This code will eventually notify so we are aware of ignored interrupts.
    // For now, just clear it and move on.
}

/// "Handler" for interrupts that should be handled;
/// ignoring them is dangerous.
#[allow(dead_code)]
unsafe fn fail_interrupt() {
    loop { cortexm0p::support::wfe(); }
}
