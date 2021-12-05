use kernel::static_init;
use kernel::Kernel;
use kernel::dsp::engine::DSPEngine;

use rp2040;
use rp2040::gpio::SIO;

use crate::{RP2040Chip, RaspberryPiPico};
use crate::aspk::interrupt;

/// Start ASPK.
///
/// Entry point for executing ASPK.
/// Receives resource information from core0 and begins executing the DSP loop.
#[no_mangle]
#[allow(unreachable_code)]
pub unsafe fn launch() -> ! {
    rp2040::init();
    interrupt::configure();

    let sio = SIO::new();

    // The first three words from the other side are the kernel, board, and chip resources.
    let (kernel, board_resources, chip_resources) = receive_resources(&sio);

    // ASPK runtime context
    let aspk = static_init!(DSPEngine, DSPEngine::new());

    kernel.dsp_loop(board_resources,
                    chip_resources,
                    aspk,
                    board_resources.dma);
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
