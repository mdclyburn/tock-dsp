use kernel::static_init;
use kernel::Kernel;
use kernel::dsp::engine::DSPEngine;
use kernel::dsp::link::{Chain, Link};

use rp2040;
use rp2040::gpio::SIO;

use crate::{RP2040Chip, RaspberryPiPico};
use crate::aspk::{effects, interrupt};

struct ASPKResources {
    engine: &'static DSPEngine,
    signal_chain: Chain,
}

macro_rules! create_link {
    ($effect_type:ty, $init:expr) => {{
        let effect = kernel::static_init!($effect_type, $init);
        kernel::static_init!(Link, Link::new(effect))
    }}
}

unsafe fn allocate_aspk_resources() -> ASPKResources {
    ASPKResources {
        engine: static_init!(DSPEngine, DSPEngine::new()),
        signal_chain: Chain::new(&[
            create_link!(effects::NoOpCopy, effects::NoOpCopy::new()),
            // create_link!(effects::Scale, effects::Scale::new(1, 4)),
        ]),
    }
}

/// Start ASPK.
///
/// Entry point for executing ASPK.
/// Receives resource information from core0 and begins executing the DSP loop.
#[no_mangle]
#[allow(unreachable_code)]
pub unsafe fn launch() -> ! {
    // core1 performs a harmless read during startup in the bootrom
    // (see pico-bootrom/bootrom/bootrom_rt0.S:338).
    // This causes the ROE flag to always go high if the FIFO was empty.
    // Clear this error here so we do not have an interrupt pending.
    asm!(
        "movs r1, 0b1000",
        "ldr r0, =0xd0000050", // 0xd0000050 = &SIO_FIFO_ST
        "str r1, [r0]",
        "pop {{r0, r1}}",
    );

    rp2040::init();

    // The first three words from the other side are the kernel, board, and chip resources.
    let sio = SIO::new();
    let (kernel, board_resources, chip_resources) = receive_resources(&sio);

    // Complete interrupt configuration.
    interrupt::configure(board_resources);

    let aspk = allocate_aspk_resources();

    board_resources.adc.configure_continuous_dma(
        rp2040::adc::Channel::Channel0,
        DSPEngine::sampling_rate() as u32);

    kernel.dsp_loop(board_resources,
                    chip_resources,
                    aspk.engine,
                    board_resources.dma,
                    board_resources.timer,
                    &aspk.signal_chain);
}

#[inline(never)]
fn receive_resources(sio: &SIO) -> (&'static Kernel,
                                    &'static RaspberryPiPico,
                                    &'static RP2040Chip)
{
    let mut resources: [u32; 3] = [0; 3];
    for i in 0..3 {
        while !sio.fifo_valid() {  }
        resources[i] = sio.read_fifo();
    }

    unsafe {
        ((resources[0] as *const Kernel).as_ref::<'static>().unwrap(),
         (resources[1] as *const RaspberryPiPico).as_ref::<'static>().unwrap(),
         (resources[2] as *const RP2040Chip).as_ref::<'static>().unwrap())
    }
}
