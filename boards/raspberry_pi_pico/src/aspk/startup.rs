use kernel::static_init;
use kernel::Kernel;
use kernel::dsp::engine::DSPEngine;
use kernel::dsp::link::{Chain, Link};

use dsp::effects;
use rp2040;
use rp2040::gpio::SIO;
use rp2040::pio::{self, PIO, PIOBlock};

use crate::{RP2040Chip, RaspberryPiPico};
use crate::aspk::interrupt;

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
            create_link!(effects::NoOp, effects::NoOp::new()),
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

    let i2s = configure_pio(board_resources.pio);

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

#[inline(never)]
fn configure_pio(pio: &'static PIO) -> &'static PIOBlock {
    let i2s_pio = pio_proc::pio!(
        32,
        "
set pindirs, 1
.wrap_target
set pins, 0 [1]
set pins, 1 [1]
.wrap
");
    let parameters = {
    let default = pio::Parameters::default();
        pio::Parameters {
            clock_divider: (3000, 0),
            osr_direction: pio::ShiftDirection::Left,
            set_count: 1,
            out_count: 1,
            out_sticky: true,
            wrap_top: i2s_pio.program.wrap.source,
            wrap_bottom: i2s_pio.program.wrap.target,
            autopull: pio::Autoshift::On(16),
            set_base_pin: 16,
            ..default
        }
    };

    pio.configure(
        &i2s_pio.program.code,
        &[Some(&parameters), None, None, None],
        &[pio::Interrupt::ReceiveNotEmpty(pio::StateMachine::SM0)],
        pio::InterruptLine::IRQ0)
        .unwrap()
}
