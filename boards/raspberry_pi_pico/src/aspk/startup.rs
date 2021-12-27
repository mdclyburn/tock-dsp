use kernel::static_init;
use kernel::Kernel;
use kernel::dsp::engine::{self, DSPEngine, Resources};
use kernel::dsp::link::{Chain, Link};
use kernel::hil::dma::{SourcePeripheral, TargetPeripheral};
use kernel::platform::sync::UnmanagedSpinlock;
use kernel::sync::{Lockable, Mutex};

use dsp::effects;
use rp2040;
use rp2040::gpio::SIO;
use rp2040::pio::{self, PIO, PIOBlock};

use crate::{RP2040Chip, RaspberryPiPico};
use crate::aspk::interrupt;

struct ASPKResources<L: 'static + Lockable> {
    engine: &'static DSPEngine<L>,
    signal_chain: Chain,
}

macro_rules! create_link {
    ($effect_type:ty, $init:expr) => {{
        let effect = kernel::static_init!($effect_type, $init);
        kernel::static_init!(Link, Link::new(effect))
    }}
}

unsafe fn allocate_aspk_resources(board_resources: &RaspberryPiPico) -> ASPKResources<UnmanagedSpinlock> {
    // Create statistics container.
    let mtx_stats = {
        use kernel::platform::KernelResources;
        use kernel::platform::sync::HardwareSyncAccess;

        let spinlock: UnmanagedSpinlock = board_resources.hardware_sync()
            .expect("no hardware sync configured")
            .access(true, |hs| hs.get_spinlock().expect("no free spinlocks left"))
            .expect("cannot access hardware sync")
            .into();

        let stats = static_init!(engine::Statistics, engine::Statistics::default());

        Mutex::new(spinlock, stats)
    };

    ASPKResources {
        engine: static_init!(DSPEngine<UnmanagedSpinlock>, DSPEngine::new(mtx_stats)),
        signal_chain: Chain::new(&[
            create_link!(effects::NoOp, effects::NoOp::new()),
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

    let aspk = allocate_aspk_resources(&board_resources);

    let _i2s = configure_pio(board_resources.pio);

    board_resources.adc.configure_continuous_dma(
        rp2040::adc::Channel::Channel0,
        engine::sampling_rate() as u32);

    aspk.engine.run(
        &engine::Resources {
            chip: chip_resources,
            dma: board_resources.dma,
            time: board_resources.timer.allocate_alarm().unwrap(),
        },
        &aspk.signal_chain,
        SourcePeripheral::ADC,
        TargetPeripheral::Custom(0));
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
    let i2s_pio = pio_proc::pio!(32, "
    ;; bit 0 = BCLK, bit 1 = LRCLK
    .side_set 2

    .wrap_target
left_ch:
    set x, 15 side 0b01                           ; Set up counter. Cue on BCLK (LSB of right channel).
left_ch_loop:
    out pins, 1 side 0b00                         ; Write left channel bit.
    jmp x-- left_ch_loop side 0b01                ; Repeat for the first 15 bits. Cue on BCLK.
    out pins, 1 side 0b10                         ; Write last bit. Goes with the right channel's LRCLK.

right_ch:
    set x, 15 side 0b11                           ; Set up counter. Cue on BCLK (LSB of left channel).
right_ch_loop:
    set pins, 0 side 0b10                         ; Write empty right channel.
    jmp x--, right_ch_loop side 0b11              ; Repeat for the first 15 bits. Cue on BCLK.
    set pins, 0 side 0b00                         ; Write last bit. Goes with the left channel's LRCLK.
    .wrap
");

    let parameters = {
    let default = pio::Parameters::default();
        pio::Parameters {
            // Desired frequency: 88.2kHz = 88,200Hz
            //
            // Calculating clock divider as follows...
            // 32 bits to output a sample (16-bit samples)
            //   * 2 cycles per sample (see the PIOASM)
            //   * 88,200 samples to output per second
            //   = 5,644,800Hz clock rate
            //
            // 125,000,000 system clock ticks (125MHz system clock)
            //   / 5,644,800 PIO cycles necessary for operation
            //   = 22.144274376 system clock ticks per PIO cycle
            //
            // Integer divider = 22
            // Fractional divider (8-bit) = .144274376 * 256
            //   = 36.934240256 ≅ 37
            clock_divider: (22, 37),
            osr_direction: pio::ShiftDirection::Left,
            set_count: 1,
            out_count: 1,
            out_sticky: true,
            wrap_top: i2s_pio.program.wrap.source,
            wrap_bottom: i2s_pio.program.wrap.target,
            autopull: pio::Autoshift::On(16),
            out_base_pin: 16,
            set_base_pin: 16,
            side_set_base_pin: 17,
            side_set_count: 2,
            initial_pindir: 0b00001,
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
