use core::slice;

use kernel::static_init;
use kernel::Kernel;
use kernel::dsp::engine::{self, DSPEngine, ProcessControl};
use kernel::errorcode::ErrorCode;
use kernel::hil;
use kernel::hil::time;
use kernel::sync::Mutex;

use dsp;
use dsp::signal::{Chain, Link};
use dsp::effects;
use rp2040;
use rp2040::gpio::SIO;
use rp2040::pio::{self, PIO, PIOBlock};

use crate::{RP2040Chip, RaspberryPiPico};
use crate::aspk::interrupt;
use crate::aspk::cnc::FIFOCommandReceiver;

/// DSP implemented with an ADC channel and a PIO block implementing I²S.
struct ADCToI2S {
    adc: &'static rp2040::adc::Adc,
    adc_dma: &'static dyn hil::dma::DMAChannel,
    i2s_pio: &'static rp2040::pio::PIOBlock,
    i2s_pio_dma: &'static dyn hil::dma::DMAChannel,
}

impl ADCToI2S {
    fn shift_scale16(usamples: &[u16], isamples: &mut [i16]) {
        // Scale (12- to 16-bit), translate the samples toward the baseline.
        for (usample, isample) in usamples.iter().zip(isamples.iter_mut()) {
            // Scale the sample up to a 16-bit value.
            // u12::MAX << 4 = 65,520 cannot exceed u16::MAX, so there's no worry about overflow.
            let usample16 = *usample << 4;
            *isample = if usample16 > i16::MAX as u16 {
                (usample16 - (i16::MAX as u16)) as i16
            } else {
                i16::MIN + (usample16 as i16)
            };
        }
    }

    fn shift_16(usamples: &[u16], isamples: &mut [i16]) {
        for (usample, isample) in usamples.iter().zip(isamples.iter_mut()) {
            *isample = -2048 + (*usample as i16);
        }
    }
}

impl ProcessControl for ADCToI2S {
    fn source_dma_channel(&self) -> &'static dyn hil::dma::DMAChannel {
        self.adc_dma
    }

    fn sink_dma_channel(&self) -> &'static dyn hil::dma::DMAChannel {
        self.i2s_pio_dma
    }

    fn start(&self) -> Result<(), ErrorCode> {
        Err(ErrorCode::NOSUPPORT)
    }

    fn stop(&self) -> Result<(), ErrorCode> {
        Err(ErrorCode::NOSUPPORT)
    }

    fn preprocess(&self, samples: &mut [usize]) {
        // Make it possible to see a signed representation of the buffers:
        // from [usize] to [i16] for processing and output.
        // Since we perform half-word transfers during the DMA, each usize is actually two samples.
        // The lower two bytes are the first sample, the upper two bytes are the second sample.
        //
        // This means that we end up double-aliasing the buffers here.
        // We continue to keep both representations around because
        // the links get the signed numbers, and the usize buffer is what we .put() back.
        let (usamples, isamples) = unsafe {
            (slice::from_raw_parts_mut(samples.as_mut_ptr() as *mut u16, dsp::config::buffer_len_samples()),
             slice::from_raw_parts_mut(samples.as_mut_ptr() as *mut i16, dsp::config::buffer_len_samples()))
        };

        Self::shift_scale16(usamples, isamples);
        // Self::shift_16(&*usamples, isamples);
    }
}

macro_rules! create_link {
    ($effect_type:ty, $init:expr) => {{
        let effect = kernel::static_init!($effect_type, $init);
        kernel::static_init!(Link, Link::new(effect))
    }}
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
        "push {{r0, r1}}",
        "movs r1, 0b1000",
        "ldr r0, =0xd0000050", // 0xd0000050 = &SIO_FIFO_ST
        "str r1, [r0]",
        "pop {{r0, r1}}",
    );

    // Start with clearing and disabling all interrupts.
    rp2040::init();

    // The first three words from the other side are the kernel, board, and chip resources.
    let sio = SIO::new();
    let (_kernel, board_resources, chip_resources) = receive_resources(&sio);

    // Create the command receiver built on the FIFO.
    let cmd_recv = FIFOCommandReceiver::new(board_resources.fifo);

    // Grab an alarm for timing things.
    let alarm = board_resources.timer.allocate_alarm().unwrap();

    // Complete interrupt configuration.
    interrupt::configure(board_resources, alarm);

    // Create statistics container.
    let mtx_stats = {
        use kernel::platform::KernelResources;
        use kernel::platform::sync::HardwareSyncAccess;

        let spinlock = board_resources.hardware_sync()
            .expect("no hardware sync configured")
            .access(true, |hs| hs.get_lock().expect("no free spinlocks left"))
            .expect("cannot access hardware sync");

        let stats = static_init!(engine::Statistics, engine::Statistics::default());

        Mutex::new(spinlock, stats)
    };
    board_resources.dsp.add_stats(mtx_stats.clone());

    // DSP engine
    let engine = static_init!(
        DSPEngine<time::Freq1MHz, time::Ticks32>,
        DSPEngine::new(cmd_recv, mtx_stats, board_resources.timer));

    // Signal chain
    let signal_chain = Chain::new(&[
        // create_link!(effects::special::Zero, effects::special::Zero::new()),
        create_link!(effects::special::NoOp, effects::special::NoOp::new()),
        // create_link!(effects::delay::Flange, effects::delay::Flange::new(10_000, 5000)),
    ]);

    // Set up DSP source and sink backing producer and consumers.
    // - ADC set for continuous sampling.
    // - I²S implemented on top of PIO.
    board_resources.adc.configure_continuous_dma(
        rp2040::adc::Channel::Channel0,
        dsp::config::sampling_rate() as u32);
    let i2s_pio = configure_pio_pcm177x(board_resources.pio);

    let dsp_process_ctrl = {
        let source_channel = board_resources.dma.configure(&hil::dma::Parameters {
            kind: hil::dma::TransferKind::PeripheralToMemory(hil::dma::SourcePeripheral::ADC, 0),
            transfer_count: dsp::config::buffer_len_samples(),
            transfer_size: hil::dma::TransferSize::HalfWord,
            increment_on_read: false,
            increment_on_write: true,
            high_priority: true,
        }).unwrap();
        let sink_channel = board_resources.dma.configure(&hil::dma::Parameters {
            kind: hil::dma::TransferKind::MemoryToPeripheral(0, hil::dma::TargetPeripheral::Custom(0)),
            transfer_count: dsp::config::buffer_len_samples(),
            transfer_size: hil::dma::TransferSize::HalfWord,
            increment_on_read: true,
            increment_on_write: false,
            high_priority: true,
        }).unwrap();

        ADCToI2S {
            adc: board_resources.adc,
            adc_dma: source_channel,
            i2s_pio: i2s_pio,
            i2s_pio_dma: sink_channel,
        }
    };

    engine.run(
        chip_resources,
        &signal_chain,
        &dsp_process_ctrl);
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
fn configure_pio_uda1334(pio: &'static PIO) -> &'static PIOBlock {
    let i2s_pio = pio_proc::pio!(32, "
    ;; bit 0 = BCLK, bit 1 = LRCLK
    .side_set 2

    .wrap_target
left_ch:
    set pins, 0 side 0b00                         ; Output known low value. Clock low for empty bit.
    set x, 15 side 0b01                           ; Set up counter. Cue on BCLK.
left_ch_loop:
    out pins, 1 side 0b00                         ; Write the left channel bit.
    jmp x-- left_ch_loop side 0b01                ; Repeat to output 16 bits. Cue on BCLK.

right_ch:
    set pins, 0 side 0b10                         ; Output known low value. Clock low for empty bit.
    set x, 15 side 0b11                           ; Set up counter. Cue on BCLK.
right_ch_loop:
    set pins, 0 side 0b10                         ; Write empty right channel.
    jmp x--, right_ch_loop side 0b11              ; Repeat to output 16 bits. Cue on BCLK.
    .wrap
");

    // Calculate clock divider.
    //
    // ===== EXAMPLE
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
    let (div_int, div_frac) = {
        let req_bandwidth =
            // (don't-care bit + N bits per channel) * no. channels * cycles per sample
            ((1 + 16) * 2) * 2
            * dsp::config::sampling_rate();
        let clk_ticks_per = 125_000_000f32 / req_bandwidth as f32;

        let frac = (clk_ticks_per - (clk_ticks_per as u32 as f32)) * 256f32;
        let frac = if frac - (frac as u32 as f32) >= 0.5 {
            frac + 1.0
        } else {
            frac
        } as u8;

        (clk_ticks_per as u16, frac)
    };
    let parameters = {
        let default = pio::Parameters::default();
        pio::Parameters {
            clock_divider: (div_int, div_frac),
            osr_direction: pio::ShiftDirection::Left,
            set_count: 1,
            out_count: 1,
            out_sticky: true,
            wrap_top: i2s_pio.program.wrap.source,
            wrap_bottom: i2s_pio.program.wrap.target,
            autopull: pio::Autoshift::On(16),
            out_base_pin: 15,
            set_base_pin: 15,
            side_set_base_pin: 16,
            side_set_count: 2,
            initial_pindir: 0b00001,
            fifo_allocation: pio::FIFOAllocation::Transmit,
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

#[inline(never)]
fn configure_pio_pcm177x(pio: &'static PIO) -> &'static PIOBlock {
    let i2s_pio = pio_proc::pio!(32,"
    .side_set 2

;;; I²S hardware logic (instructions 0 to 7)
left_ch:
    set pins, 0 side 0b00                         ; Output known low value. Clock low for empty bit.
    set x, 15 side 0b01                           ; Set up counter. Cue on BCLK.
left_ch_loop:
    out pins, 1 side 0b00                         ; Write the left channel bit.
    jmp x-- left_ch_loop side 0b01                ; Repeat to output 16 bits. Cue on BCLK.

right_ch:
    set pins, 0 side 0b10                         ; Output known low value. Clock low for empty bit.
    set x, 15 side 0b11                           ; Set up counter. Cue on BCLK.
right_ch_loop:
    set pins, 1 side 0b10                         ; Write empty right channel.
    jmp x--, right_ch_loop side 0b11              ; Repeat to output 16 bits. Cue on BCLK.

;;; PCM177x SCKI (instructions 8 to 9)
;;;
;;; Configuration:
;;; Assign a set pin for the clock output signal.
scki_loop:
    set pins, 0 side 0b00                        ; Output clock high signal.
    set pins, 1 side 0b00                        ; Output clock low signal.
");

    // Clock divider values.
    // See `configure_pio_uda1334()` for the explanation.
    let (bclk_div_int, bclk_div_frac, scki_div_int, scki_div_frac) = {
        // Assuming a 125MHz source clock.
        let clock_freq = 125_000_000f32;
        let req_bandwidth =
            // (don't-care bit + N bits per channel) * no. channels * cycles per sample
            ((1 + 16) * 2) * 2
            * dsp::config::sampling_rate();
        // Calculate a more precise, floating-point divider value.
        let clk_ticks_per = clock_freq / req_bandwidth as f32;

        // Get the decimal part of the FP divider value.
        // Using the corresponding f32 function is not possible, for some reason.
        let frac = (clk_ticks_per - (clk_ticks_per as u32 as f32)) * 256f32;
        // ...and round it.
        let frac = if frac - (frac as u32 as f32) >= 0.5 {
            frac + 1.0
        } else {
            frac
        } as u8;

        let bclk_div_int = clk_ticks_per as u16;
        let bclk_div_frac = frac;

        // Calculate the SCKI from the BCLK FP divider value.
        // We choose an SCKI of x128 faster than the sampling frequency.
        let scki_div = clock_freq
            // sampling freq. * 2 cycles per rising edge * rate factor
            / ((dsp::config::sampling_rate() * 2 * 128) as f32);

        let scki_div_int = scki_div as u16;

        let scki_div_frac = (scki_div - (scki_div as u32 as f32)) * 256f32;
        let scki_div_frac = if scki_div - (scki_div as u32 as f32) >= 0.5 {
            scki_div_frac + 1.0
        } else {
            scki_div_frac
        } as u8;

        kernel::debug!("BCLK divider: {}, {}", bclk_div_int, bclk_div_frac);
        kernel::debug!("SCKI divider: {}, {}", scki_div_int, scki_div_frac);

        (bclk_div_int, bclk_div_frac, scki_div_int, scki_div_frac)
    };

    let (sm0_params, sm1_params) = {
        let default = pio::Parameters::default();

        (
            // State machine 0: I²S logic
            pio::Parameters {
                clock_divider: (bclk_div_int, bclk_div_frac),
                osr_direction: pio::ShiftDirection::Left,
                set_count: 1,
                out_count: 1,
                out_sticky: true,
                wrap_top: 7,
                wrap_bottom: 0,
                autopull: pio::Autoshift::On(16),
                out_base_pin: 15,
                set_base_pin: 15,
                side_set_base_pin: 16,
                side_set_count: 2,
                initial_pindir: 0b00001,
                fifo_allocation: pio::FIFOAllocation::Transmit,
                ..default
            },

            // State machine 1: SCKI
            pio::Parameters {
                clock_divider: (scki_div_int, scki_div_frac),
                set_count: 1,
                out_sticky: true,
                wrap_top: 9,
                wrap_bottom: 8,
                set_base_pin: 14,
                initial_pindir: 0b1,
                ..default
            },
        )
    };

    pio.configure(
        &i2s_pio.program.code,
        &[Some(&sm0_params), Some(&sm1_params), None, None],
        &[],
        pio::InterruptLine::IRQ0)
        .unwrap()
}
