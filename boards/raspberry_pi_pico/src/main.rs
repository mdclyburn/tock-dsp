//! Tock kernel for the Raspberry Pi Pico.
//!
//! It is based on RP2040SoC SoC (Cortex M0+).

#![no_std]
// Disable this attribute when documenting, as a workaround for
// https://github.com/rust-lang/rust/issues/62184.
#![cfg_attr(not(doc), no_main)]
#![deny(missing_docs)]
#![feature(asm, naked_functions)]

use capsules::virtual_alarm::VirtualMuxAlarm;
use components::gpio::GpioComponent;
use components::led::LedsComponent;
use enum_primitive::cast::FromPrimitive;
use kernel::component::Component;
use kernel::debug;
use kernel::dynamic_deferred_call::{DynamicDeferredCall, DynamicDeferredCallClientState};
use kernel::hil::led::LedHigh;
use kernel::platform::{KernelResources, SyscallDriverLookup};
use kernel::scheduler::round_robin::RoundRobinSched;
use kernel::syscall::SyscallDriver;
use kernel::{capabilities, create_capability, static_init, Kernel};

use rp2040;
use rp2040::adc::{Adc, Channel};
use rp2040::chip::{Rp2040, Rp2040DefaultPeripherals};
use rp2040::clocks::{
    AdcAuxiliaryClockSource, PeripheralAuxiliaryClockSource, PllClock,
    ReferenceAuxiliaryClockSource, ReferenceClockSource, RtcAuxiliaryClockSource,
    SystemAuxiliaryClockSource, SystemClockSource, UsbAuxiliaryClockSource,
};
use rp2040::gpio::{GpioFunction, RPGpio, RPGpioPin, SIO};
use rp2040::multicore;
use rp2040::psm::PowerOnStateMachine;
use rp2040::resets::Peripheral;
use rp2040::sysinfo;
use rp2040::timer::RPTimer;

mod aspk;
mod io;
mod flash_bootloader;
mod sync;

/// Allocate memory for the stack
#[no_mangle]
#[link_section = ".stack_buffer"]
pub static mut STACK_MEMORY: [u8; 0x1000] = [0; 0x1000];

// Manually setting the boot header section that contains the FCB header
#[used]
#[link_section = ".flash_bootloader"]
static FLASH_BOOTLOADER: [u8; 256] = flash_bootloader::FLASH_BOOTLOADER;

// State for loading and holding applications.
// How should the kernel respond when a process faults.
const FAULT_RESPONSE: kernel::process::PanicFaultPolicy = kernel::process::PanicFaultPolicy {};

// Number of concurrent processes this platform supports.
const NUM_PROCS: usize = 4;
const NUM_UPCALLS_IPC: usize = NUM_PROCS + 1;

static mut PROCESSES: [Option<&'static dyn kernel::process::Process>; NUM_PROCS] =
    [None; NUM_PROCS];

static mut CHIP: Option<&'static Rp2040<Rp2040DefaultPeripherals>> = None;

/// Supported drivers by the platform
pub struct RaspberryPiPico {
    ipc: kernel::ipc::IPC<NUM_PROCS, NUM_UPCALLS_IPC>,
    console: &'static capsules::console::Console<'static>,
    alarm: &'static capsules::alarm::AlarmDriver<
        'static,
        VirtualMuxAlarm<'static, rp2040::timer::RPTimer<'static>>,
    >,
    gpio: &'static capsules::gpio::GPIO<'static, RPGpioPin<'static>>,
    hw_sync_access: &'static sync::HardwareSyncBlockAccess,
    led: &'static capsules::led::LedDriver<'static, LedHigh<'static, RPGpioPin<'static>>>,
    adc: &'static capsules::adc::AdcVirtualized<'static>,
    temperature: &'static capsules::temperature::TemperatureSensor<'static>,

    scheduler: &'static RoundRobinSched<'static>,
    systick: cortexm0p::systick::SysTick,
}

impl SyscallDriverLookup for RaspberryPiPico {
    fn with_driver<F, R>(&self, driver_num: usize, f: F) -> R
    where
        F: FnOnce(Option<&dyn SyscallDriver>) -> R,
    {
        match driver_num {
            capsules::console::DRIVER_NUM => f(Some(self.console)),
            capsules::alarm::DRIVER_NUM => f(Some(self.alarm)),
            capsules::gpio::DRIVER_NUM => f(Some(self.gpio)),
            capsules::led::DRIVER_NUM => f(Some(self.led)),
            kernel::ipc::DRIVER_NUM => f(Some(&self.ipc)),
            capsules::adc::DRIVER_NUM => f(Some(self.adc)),
            capsules::temperature::DRIVER_NUM => f(Some(self.temperature)),
            _ => f(None),
        }
    }
}

/// Abbreviated type for the RP2040 chip.
pub type RP2040Chip = Rp2040<'static, Rp2040DefaultPeripherals<'static>>;

impl KernelResources<RP2040Chip> for RaspberryPiPico {
    type SyscallDriverLookup = Self;
    type SyscallFilter = ();
    type ProcessFault = ();
    type Scheduler = RoundRobinSched<'static>;
    type SchedulerTimer = cortexm0p::systick::SysTick;
    type WatchDog = ();
    type HardwareSyncAccess = sync::HardwareSyncBlockAccess;

    fn syscall_driver_lookup(&self) -> &Self::SyscallDriverLookup {
        &self
    }
    fn syscall_filter(&self) -> &Self::SyscallFilter {
        &()
    }
    fn process_fault(&self) -> &Self::ProcessFault {
        &()
    }
    fn scheduler(&self) -> &Self::Scheduler {
        self.scheduler
    }
    fn scheduler_timer(&self) -> &Self::SchedulerTimer {
        &self.systick
    }
    fn watchdog(&self) -> &Self::WatchDog {
        &()
    }

    fn hardware_sync(&self) -> Option<&Self::HardwareSyncAccess> {
        Some(self.hw_sync_access)
    }
}

/// Entry point used for debuger
///
/// When loaded using gdb, the Raspberry Pi Pico is not reset
/// by default. Without this function, gdb sets the PC to the
/// beginning of the flash. This is not correct, as the RP2040
/// has a more complex boot process.
///
/// This function is set to be the entry point for gdb and is used
/// to send the RP2040 back in the bootloader so that all the boot
/// sqeuence is performed.
#[no_mangle]
#[naked]
pub unsafe extern "C" fn jump_to_bootloader() {
    asm!(
        "
    movs r0, #0
    ldr r1, =(0xe0000000 + 0x0000ed08)

    ldmia r0!, {{r1, r2}}
    msr msp, r1
    bx r2
    ",
        options(noreturn)
    );
}

fn init_clocks(peripherals: &Rp2040DefaultPeripherals) {
    // Start tick in watchdog
    peripherals.watchdog.start_tick(12);

    // Disable the Resus clock
    peripherals.clocks.disable_resus();

    // Setup the external Osciallator
    peripherals.xosc.init();

    // disable ref and sys clock aux sources
    peripherals.clocks.disable_sys_aux();
    peripherals.clocks.disable_ref_aux();

    peripherals
        .resets
        .reset(&[Peripheral::PllSys, Peripheral::PllUsb]);
    peripherals
        .resets
        .unreset(&[Peripheral::PllSys, Peripheral::PllUsb], true);

    // Configure PLLs (from Pico SDK)
    //                   REF     FBDIV VCO            POSTDIV
    // PLL SYS: 12 / 1 = 12MHz * 125 = 1500MHZ / 6 / 2 = 125MHz
    // PLL USB: 12 / 1 = 12MHz * 40  = 480 MHz / 5 / 2 =  48MHz

    // It seems that the external osciallator is clocked at 12 MHz

    peripherals
        .clocks
        .pll_init(PllClock::Sys, 12, 1, 1500 * 1000000, 6, 2);
    peripherals
        .clocks
        .pll_init(PllClock::Usb, 12, 1, 480 * 1000000, 5, 2);

    // pico-sdk: // CLK_REF = XOSC (12MHz) / 1 = 12MHz
    peripherals.clocks.configure_reference(
        ReferenceClockSource::Xosc,
        ReferenceAuxiliaryClockSource::PllUsb,
        12000000,
        12000000,
    );
    // pico-sdk: CLK SYS = PLL SYS (125MHz) / 1 = 125MHz
    peripherals.clocks.configure_system(
        SystemClockSource::Auxiliary,
        SystemAuxiliaryClockSource::PllSys,
        125000000,
        125000000,
    );
    // pico-sdk: CLK USB = PLL USB (48MHz) / 1 = 48MHz
    peripherals
        .clocks
        .configure_usb(UsbAuxiliaryClockSource::PllSys, 48000000, 48000000);
    // pico-sdk: CLK ADC = PLL USB (48MHZ) / 1 = 48MHz
    peripherals
        .clocks
        .configure_adc(AdcAuxiliaryClockSource::PllUsb, 48000000, 48000000);
    // pico-sdk: CLK RTC = PLL USB (48MHz) / 1024 = 46875Hz
    peripherals
        .clocks
        .configure_rtc(RtcAuxiliaryClockSource::PllSys, 48000000, 46875);
    // pico-sdk:
    // CLK PERI = clk_sys. Used as reference clock for Peripherals. No dividers so just select and enable
    // Normally choose clk_sys or clk_usb
    peripherals
        .clocks
        .configure_peripheral(PeripheralAuxiliaryClockSource::System, 125000000);
}

/// This is in a separate, inline(never) function so that its stack frame is
/// removed when this function returns. Otherwise, the stack space used for
/// these static_inits is wasted.
#[inline(never)]
unsafe fn get_peripherals() -> &'static mut Rp2040DefaultPeripherals<'static> {
    static_init!(Rp2040DefaultPeripherals, Rp2040DefaultPeripherals::new())
}

#[inline(never)]
unsafe fn initialize_multicore(kernel: &Kernel,
                               resources: &RaspberryPiPico,
                               chip: &RP2040Chip,
                               psm: &PowerOnStateMachine,
                               sio: &SIO)
{
    debug!("Preparing synchronization structures...");

    // debug!("Initializing peripherals for handoff to core1.");

    // UART for core1: 8 = UART1 TX, 9 = UART1 RX.
    // peripherals.gpio.get_pin(RPGpio::GPIO0).set_function(GpioFunction::UART);
    // peripherals.gpio.get_pin(RPGpio::GPIO9).set_function(GpioFunction::UART);

    // We actively use the SIO FIFO during launch_core1, so we wait until after
    // launch to take interrupts for FIFO content from core1.
    sio.disable_interrupt();

    let (core1_vectors, core1_sp, core1_entry) =
        (aspk::CORE1_VECTORS.as_ptr() as usize,
         (&aspk::_core1_estack as *const u8) as usize,
         (aspk::aspk_main as *const fn()) as usize);
    // May be useful to know later.
    aspk::CORE1_VECTORS[0] = core1_sp;
    aspk::CORE1_VECTORS[1] = core1_entry;

    debug!("Launching core1. VTOR: {:#X}, SP: {:#X}, IP: {:#X}",
           core1_vectors, core1_sp, core1_entry);
    multicore::launch_core1(&psm, &sio, core1_vectors, core1_sp, core1_entry);

    // Send over the locations for the kernel and chip resources.
    sio.write_fifo(kernel as *const _ as usize as u32);
    sio.write_fifo(resources as *const _ as usize as u32);
    sio.write_fifo(chip as *const _ as usize as u32);

    // Now we're ready to take interrupts from SIO.
    // sio.enable_interrupt();

    debug!("Launched core1!");
}

/// Main function called after RAM initialized.
#[no_mangle]
pub unsafe fn main() {
    // Loads relocations and clears BSS
    rp2040::init();

    let peripherals = get_peripherals();

    // Set the UART used for panic
    io::WRITER.set_uart(&peripherals.uart0);

    // Reset all peripherals except QSPI (we might be booting from Flash), PLL USB and PLL SYS
    peripherals.resets.reset_all_except(&[
        Peripheral::IOQSpi,
        Peripheral::PadsQSpi,
        Peripheral::PllUsb,
        Peripheral::PllSys,
    ]);

    // Unreset all the peripherals that do not require clock setup as they run using the sys_clk or ref_clk
    // Wait for the peripherals to reset
    peripherals.resets.unreset_all_except(
        &[
            Peripheral::Adc,
            Peripheral::Rtc,
            Peripheral::Spi0,
            Peripheral::Spi1,
            Peripheral::Uart0,
            Peripheral::Uart1,
            Peripheral::UsbCtrl,
        ],
        true,
    );

    init_clocks(&peripherals);

    // Unreset all peripherals
    peripherals.resets.unreset_all_except(&[], true);

    //set RX and TX pins in UART mode
    let gpio_tx = peripherals.pins.get_pin(RPGpio::GPIO0);
    let gpio_rx = peripherals.pins.get_pin(RPGpio::GPIO1);
    gpio_rx.set_function(GpioFunction::UART);
    gpio_tx.set_function(GpioFunction::UART);
    // Disable IE for pads 26-29 (the Pico SDK runtime does this, not sure why)
    for pin in 26..30 {
        peripherals
            .pins
            .get_pin(RPGpio::from_usize(pin).unwrap())
            .deactivate_pads();
    }

    let chip = static_init!(
        Rp2040<Rp2040DefaultPeripherals>,
        Rp2040::new(peripherals, &peripherals.sio)
    );

    CHIP = Some(chip);

    let board_kernel = static_init!(Kernel, Kernel::new(&PROCESSES));

    // Single-cycle IO spinlocks
    let hw_sync_access = static_init!(sync::HardwareSyncBlockAccess,
                                      sync::HardwareSyncBlockAccess::new());

    let process_management_capability =
        create_capability!(capabilities::ProcessManagementCapability);
    let main_loop_capability = create_capability!(capabilities::MainLoopCapability);
    let memory_allocation_capability = create_capability!(capabilities::MemoryAllocationCapability);

    let dynamic_deferred_call_clients =
        static_init!([DynamicDeferredCallClientState; 2], Default::default());
    let dynamic_deferred_caller = static_init!(
        DynamicDeferredCall,
        DynamicDeferredCall::new(dynamic_deferred_call_clients)
    );
    DynamicDeferredCall::set_global_instance(dynamic_deferred_caller);

    let mux_alarm = components::alarm::AlarmMuxComponent::new(&peripherals.timer)
        .finalize(components::alarm_mux_component_helper!(RPTimer));

    let alarm = components::alarm::AlarmDriverComponent::new(
        board_kernel,
        capsules::alarm::DRIVER_NUM,
        mux_alarm,
    )
    .finalize(components::alarm_component_helper!(RPTimer));

    // UART
    // Create a shared UART channel for kernel debug.
    let uart_mux = components::console::UartMuxComponent::new(
        &peripherals.uart0,
        115200,
        dynamic_deferred_caller,
    )
    .finalize(());

    // Setup the console.
    let console = components::console::ConsoleComponent::new(
        board_kernel,
        capsules::console::DRIVER_NUM,
        uart_mux,
    )
        .finalize(());

    // Create the debugger object that handles calls to `debug!()`.
    let debug_sl: kernel::platform::sync::ManagedSpinlock =
        hw_sync_access.access(true, |hsb| hsb.get_spinlock()).unwrap().unwrap().into();
    let mtx_debug_write = kernel::sync::Mutex::new(debug_sl, ());
    components::debug_writer::DebugWriterComponent::new(uart_mux, Some(mtx_debug_write)).finalize(());

    let gpio = GpioComponent::new(
        board_kernel,
        capsules::gpio::DRIVER_NUM,
        components::gpio_component_helper!(
            RPGpioPin,
            // Used for serial communication. Comment them in if you don't use serial.
            // 0 => &peripherals.pins.get_pin(RPGpio::GPIO0),
            // 1 => &peripherals.pins.get_pin(RPGpio::GPIO1),
            // 2 => &peripherals.pins.get_pin(RPGpio::GPIO2),
            // 3 => &peripherals.pins.get_pin(RPGpio::GPIO3),
            4 => &peripherals.pins.get_pin(RPGpio::GPIO4),
            5 => &peripherals.pins.get_pin(RPGpio::GPIO5),
            6 => &peripherals.pins.get_pin(RPGpio::GPIO6),
            7 => &peripherals.pins.get_pin(RPGpio::GPIO7),

            // UART1
            // 8 => &peripherals.pins.get_pin(RPGpio::GPIO8),
            // 9 => &peripherals.pins.get_pin(RPGpio::GPIO9),

            10 => &peripherals.pins.get_pin(RPGpio::GPIO10),
            11 => &peripherals.pins.get_pin(RPGpio::GPIO11),
            12 => &peripherals.pins.get_pin(RPGpio::GPIO12),
            13 => &peripherals.pins.get_pin(RPGpio::GPIO13),
            14 => &peripherals.pins.get_pin(RPGpio::GPIO14),
            15 => &peripherals.pins.get_pin(RPGpio::GPIO15),
            16 => &peripherals.pins.get_pin(RPGpio::GPIO16),
            17 => &peripherals.pins.get_pin(RPGpio::GPIO17),
            18 => &peripherals.pins.get_pin(RPGpio::GPIO18),
            19 => &peripherals.pins.get_pin(RPGpio::GPIO19),
            20 => &peripherals.pins.get_pin(RPGpio::GPIO20),
            21 => &peripherals.pins.get_pin(RPGpio::GPIO21),
            22 => &peripherals.pins.get_pin(RPGpio::GPIO22),
            23 => &peripherals.pins.get_pin(RPGpio::GPIO23),
            24 => &peripherals.pins.get_pin(RPGpio::GPIO24),
            // LED pin
            // 25 => &peripherals.pins.get_pin(RPGpio::GPIO25),

            // Uncomment to use these as GPIO pins instead of ADC pins
            // 26 => &peripherals.pins.get_pin(RPGpio::GPIO26),
            // 27 => &peripherals.pins.get_pin(RPGpio::GPIO27),
            // 28 => &peripherals.pins.get_pin(RPGpio::GPIO28),
            // 29 => &peripherals.pins.get_pin(RPGpio::GPIO29)
        ),
    )
    .finalize(components::gpio_component_buf!(RPGpioPin<'static>));

    let led = LedsComponent::new(components::led_component_helper!(
        LedHigh<'static, RPGpioPin<'static>>,
        LedHigh::new(&peripherals.pins.get_pin(RPGpio::GPIO25))
    ))
    .finalize(components::led_component_buf!(
        LedHigh<'static, RPGpioPin<'static>>
    ));

    peripherals.adc.init();

    let adc_mux = components::adc::AdcMuxComponent::new(&peripherals.adc)
        .finalize(components::adc_mux_component_helper!(Adc));

    let temp_sensor = components::temperature_rp2040::TemperatureRp2040Component::new(1.721, 0.706)
        .finalize(components::temperaturerp2040_adc_component_helper!(
            rp2040::adc::Adc,
            Channel::Channel4,
            adc_mux
        ));

    let grant_cap = create_capability!(capabilities::MemoryAllocationCapability);
    let grant_temperature =
        board_kernel.create_grant(capsules::temperature::DRIVER_NUM, &grant_cap);

    let temp = static_init!(
        capsules::temperature::TemperatureSensor<'static>,
        capsules::temperature::TemperatureSensor::new(temp_sensor, grant_temperature)
    );
    kernel::hil::sensors::TemperatureDriver::set_client(temp_sensor, temp);

    let adc_channel_0 = components::adc::AdcComponent::new(&adc_mux, Channel::Channel0)
        .finalize(components::adc_component_helper!(Adc));

    let adc_channel_1 = components::adc::AdcComponent::new(&adc_mux, Channel::Channel1)
        .finalize(components::adc_component_helper!(Adc));

    let adc_channel_2 = components::adc::AdcComponent::new(&adc_mux, Channel::Channel2)
        .finalize(components::adc_component_helper!(Adc));

    let adc_channel_3 = components::adc::AdcComponent::new(&adc_mux, Channel::Channel3)
        .finalize(components::adc_component_helper!(Adc));

    let adc_syscall =
        components::adc::AdcVirtualComponent::new(board_kernel, capsules::adc::DRIVER_NUM)
            .finalize(components::adc_syscall_component_helper!(
                adc_channel_0,
                adc_channel_1,
                adc_channel_2,
                adc_channel_3,
            ));
    // PROCESS CONSOLE
    let process_console =
        components::process_console::ProcessConsoleComponent::new(board_kernel, uart_mux)
            .finalize(());
    let _ = process_console.start();

    let scheduler = components::sched::round_robin::RoundRobinComponent::new(&PROCESSES)
        .finalize(components::rr_component_helper!(NUM_PROCS));

    let raspberry_pi_pico = RaspberryPiPico {
        ipc: kernel::ipc::IPC::new(
            board_kernel,
            kernel::ipc::DRIVER_NUM,
            &memory_allocation_capability,
        ),
        alarm,
        gpio,
        hw_sync_access,
        led,
        console,
        adc: adc_syscall,
        temperature: temp,

        scheduler,
        systick: cortexm0p::systick::SysTick::new_with_calibration(125_000_000),
    };

    let platform_type = match peripherals.sysinfo.get_platform() {
        sysinfo::Platform::Asic => "ASIC",
        sysinfo::Platform::Fpga => "FPGA",
    };

    debug!(
        "RP2040 Revision {} {}",
        peripherals.sysinfo.get_revision(),
        platform_type
    );

    /// These symbols are defined in the linker script.
    extern "C" {
        /// Beginning of the ROM region containing app images.
        static _sapps: u8;
        /// End of the ROM region containing app images.
        static _eapps: u8;
        /// Beginning of the RAM region for app memory.
        static mut _sappmem: u8;
        /// End of the RAM region for app memory.
        static _eappmem: u8;
    }

    kernel::process::load_processes(
        board_kernel,
        chip,
        core::slice::from_raw_parts(
            &_sapps as *const u8,
            &_eapps as *const u8 as usize - &_sapps as *const u8 as usize,
        ),
        core::slice::from_raw_parts_mut(
            &mut _sappmem as *mut u8,
            &_eappmem as *const u8 as usize - &_sappmem as *const u8 as usize,
        ),
        &mut PROCESSES,
        &FAULT_RESPONSE,
        &process_management_capability,
    )
    .unwrap_or_else(|err| {
        debug!("Error loading processes!");
        debug!("{:?}", err);
    });

    initialize_multicore(&board_kernel, &raspberry_pi_pico, chip, &peripherals.psm, &peripherals.sio);

    let sio = SIO::new();
    while !sio.fifo_valid() {  }
    let first_word = sio.read_fifo();
    debug!("First word: {:0X}", first_word);

    use kernel::platform::sync::HardwareSyncAccess;
    let allocated = hw_sync_access.access(true, |hsb| hsb.spinlocks_allocated()).unwrap();
    debug!("Spinlock allocation: {}", allocated);
    {
        let (_sl0, _sl1, _sl2, allocated) = hw_sync_access.access(true, |hsb| {
            (hsb.get_spinlock(), hsb.get_spinlock(), hsb.get_spinlock(), hsb.spinlocks_allocated())
        }).unwrap();
        debug!("After a few allocations: {}", allocated);
    }
    for i in 0..1000 { if sio.fifo_valid() { sio.read_fifo(); } }
    let allocated = hw_sync_access.access(true, |hsb| hsb.spinlocks_allocated()).unwrap();
    debug!("After fall from scope: {}", allocated);

    debug!("Initialization complete. Enter main loop");

    board_kernel.kernel_loop(
        &raspberry_pi_pico,
        chip,
        Some(&raspberry_pi_pico.ipc),
        &main_loop_capability,
    );
}
