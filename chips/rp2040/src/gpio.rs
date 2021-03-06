//! GPIO, RP2040
//!
//! ### Author
//! * Ioana Culic <ioana.culic@wyliodrin.com>

use cortexm0p::nvic::Nvic;
use enum_primitive::cast::FromPrimitive;
use enum_primitive::enum_from_primitive;
use kernel::hil;
use kernel::utilities::cells::OptionalCell;
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::utilities::registers::{register_bitfields, register_structs, ReadOnly, ReadWrite, WriteOnly};
use kernel::utilities::StaticRef;

use crate::chip::Processor;
use crate::interrupts::{SIO_IRQ_PROC0, SIO_IRQ_PROC1};

#[repr(C)]
struct GpioPin {
    status: ReadOnly<u32, GPIOx_STATUS::Register>,
    ctrl: ReadWrite<u32, GPIOx_CTRL::Register>,
}
#[repr(C)]
struct GpioProc {
    enable: [ReadWrite<u32, GPIO_INTxx::Register>; 4],
    force: [ReadWrite<u32, GPIO_INTxx::Register>; 4],
    status: [ReadWrite<u32, GPIO_INTxx::Register>; 4],
}

register_structs! {
    /// GPIO Registers.
    GpioRegisters {
        (0x000 => pin: [GpioPin; 30]),

        /// Raw interrupts
        (0x0f0 => intr: [ReadWrite<u32, GPIO_INTxx::Register>; 4]),

        /// Interrupts for procs
        (0x100 => interrupt_proc: [GpioProc; 2]),

        /// Wake
        (0x160 => wake: GpioProc),

        /// End
        (0x190 => @END),
    },
    /// User Bank Pad Control Registers
    GpioPadRegisters {
        /// Voltage select
        (0x00 => voltage: ReadWrite<u32, VOLTAGE_SELECT::Register>),

        /// Pads control
        (0x04 => gpio_pad: [ReadWrite<u32, GPIO_PAD::Register>; 32]),

        /// End
        (0x84 => @END),
    },
    /// SIO Control Registers
    SIORegisters {
        /// Not used
        (0x000 => cpuid: ReadOnly<u32, CPUID::Register>),

        /// Input value for GPIO pins
        (0x004 => gpio_in: ReadOnly<u32, GPIO_IN::Register>),

        /// Not used
        (0x008 => _reserved1),

        /// GPIO output value
        (0x010 => gpio_out: ReadWrite<u32, GPIO_OUT::Register>),

        /// GPIO output value set
        (0x014 => gpio_out_set: ReadWrite<u32, GPIO_OUT_SET::Register>),

        /// GPIO output value clear
        (0x018 => gpio_out_clr: ReadWrite<u32, GPIO_OUT_CLR::Register>),

        /// GPIO output value XOR
        (0x01c => gpio_out_xor: ReadWrite<u32, GPIO_OUT_XOR::Register>),

        /// GPIO output enable
        (0x020 => gpio_oe: ReadWrite<u32, GPIO_OE::Register>),

        /// GPIO output enable set
        (0x024 => gpio_oe_set: ReadWrite<u32, GPIO_OE_SET::Register>),

        /// GPIO output enable clear
        (0x028 => gpio_oe_clr: ReadWrite<u32, GPIO_OE_CLR::Register>),

        /// Not used
        (0x02C => _reserved2),

        /// FIFO status
        (0x050 => fifo_st: ReadWrite<u32, FIFO_ST::Register>),

        /// FIFO write
        (0x054 => fifo_wr: ReadWrite<u32, FIFO_WR::Register>),

        /// FIFO read
        (0x058 => fifo_rd: ReadOnly<u32, FIFO_RD::Register>),

        /// Hardware spinlock state
        (0x05c => spinlock_st: ReadOnly<u32, SPINLOCK_ST::Register>),

        /// Divider unsigned dividend
        (0x060 => div_udividend: WriteOnly<u32, DIV_UDIVIDEND::Register>),

        /// Divider unsigned divisor
        (0x064 => div_udivisor: WriteOnly<u32, DIV_UDIVISOR::Register>),

        /// Divider signed dividend
        (0x068 => div_sdividend: WriteOnly<u32, DIV_SDIVIDEND::Register>),

        /// Divider signed divisor
        (0x06c => div_sdivisor: WriteOnly<u32, DIV_SDIVISOR::Register>),

        /// Divider result quotient
        (0x070 => div_quotient: ReadOnly<u32, DIV_QUOTIENT::Register>),

        /// Divider result remainder
        (0x074 => div_remainder: ReadOnly<u32, DIV_REMAINDER::Register>),

        /// Control and status register for divider
        (0x078 => div_csr: ReadWrite<u32, DIV_CSR::Register>),

        /// Easy-to-miss gap...
        (0x07c => _reserved3),

        /// Read/write access to accumulator 0
        (0x080 => interp0_accum0: ReadWrite<u32, INTERP0_ACCUM0::Register>),

        /// Read/write access to accumulator 1
        (0x084 => interp0_accum1: ReadWrite<u32, INTERP0_ACCUM1::Register>),

        /// Read/write access to BASE0 register
        (0x088 => interp0_base0: ReadWrite<u32, INTERP0_BASE0::Register>),

        /// Read/write access to BASE1 register
        (0x08c => interp0_base1: ReadWrite<u32, INTERP0_BASE1::Register>),

        /// Read/write access to BASE2 register
        (0x090 => interp0_base2: ReadWrite<u32, INTERP0_BASE2::Register>),

        /// Read LANE0 result and simultaneously write lane results to both accumulators (POP)
        (0x094 => interp0_pop_lane0: ReadOnly<u32, INTERP0_POP_LANE0::Register>),

        /// Read LANE1 result and simultaneously write lane results to both accumulators (POP)
        (0x098 => interp0_pop_lane1: ReadOnly<u32, INTERP0_POP_LANE1::Register>),

        /// Read FULL result and simultaneously write lane results to both accumulators (POP)
        (0x09c => interp0_pop_full: ReadOnly<u32, INTERP0_POP_FULL::Register>),

        /// Read LANE0 result without altering any internal state (PEEK)
        (0x0a0 => interp0_peek_lane0: ReadOnly<u32, INTERP0_PEEK_LANE0::Register>),

        /// Read LANE1 result without altering any internal state (PEEK)
        (0x0a4 => interp0_peek_lane1: ReadOnly<u32, INTERP0_PEEK_LANE1::Register>),

        /// Read FULL result without altering any internal state (PEEK)
        (0x0a8 => interp0_peek_full: ReadOnly<u32, INTERP0_PEEK_FULL::Register>),

        /// Control register for lane 0
        (0x0ac => interp0_ctrl_lane0: ReadOnly<u32, INTERP0_CTRL_LANE0::Register>),

        /// Control register for lane 1
        (0x0b0 => interp0_ctrl_lane1: ReadOnly<u32, INTERP0_CTRL_LANE1::Register>),

        /// Values written here are atomically added to ACCUM0
        (0x0b4 => interp0_accum0_add: ReadWrite<u32, INTERP0_ACCUM0_ADD::Register>),

        /// Values written here are atomically added to ACCUM1
        (0x0b8 => interp0_accum1_add: ReadWrite<u32, INTERP0_ACCUM1_ADD::Register>),

        /// On write, the lower 16 bits go to BASE0, upper bits to BASE1 simultaneously
        (0x0bc => interp0_base_1and0: WriteOnly<u32, INTERP0_BASE_1AND0::Register>),

        /// Read/write access to accumulator 0
        (0x0c0 => interp1_accum0: ReadWrite<u32, INTERP1_ACCUM0::Register>),

        /// Read/write access to accumulator 1
        (0x0c4 => interp1_accum1: ReadWrite<u32, INTERP1_ACCUM1::Register>),

        /// Read/write access to BASE0 register
        (0x0c8 => interp1_base0: ReadWrite<u32, INTERP1_BASE0::Register>),

        /// Read/write access to BASE1 register
        (0x0cc => interp1_base1: ReadWrite<u32, INTERP1_BASE1::Register>),

        /// Read/write access to BASE2 register
        (0x0d0 => interp1_base2: ReadWrite<u32, INTERP1_BASE2::Register>),

        /// Read LANE0 result and simultaneously write lane results to both accumulators (POP)
        (0x0d4 => interp1_pop_lane0: ReadOnly<u32, INTERP1_POP_LANE0::Register>),

        /// Read LANE1 result and simultaneously write lane results to both accumulators (POP)
        (0x0d8 => interp1_pop_lane1: ReadOnly<u32, INTERP1_POP_LANE1::Register>),

        /// Read FULL result and simultaneously write lane results to both accumulators (POP)
        (0x0dc => interp1_pop_full: ReadOnly<u32, INTERP1_POP_FULL::Register>),

        /// Read LANE0 result without altering any internal state (PEEK)
        (0x0e0 => interp1_peek_lane0: ReadOnly<u32, INTERP1_PEEK_LANE0::Register>),

        /// Read LANE1 result without altering any internal state (PEEK)
        (0x0e4 => interp1_peek_lane1: ReadOnly<u32, INTERP1_PEEK_LANE1::Register>),

        /// Read FULL result without altering any internal state (PEEK)
        (0x0e8 => interp1_peek_full: ReadOnly<u32, INTERP1_PEEK_FULL::Register>),

        /// Control register for lane 0
        (0x0ec => interp1_ctrl_lane0: ReadOnly<u32, INTERP1_CTRL_LANE0::Register>),

        /// Control register for lane 1
        (0x0f0 => interp1_ctrl_lane1: ReadOnly<u32, INTERP1_CTRL_LANE1::Register>),

        /// Values written here are atomically added to ACCUM0
        (0x0f4 => interp1_accum0_add: ReadWrite<u32, INTERP1_ACCUM0_ADD::Register>),

        /// Values written here are atomically added to ACCUM1
        (0x0f8 => interp1_accum1_add: ReadWrite<u32, INTERP1_ACCUM1_ADD::Register>),

        /// On write, the lower 16 bits go to BASE0, upper bits to BASE1 simultaneously
        (0x0fc => interp1_base_1and0: WriteOnly<u32, INTERP1_BASE_1AND0::Register>),

        /// Hardware spinlock
        (0x100 => spinlock: [ReadWrite<u32, SPINLOCK::Register>; 32]),

        /// End
        (0x180 => @END),
    }
}

register_bitfields![
    u32,
    GPIOx_STATUS [
        /// interrupt to processors, after override is applied
        IRQTOPROC OFFSET(26) NUMBITS(1) [],
        /// interrupt from pad before override is applied
        IRQFROMPAD OFFSET(24) NUMBITS(1) [],
        /// input signal to peripheral, after override is applied
        INTOPERI OFFSET(19) NUMBITS(1) [],
        /// input signal from pad, before override is applied
        INFROMPAD OFFSET(17) NUMBITS(1) [],
        /// output enable to pad after register override is applied
        OETOPAD OFFSET(13) NUMBITS(1) [],
        /// output enable from selected peripheral, before registeroverride is applied
        OEFROMPERI OFFSET(12) NUMBITS(1) [],
        /// output signal to pad after register override is applied
        OUTTOPAD OFFSET(9) NUMBITS(1) [],
        /// output signal from selected peripheral, before registeroverride is applied
        OUTFROMPERI OFFSET(8) NUMBITS(1) []
    ],
    GPIOx_CTRL [
        /// interrupt override?
        IRQOVER OFFSET(28) NUMBITS(2) [
            NoInvert = 0,
            Invert = 1,
            DriveLow = 2,
            DriveHigh = 3
        ],
        /// input override
        INOVER OFFSET(16) NUMBITS(2) [
            NoInvert = 0,
            Invert = 1,
            DriveLow = 2,
            DriveHigh = 3
        ],
        /// output enable override
        OEOVER OFFSET(12) NUMBITS(2) [
            EnableSignal = 0,
            EnableInverseSignal = 1,
            Disable = 2,
            Enable = 3
        ],
        /// output override
        OUTOVER OFFSET(8) NUMBITS(2) [
            Signal = 0,
            InverseSignal = 1,
            Low = 2,
            High = 3
        ],
        /// Function select
        FUNCSEL OFFSET(0) NUMBITS(5) [
            GPIO_FUNC_XIP = 0,
            GPIO_FUNC_SPI = 1,
            GPIO_FUNC_UART = 2,
            GPIO_FUNC_I2C = 3,
            GPIO_FUNC_PWM = 4,
            GPIO_FUNC_SIO = 5,
            GPIO_FUNC_PIO0 = 6,
            GPIO_FUNC_PIO1 = 7,
            GPIO_FUNC_GPCK = 8,
            GPIO_FUNC_USB = 9,
            GPIO_FUNC_NULL = 0x1f
        ]
    ],
    GPIO_INTxx [
        GPIO7_EDGE_HIGH OFFSET(31) NUMBITS(1) [],
        GPIO7_EDGE_LOW OFFSET(30) NUMBITS(1) [],
        GPIO7_LEVEL_HIGH OFFSET(29) NUMBITS(1) [],
        GPIO7_LEVEL_LOW OFFSET(28) NUMBITS(1) [],

        GPIO6_EDGE_HIGH OFFSET(27) NUMBITS(1) [],
        GPIO6_EDGE_LOW OFFSET(26) NUMBITS(1) [],
        GPIO6_LEVEL_HIGH OFFSET(25) NUMBITS(1) [],
        GPIO6_LEVEL_LOW OFFSET(24) NUMBITS(1) [],

        GPIO5_EDGE_HIGH OFFSET(23) NUMBITS(1) [],
        GPIO5_EDGE_LOW OFFSET(22) NUMBITS(1) [],
        GPIO5_LEVEL_HIGH OFFSET(21) NUMBITS(1) [],
        GPIO5_LEVEL_LOW OFFSET(20) NUMBITS(1) [],

        GPIO4_EDGE_HIGH OFFSET(19) NUMBITS(1) [],
        GPIO4_EDGE_LOW OFFSET(18) NUMBITS(1) [],
        GPIO4_LEVEL_HIGH OFFSET(17) NUMBITS(1) [],
        GPIO4_LEVEL_LOW OFFSET(16) NUMBITS(1) [],

        GPIO3_EDGE_HIGH OFFSET(15) NUMBITS(1) [],
        GPIO3_EDGE_LOW OFFSET(14) NUMBITS(1) [],
        GPIO3_LEVEL_HIGH OFFSET(13) NUMBITS(1) [],
        GPIO3_LEVEL_LOW OFFSET(12) NUMBITS(1) [],

        GPIO2_EDGE_HIGH OFFSET(11) NUMBITS(1) [],
        GPIO2_EDGE_LOW OFFSET(10) NUMBITS(1) [],
        GPIO2_LEVEL_HIGH OFFSET(9) NUMBITS(1) [],
        GPIO2_LEVEL_LOW OFFSET(8) NUMBITS(1) [],

        GPIO1_EDGE_HIGH OFFSET(7) NUMBITS(1) [],
        GPIO1_EDGE_LOW OFFSET(6) NUMBITS(1) [],
        GPIO1_LEVEL_HIGH OFFSET(5) NUMBITS(1) [],
        GPIO1_LEVEL_LOW OFFSET(4) NUMBITS(1) [],

        GPIO0_EDGE_HIGH OFFSET(3) NUMBITS(1) [],
        GPIO0_EDGE_LOW OFFSET(2) NUMBITS(1) [],
        GPIO0_LEVEL_HIGH OFFSET(1) NUMBITS(1) [],
        GPIO0_LEVEL_LOW OFFSET(0) NUMBITS(1) []
    ],
    VOLTAGE_SELECT[
        VOLTAGE OFFSET(0) NUMBITS(1) [
            Set3V3 = 0,
            Set1V8 = 1
        ]
    ],
    GPIO_PAD [
        OD OFFSET(7) NUMBITS(1) [],
        IE OFFSET(6) NUMBITS(1) [],
        DRIVE OFFSET(4) NUMBITS(2) [],
        PUE OFFSET(3) NUMBITS(1) [],
        PDE OFFSET(2) NUMBITS(1) [],
        SCHMITT OFFSET(1) NUMBITS(1) [],
        SLEWFAST OFFSET(0) NUMBITS(1) []
    ],
    GPIO_IN [
        ///Input value for GPIO0..29
        IN OFFSET(0) NUMBITS(30) []
    ],
    GPIO_OUT [
        ///Set output level (1/0 ??? high/low) for GPIO0...29.
        OUT OFFSET(0) NUMBITS(30) []
    ],
    GPIO_OUT_SET [
        ///Perform an atomic bit-set on GPIO_OUT
        OUT OFFSET(0) NUMBITS(30) []
    ],
    GPIO_OUT_CLR [
        ///Perform an atomic bit-clear on GPIO_OUT
        OUT OFFSET(0) NUMBITS(30) []
    ],
    GPIO_OUT_XOR [
        ///Perform an atomic bitwise XOR on GPIO_OUT
        OUT OFFSET(0) NUMBITS(30) []
    ],
    GPIO_OE [
        ///Set output enable (1/0 ??? output/input) for GPIO0...29
        OE OFFSET(0) NUMBITS(30) []
    ],
    GPIO_OE_SET [
        ///Perform an atomic bit-set on GPIO_OE
        OE OFFSET(0) NUMBITS(30) []
    ],
    GPIO_OE_CLR [
        ///Perform an atomic bit-clear on GPIO_OE
        OE OFFSET(0) NUMBITS(30) []
    ],
    CPUID [
        VALUE OFFSET(0) NUMBITS (32)
    ],
    FIFO_ST [
        /// FIFO read when empty
        ROE OFFSET(3) NUMBITS(1) [],
        /// FIFO written when full
        WOF OFFSET(2) NUMBITS(1) [],
        /// FIFO not full
        RDY OFFSET(1) NUMBITS(1) [],
        /// FIFO not empty
        VLD OFFSET(0) NUMBITS(1) []
    ],
    FIFO_WR [
        /// FIFO Write
        VALUE OFFSET(0) NUMBITS(32)
    ],
    FIFO_RD [
        /// FIFO Read
        VALUE OFFSET(0) NUMBITS(32)
    ],
    SPINLOCK_ST [
        STATE OFFSET(0) NUMBITS(32)
    ],

    DIV_UDIVIDEND [
        VALUE OFFSET(0) NUMBITS(32)
    ],

    DIV_UDIVISOR [
        VALUE OFFSET(0) NUMBITS(32)
    ],

    DIV_SDIVIDEND [
        VALUE OFFSET(0) NUMBITS(32)
    ],

    DIV_SDIVISOR [
        VALUE OFFSET(0) NUMBITS(32)
    ],

    DIV_QUOTIENT [
        VALUE OFFSET(0) NUMBITS(32)
    ],

    DIV_REMAINDER [
        VALUE OFFSET(0) NUMBITS(32)
    ],

    DIV_CSR [
        DIRTY OFFSET(1) NUMBITS(1),
        READY OFFSET(0) NUMBITS(1)
    ],

    INTERP0_ACCUM0 [
        VALUE OFFSET(0) NUMBITS(32)
    ],

    INTERP0_ACCUM1 [
        VALUE OFFSET(0) NUMBITS(32)
    ],

    INTERP0_BASE0 [
        VALUE OFFSET(0) NUMBITS(32)
    ],

    INTERP0_BASE1 [
        VALUE OFFSET(0) NUMBITS(32)
    ],

    INTERP0_BASE2 [
        VALUE OFFSET(0) NUMBITS(32)
    ],

    INTERP0_POP_LANE0 [
        RESULT OFFSET(0) NUMBITS(32)
    ],

    INTERP0_POP_LANE1 [
        RESULT OFFSET(0) NUMBITS(32)
    ],

    INTERP0_POP_FULL [
        RESULT OFFSET(0) NUMBITS(32)
    ],

    INTERP0_PEEK_LANE0 [
        RESULT OFFSET(0) NUMBITS(32)
    ],

    INTERP0_PEEK_LANE1 [
        RESULT OFFSET(0) NUMBITS(32)
    ],

    INTERP0_PEEK_FULL [
        RESULT OFFSET(0) NUMBITS(32)
    ],

    INTERP0_CTRL_LANE0 [
        OVERF OFFSET(25) NUMBITS(1),
        OVERF1 OFFSET(24) NUMBITS(1),
        OVERF0 OFFSET(23) NUMBITS(1),
        CLAMP OFFSET(22) NUMBITS(1),
        FORCE_MSB OFFSET(19) NUMBITS(2),
        ADD_RAW OFFSET(18) NUMBITS(1),
        CROSS_RESULT OFFSET(17) NUMBITS(1),
        CROSS_INPUT OFFSET(16) NUMBITS(1),
        SIGNED OFFSET(15) NUMBITS(1),
        MASK_MSB OFFSET(10) NUMBITS(5),
        MASK_LSB OFFSET(5) NUMBITS(5),
        SHIFT OFFSET(0) NUMBITS(5)
    ],

    INTERP0_CTRL_LANE1 [
        FORCE_MSB OFFSET(19) NUMBITS(2),
        ADD_RAW OFFSET(18) NUMBITS(1),
        CROSS_RESULT OFFSET(17) NUMBITS(1),
        CROSS_INPUT OFFSET(16) NUMBITS(1),
        SIGNED OFFSET(15) NUMBITS(1),
        MASK_MSB OFFSET(10) NUMBITS(5),
        MASK_LSB OFFSET(5) NUMBITS(5),
        SHIFT OFFSET(0) NUMBITS(5)
    ],

    INTERP0_ACCUM0_ADD [
        VALUE OFFSET(0) NUMBITS(24)
    ],

    INTERP0_ACCUM1_ADD [
        VALUE OFFSET(0) NUMBITS(24)
    ],

    INTERP0_BASE_1AND0 [
        BASE1_BASE0 OFFSET(0) NUMBITS(32)
    ],

    INTERP1_ACCUM0 [
        VALUE OFFSET(0) NUMBITS(32)
    ],

    INTERP1_ACCUM1 [
        VALUE OFFSET(0) NUMBITS(32)
    ],

    INTERP1_BASE0 [
        VALUE OFFSET(0) NUMBITS(32)
    ],

    INTERP1_BASE1 [
        VALUE OFFSET(0) NUMBITS(32)
    ],

    INTERP1_BASE2 [
        VALUE OFFSET(0) NUMBITS(32)
    ],

    INTERP1_POP_LANE0 [
        RESULT OFFSET(0) NUMBITS(32)
    ],

    INTERP1_POP_LANE1 [
        RESULT OFFSET(0) NUMBITS(32)
    ],
    INTERP1_POP_FULL [
        RESULT OFFSET(0) NUMBITS(32)
    ],

    INTERP1_PEEK_LANE0 [
        RESULT OFFSET(0) NUMBITS(32)
    ],

    INTERP1_PEEK_LANE1 [
        RESULT OFFSET(0) NUMBITS(32)
    ],

    INTERP1_PEEK_FULL [
        RESULT OFFSET(0) NUMBITS(32)
    ],

    INTERP1_CTRL_LANE0 [
        OVERF OFFSET(25) NUMBITS(1),
        OVERF1 OFFSET(24) NUMBITS(1),
        OVERF0 OFFSET(23) NUMBITS(1),
        CLAMP OFFSET(22) NUMBITS(1),
        FORCE_MSB OFFSET(19) NUMBITS(2),
        ADD_RAW OFFSET(18) NUMBITS(1),
        CROSS_RESULT OFFSET(17) NUMBITS(1),
        CROSS_INPUT OFFSET(16) NUMBITS(1),
        SIGNED OFFSET(15) NUMBITS(1),
        MASK_MSB OFFSET(10) NUMBITS(5),
        MASK_LSB OFFSET(5) NUMBITS(5),
        SHIFT OFFSET(0) NUMBITS(5)
    ],

    INTERP1_CTRL_LANE1 [
        FORCE_MSB OFFSET(19) NUMBITS(2),
        ADD_RAW OFFSET(18) NUMBITS(1),
        CROSS_RESULT OFFSET(17) NUMBITS(1),
        CROSS_INPUT OFFSET(16) NUMBITS(1),
        SIGNED OFFSET(15) NUMBITS(1),
        MASK_MSB OFFSET(10) NUMBITS(5),
        MASK_LSB OFFSET(5) NUMBITS(5),
        SHIFT OFFSET(0) NUMBITS(5)
    ],

    INTERP1_ACCUM0_ADD [
        VALUE OFFSET(0) NUMBITS(24)
    ],

    INTERP1_ACCUM1_ADD [
        VALUE OFFSET(0) NUMBITS(24)
    ],

    INTERP1_BASE_1AND0 [
        BASE1_BASE0 OFFSET(0) NUMBITS(32)
    ],

    SPINLOCK [
        STATE OFFSET(0) NUMBITS(32)
    ]
];

const GPIO_BASE_ADDRESS: usize = 0x40014000;
const GPIO_BASE: StaticRef<GpioRegisters> =
    unsafe { StaticRef::new(GPIO_BASE_ADDRESS as *const GpioRegisters) };

const GPIO_PAD_BASE_ADDRESS: usize = 0x4001c000;
const GPIO_PAD_BASE: StaticRef<GpioPadRegisters> =
    unsafe { StaticRef::new(GPIO_PAD_BASE_ADDRESS as *const GpioPadRegisters) };

const SIO_BASE_ADDRESS: usize = 0xd0000000;
const SIO_BASE: StaticRef<SIORegisters> =
    unsafe { StaticRef::new(SIO_BASE_ADDRESS as *const SIORegisters) };

pub struct RPPins<'a> {
    pub pins: [RPGpioPin<'a>; 30],
    gpio_registers: StaticRef<GpioRegisters>,
}

impl<'a> RPPins<'a> {
    pub const fn new() -> Self {
        Self {
            pins: [
                RPGpioPin::new(RPGpio::GPIO0),
                RPGpioPin::new(RPGpio::GPIO1),
                RPGpioPin::new(RPGpio::GPIO2),
                RPGpioPin::new(RPGpio::GPIO3),
                RPGpioPin::new(RPGpio::GPIO4),
                RPGpioPin::new(RPGpio::GPIO5),
                RPGpioPin::new(RPGpio::GPIO6),
                RPGpioPin::new(RPGpio::GPIO7),
                RPGpioPin::new(RPGpio::GPIO8),
                RPGpioPin::new(RPGpio::GPIO9),
                RPGpioPin::new(RPGpio::GPIO10),
                RPGpioPin::new(RPGpio::GPIO11),
                RPGpioPin::new(RPGpio::GPIO12),
                RPGpioPin::new(RPGpio::GPIO13),
                RPGpioPin::new(RPGpio::GPIO14),
                RPGpioPin::new(RPGpio::GPIO15),
                RPGpioPin::new(RPGpio::GPIO16),
                RPGpioPin::new(RPGpio::GPIO17),
                RPGpioPin::new(RPGpio::GPIO18),
                RPGpioPin::new(RPGpio::GPIO19),
                RPGpioPin::new(RPGpio::GPIO20),
                RPGpioPin::new(RPGpio::GPIO21),
                RPGpioPin::new(RPGpio::GPIO22),
                RPGpioPin::new(RPGpio::GPIO23),
                RPGpioPin::new(RPGpio::GPIO24),
                RPGpioPin::new(RPGpio::GPIO25),
                RPGpioPin::new(RPGpio::GPIO26),
                RPGpioPin::new(RPGpio::GPIO27),
                RPGpioPin::new(RPGpio::GPIO28),
                RPGpioPin::new(RPGpio::GPIO29),
            ],
            gpio_registers: GPIO_BASE,
        }
    }

    pub fn get_pin(&self, pin: RPGpio) -> &'a RPGpioPin {
        &self.pins[pin as usize]
    }

    pub fn handle_interrupt(&self) {
        for bank_no in 0..4 {
            let current_val = self.gpio_registers.intr[bank_no].get();
            let enabled_val = self.gpio_registers.interrupt_proc[0].enable[bank_no].get();
            for pin in 0..8 {
                let l_low_reg_no = pin * 4;
                if (current_val & enabled_val & (1 << l_low_reg_no)) != 0 {
                    self.pins[pin + bank_no * 8].handle_interrupt();
                } else if (current_val & enabled_val & (1 << l_low_reg_no + 1)) != 0 {
                    self.pins[pin + bank_no * 8].handle_interrupt();
                } else if (current_val & enabled_val & (1 << l_low_reg_no + 2)) != 0 {
                    self.gpio_registers.intr[bank_no].set(current_val & (1 << l_low_reg_no + 2));
                    self.pins[pin + bank_no * 8].handle_interrupt();
                } else if (current_val & enabled_val & (1 << l_low_reg_no + 3)) != 0 {
                    self.gpio_registers.intr[bank_no].set(current_val & (1 << l_low_reg_no + 3));
                    self.pins[pin + bank_no * 8].handle_interrupt();
                }
            }
        }
    }
}

enum_from_primitive! {
    #[derive(Copy, Clone, PartialEq)]
    #[repr(usize)]
    #[rustfmt::skip]
    pub enum RPGpio {
        GPIO0=0, GPIO1=1, GPIO2=2, GPIO3=3, GPIO4=4, GPIO5=5, GPIO6=6, GPIO7=7,
        GPIO8=8, GPIO9=9, GPIO10=10, GPIO11=11, GPIO12=12, GPIO13=13, GPIO14=14, GPIO15=15,
        GPIO16=16, GPIO17=17, GPIO18=18, GPIO19=19, GPIO20=20, GPIO21=21, GPIO22=22, GPIO23=23,
        GPIO24=24, GPIO25=25, GPIO26=26, GPIO27=27, GPIO28=28, GPIO29=29
    }
}
enum_from_primitive! {
    #[derive(Copy, Clone, PartialEq)]
    #[repr(u32)]
    #[rustfmt::skip]

    pub enum GpioFunction {
       SPI = 1,
       UART = 2,
       I2C = 3,
       PWM = 4,
       SIO = 5,
       PIO0 = 6,
       PIO1 = 7,
       GPCK = 8,
       USB = 9,
       NULL = 0x1f
    }
}

pub struct RPGpioPin<'a> {
    pin: usize,
    client: OptionalCell<&'a dyn hil::gpio::Client>,
    gpio_registers: StaticRef<GpioRegisters>,
    gpio_pad_registers: StaticRef<GpioPadRegisters>,
    sio_registers: StaticRef<SIORegisters>,
}

impl<'a> RPGpioPin<'a> {
    pub const fn new(pin: RPGpio) -> RPGpioPin<'a> {
        RPGpioPin {
            pin: pin as usize,
            client: OptionalCell::empty(),
            gpio_registers: GPIO_BASE,
            gpio_pad_registers: GPIO_PAD_BASE,
            sio_registers: SIO_BASE,
        }
    }

    fn get_mode(&self) -> hil::gpio::Configuration {
        //TODO - read alternate function
        let pad_output_disable = !self.gpio_pad_registers.gpio_pad[self.pin].is_set(GPIO_PAD::OD);
        let pin_mask = 1 << self.pin;
        let sio_output_enable = (self.sio_registers.gpio_oe.read(GPIO_OE::OE) & pin_mask) != 0;

        match (pad_output_disable, sio_output_enable) {
            (true, true) => hil::gpio::Configuration::Output,
            (true, false) => hil::gpio::Configuration::Input,
            (false, _) => hil::gpio::Configuration::LowPower,
        }
    }

    fn read_pin(&self) -> bool {
        //TODO - read alternate function
        let value = self.sio_registers.gpio_out.read(GPIO_OUT::OUT) & (1 << self.pin);
        if value == 0 {
            false
        } else {
            true
        }
    }

    pub fn set_function(&self, f: GpioFunction) {
        self.activate_pads();
        self.gpio_registers.pin[self.pin]
            .ctrl
            .write(GPIOx_CTRL::FUNCSEL.val(f as u32));
    }

    fn get_pullup_pulldown(&self) -> hil::gpio::FloatingState {
        //TODO - read alternate function
        let pullup = self.gpio_pad_registers.gpio_pad[self.pin].read(GPIO_PAD::PUE);
        let pulldown = self.gpio_pad_registers.gpio_pad[self.pin].read(GPIO_PAD::PDE);

        match (pullup, pulldown) {
            (0, 0) => hil::gpio::FloatingState::PullNone,
            (0, 1) => hil::gpio::FloatingState::PullDown,
            (1, 0) => hil::gpio::FloatingState::PullUp,
            _ => panic!("Invalid GPIO floating state."),
        }
    }

    pub fn activate_pads(&self) {
        self.gpio_pad_registers.gpio_pad[self.pin].modify(GPIO_PAD::OD::CLEAR + GPIO_PAD::IE::SET);
    }

    pub fn deactivate_pads(&self) {
        self.gpio_pad_registers.gpio_pad[self.pin].modify(GPIO_PAD::OD::SET + GPIO_PAD::IE::CLEAR);
    }

    pub fn handle_interrupt(&self) {
        self.client.map(|client| client.fired());
    }
}

impl<'a> hil::gpio::Interrupt<'a> for RPGpioPin<'a> {
    fn set_client(&self, client: &'a dyn hil::gpio::Client) {
        self.client.set(client);
    }

    fn is_pending(&self) -> bool {
        let interrupt_bank_no = self.pin / 8;
        let l_low_reg_no = (self.pin * 4) % 32;
        let current_val = self.gpio_registers.interrupt_proc[0].status[interrupt_bank_no].get();
        if (current_val
            & (1 << l_low_reg_no)
            & (1 << l_low_reg_no + 1)
            & (1 << l_low_reg_no + 2)
            & (1 << l_low_reg_no + 3))
            == 0
        {
            false
        } else {
            true
        }
    }

    fn enable_interrupts(&self, mode: hil::gpio::InterruptEdge) {
        let interrupt_bank_no = self.pin / 8;
        match mode {
            hil::gpio::InterruptEdge::RisingEdge => {
                let high_reg_no = (self.pin * 4 + 3) % 32;
                let current_val =
                    self.gpio_registers.interrupt_proc[0].enable[interrupt_bank_no].get();
                self.gpio_registers.interrupt_proc[0].enable[interrupt_bank_no]
                    .set((1 << high_reg_no) | current_val);
            }
            hil::gpio::InterruptEdge::FallingEdge => {
                let low_reg_no = (self.pin * 4 + 2) % 32;
                let current_val =
                    self.gpio_registers.interrupt_proc[0].enable[interrupt_bank_no].get();
                self.gpio_registers.interrupt_proc[0].enable[interrupt_bank_no]
                    .set((1 << low_reg_no) | current_val);
            }
            hil::gpio::InterruptEdge::EitherEdge => {
                let low_reg_no = (self.pin * 4 + 2) % 32;
                let high_reg_no = low_reg_no + 1;
                let current_val =
                    self.gpio_registers.interrupt_proc[0].enable[interrupt_bank_no].get();
                self.gpio_registers.interrupt_proc[0].enable[interrupt_bank_no]
                    .set((1 << high_reg_no) | (1 << low_reg_no) | current_val);
            }
        }
    }

    fn disable_interrupts(&self) {
        let interrupt_bank_no = self.pin / 8;
        let low_reg_no = (self.pin * 4 + 2) % 32;
        let high_reg_no = low_reg_no + 1;
        let current_val = self.gpio_registers.interrupt_proc[0].enable[interrupt_bank_no].get();
        self.gpio_registers.interrupt_proc[0].enable[interrupt_bank_no]
            .set(current_val & !(1 << high_reg_no) & !(1 << low_reg_no));
    }
}

impl hil::gpio::Configure for RPGpioPin<'_> {
    fn configuration(&self) -> hil::gpio::Configuration {
        self.get_mode()
    }
    /// Set output mode
    fn make_output(&self) -> hil::gpio::Configuration {
        self.set_function(GpioFunction::SIO);
        self.activate_pads();
        self.sio_registers.gpio_oe_set.set(1 << self.pin);
        self.get_mode()
    }
    /// Disable pad output
    fn disable_output(&self) -> hil::gpio::Configuration {
        self.set_function(GpioFunction::SIO);
        self.gpio_pad_registers.gpio_pad[self.pin].modify(GPIO_PAD::OD::SET);
        self.get_mode()
    }
    /// Set input mode
    fn make_input(&self) -> hil::gpio::Configuration {
        self.set_function(GpioFunction::SIO);
        self.activate_pads();
        self.sio_registers.gpio_oe_clr.set(1 << self.pin);
        self.get_mode()
    }
    /// Disable input mode, will set pin to output mode
    fn disable_input(&self) -> hil::gpio::Configuration {
        self.make_output()
    }
    fn deactivate_to_low_power(&self) {
        self.set_function(GpioFunction::SIO);
        self.gpio_pad_registers.gpio_pad[self.pin].modify(GPIO_PAD::OD::SET);
    }

    fn set_floating_state(&self, mode: hil::gpio::FloatingState) {
        match mode {
            hil::gpio::FloatingState::PullUp => self.gpio_pad_registers.gpio_pad[self.pin]
                .modify(GPIO_PAD::PUE::SET + GPIO_PAD::PDE::CLEAR),
            hil::gpio::FloatingState::PullDown => self.gpio_pad_registers.gpio_pad[self.pin]
                .modify(GPIO_PAD::PUE::CLEAR + GPIO_PAD::PDE::SET),
            hil::gpio::FloatingState::PullNone => self.gpio_pad_registers.gpio_pad[self.pin]
                .modify(GPIO_PAD::PUE::CLEAR + GPIO_PAD::PDE::CLEAR),
        }
    }

    fn floating_state(&self) -> hil::gpio::FloatingState {
        self.get_pullup_pulldown()
    }

    fn is_input(&self) -> bool {
        let mode = self.get_mode();
        match mode {
            hil::gpio::Configuration::Input => true,
            hil::gpio::Configuration::InputOutput => true,
            _ => false,
        }
    }

    fn is_output(&self) -> bool {
        let mode = self.get_mode();
        match mode {
            hil::gpio::Configuration::Output => true,
            hil::gpio::Configuration::InputOutput => true,
            _ => false,
        }
    }
}

impl hil::gpio::Output for RPGpioPin<'_> {
    fn set(&self) {
        // For performance this match might be skipped
        match self.get_mode() {
            hil::gpio::Configuration::Output | hil::gpio::Configuration::InputOutput => {
                self.sio_registers.gpio_out_set.set(1 << self.pin);
            }
            _ => {}
        }
    }

    fn clear(&self) {
        // For performance this match might be skipped
        match self.get_mode() {
            hil::gpio::Configuration::Output | hil::gpio::Configuration::InputOutput => {
                self.sio_registers.gpio_out_clr.set(1 << self.pin);
            }
            _ => {}
        }
    }

    fn toggle(&self) -> bool {
        match self.get_mode() {
            hil::gpio::Configuration::Output | hil::gpio::Configuration::InputOutput => {
                self.sio_registers.gpio_out_xor.set(1 << self.pin);
            }
            _ => {}
        }
        self.read_pin()
    }
}

impl hil::gpio::Input for RPGpioPin<'_> {
    fn read(&self) -> bool {
        let value = self.sio_registers.gpio_in.read(GPIO_IN::IN) & (1 << self.pin);
        if value == 0 {
            false
        } else {
            true
        }
    }
}

pub struct SIO {
    registers: StaticRef<SIORegisters>,
}

impl SIO {
    pub const fn new() -> Self {
        Self {
            registers: SIO_BASE,
        }
    }

    pub fn enable_interrupt(&self) {
        let index = match self.get_processor() {
            Processor::Processor0 => SIO_IRQ_PROC0,
            Processor::Processor1 => SIO_IRQ_PROC1,
        };

        unsafe { Nvic::new(index) }.enable();
    }

    pub fn disable_interrupt(&self) {
        use Processor::*;
        let index = match self.get_processor() {
            Processor0 => SIO_IRQ_PROC0,
            Processor1 => SIO_IRQ_PROC1,
        };

        unsafe { Nvic::new(index) }.enable();
    }

    pub fn handle_proc_interrupt(&self, for_processor: Processor) {
        match for_processor {
            Processor::Processor0 => {
                // read data from the fifo
                self.registers.fifo_rd.get();
                while self.fifo_valid() { let _ = self.read_fifo(); }
                self.registers.fifo_st.set(0xff);
            }
            Processor::Processor1 => {
                if self.registers.cpuid.get() == 1 {
                    panic!("Kernel should not run on processor 1");
                } else {
                    panic!("SIO_PROC1_IRQ should be ignored for processor 1");
                }
            }
        }
    }

    pub fn get_processor(&self) -> Processor {
        let proc_id = self.registers.cpuid.get();
        match proc_id {
            0 => Processor::Processor0,
            1 => Processor::Processor1,
            _ => panic!("SIO CPUID cannot be {}", proc_id),
        }
    }

    /// Read a word from the FIFO.
    #[inline]
    pub fn read_fifo(&self) -> u32 {
        self.registers.fifo_rd.get()
    }

    /// Write a word to the FIFO, returning the number of words prior to the write.
    #[inline]
    pub fn write_fifo(&self, data: u32) {
        self.registers.fifo_wr.set(data)
    }

    /// Returns true if data is present in the FIFO.
    #[inline]
    pub fn fifo_valid(&self) -> bool {
        self.registers.fifo_st.read(FIFO_ST::VLD) == 1
    }

    /// Returns true if there is space for data in the FIFO.
    #[inline]
    pub fn fifo_ready(&self) -> bool {
        self.registers.fifo_st.read(FIFO_ST::RDY) == 1
    }

    /// Returns the raw FIFO state.
    #[inline]
    pub fn fifo_state(&self) -> u32 {
        self.registers.fifo_st.get()
    }

    /// Returns true if there is a FIFO usage error.
    #[inline]
    pub fn fifo_error(&self) -> bool {
        let fifo_st = self.registers.fifo_st.get();
        fifo_st & ((FIFO_ST::ROE.mask << FIFO_ST::ROE.shift)
                   | (FIFO_ST::WOF.mask << FIFO_ST::WOF.shift)) != 0
    }

    #[inline]
    pub fn fifo_clear_error(&self) {
        self.registers.fifo_st.set((1 << 3) | (1 << 2));
    }

    /// Attempt to claim a spinlock.
    ///
    /// This operation will return true if claiming the spinlock succeeded.
    pub fn claim_spinlock(&self, lock_no: u8) -> bool {
        if lock_no < 32 {
            self.registers.spinlock[lock_no as usize]
                .read(SPINLOCK::STATE) != 0
        } else {
            false
        }
    }

    /// Release a spinlock.
    pub fn release_spinlock(&self, lock_no: u8) {
        if lock_no < 32 {
            self.registers.spinlock[lock_no as usize]
                .set(0xffff_ffff)
        }
    }

    /// Retrieve spinlock states.
    pub fn spinlock_state(&self) -> u32 {
        self.registers.spinlock_st.get()
    }
}
