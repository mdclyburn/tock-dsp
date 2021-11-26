//! RP2040 Direct Memory Access peripheral.

use core::cell::Cell;

use cortexm0p::nvic::Nvic;

use kernel::ErrorCode;
use kernel::hil::dma::DMAClient;
use kernel::utilities::StaticRef;
use kernel::utilities::cells::{OptionalCell, TakeCell};
use kernel::utilities::registers::{
    register_bitfields,
    register_structs,
    ReadOnly,
    ReadWrite,
};
use kernel::utilities::registers::interfaces::{
    Readable,
    Writeable,
};

register_structs! {
    DMAChannel {
        // Channel read address.
        (0x000 => read_addr: ReadWrite<u32, READ_ADDR::Register>),
        // Channel write address.
        (0x004 => write_addr: ReadWrite<u32, WRITE_ADDR::Register>),
        // Transfer count.
        (0x008 => trans_count: ReadWrite<u32, TRANS_COUNT::Register>),
        // Control and status.
        (0x00c => ctrl: ReadWrite<u32, CTRL::Register>),

        // ALIAS SET 1
        (0x010 => al1_ctrl: ReadWrite<u32, CTRL::Register>),
        (0x014 => al1_read_addr: ReadWrite<u32, READ_ADDR::Register>),
        (0x018 => al1_write_addr: ReadWrite<u32, WRITE_ADDR::Register>),
        (0x01c => al1_trans_count: ReadWrite<u32, TRANS_COUNT::Register>),

        // ALIAS SET 2
        (0x020 => al2_ctrl: ReadWrite<u32, CTRL::Register>),
        (0x024 => al2_trans_count: ReadWrite<u32, TRANS_COUNT::Register>),
        (0x028 => al2_write_addr: ReadWrite<u32, WRITE_ADDR::Register>),
        (0x02c => al2_read_addr: ReadWrite<u32, READ_ADDR::Register>),

        // ALIAS SET 3
        (0x030 => al3_ctrl: ReadWrite<u32, CTRL::Register>),
        (0x034 => al3_write_addr: ReadWrite<u32, WRITE_ADDR::Register>),
        (0x038 => al3_trans_count: ReadWrite<u32, TRANS_COUNT::Register>),
        (0x03c => al3_read_addr: ReadWrite<u32, READ_ADDR::Register>),

        (0x040 => @END),
    },

    Interrupts {
        /// Interrupt status (raw).
        (0x400 => intr: ReadOnly<u32, ()>),
        /// Interrupt enablement for DMA_IRQ0.
        (0x404 => inte0: ReadWrite<u32, ()>),
        /// Force interrupts.
        (0x408 => intf0: ReadWrite<u32, ()>),
        /// Interrupt status for DMA_IRQ0.
        (0x40c => ints0: ReadWrite<u32, ()>),

        (0x410 => _reserved0),

        /// Interrupt enablement for DMA_IRQ1.
        (0x414 => inte1: ReadWrite<u32, ()>),
        /// Force interrupts.
        (0x418 => intf1: ReadWrite<u32, ()>),
        /// Interrupt status for DMA_IRQ0.
        (0x41c => ints1: ReadWrite<u32, ()>),

        (0x420 => @END),
    },

    /// Pacing fractional timers.
    Timers {
        (0x420 => timer0: ReadWrite<u32, TIMER::Register>),
        (0x424 => timer1: ReadWrite<u32, TIMER::Register>),
        (0x428 => timer2: ReadWrite<u32, TIMER::Register>),
        (0x42c => timer3: ReadWrite<u32, TIMER::Register>),

        (0x430 => @END),
    }
}

#[allow(non_camel_case_types)]
register_bitfields! [
    u32,

    READ_ADDR [
        ADDR OFFSET(0) NUMBITS(32)
    ],
    WRITE_ADDR [
        ADDR OFFSET(0) NUMBITS(32)
    ],
    TRANS_COUNT [
        VALUE OFFSET(0) NUMBITS(32)
    ],
    CTRL [
        AHB_ERROR OFFSET(31) NUMBITS(1) [],
        READ_ERROR OFFSET(30) NUMBITS(1) [],
        WRITE_ERROR OFFSET(29) NUMBITS(1) [],
        // 28:25 reserved
        BUSY OFFSET(24) NUMBITS(1) [],
        SNIFF_EN OFFSET(23) NUMBITS(1) [],
        BSWAP OFFSET(22) NUMBITS(1) [],
        IRQ_QUIET OFFSET(21) NUMBITS(1) [],
        TREQ_SEL OFFSET(15) NUMBITS(6) [
            PIO0_TX0 = 0,
            PIO0_TX1 = 1,
            PIO0_TX2 = 2,
            PIO0_TX3 = 3,

            PIO0_RX0 = 4,
            PIO0_RX1 = 5,
            PIO0_RX2 = 6,
            PIO0_RX3 = 7,

            PIO1_TX0 = 8,
            PIO1_TX1 = 9,
            PIO1_TX2 = 10,
            PIO1_TX3 = 11,

            PIO1_RX0 = 12,
            PIO1_RX1 = 13,
            PIO1_RX2 = 14,
            PIO1_RX3 = 15,

            SPI0_TX = 16,
            SPI0_RX = 17,
            SPI1_TX = 18,
            SPI1_RX = 19,

            UART0_TX = 20,
            UART0_RX = 21,
            UART1_TX = 22,
            UART1_RX = 23,

            PWM_WRAP0 = 24,
            PWM_WRAP1 = 25,
            PWM_WRAP2 = 26,
            PWM_WRAP3 = 27,
            PWM_WRAP4 = 28,
            PWM_WRAP5 = 29,
            PWM_WRAP6 = 30,
            PWM_WRAP7 = 31,

            I2C0_TX = 32,
            I2C0_RX = 33,
            I2C1_TX = 34,
            I2C1_RX = 35,

            ADC = 36,

            XIP_STREAM = 37,
            XIP_SSITX = 38,
            XIP_SSIRX = 39,

            TIMER_0 = 0x3B,
            TIMER_1 = 0x3C,
            TIMER_2 = 0x3D,
            TIMER_3 = 0x3E,

            PERMANENT_REQUEST = 0x3F
        ],
        CHAIN_TO OFFSET(11) NUMBITS(4) [],
        RING_SEL OFFSET(10) NUMBITS(1) [
            ReadIsRing = 0,
            WriteIsRing = 1,
        ],
        RING_SIZE OFFSET(6) NUMBITS(4) [],
        INCR_WRITE OFFSET(5) NUMBITS(1) [],
        INCR_READ OFFSET(4) NUMBITS(1) [],
        DATA_SIZE OFFSET(2) NUMBITS(2) [
            Byte = 0,
            HalfWord = 1,
            Word = 2,
        ],
        HIGH_PRIORITY OFFSET(1) NUMBITS(1) [],
        EN OFFSET(0) NUMBITS(1) [
            Enable = 1,
            Disable = 0,
        ],
    ],

    // Fractional [(X/Y) * SYS_CLK] pacing timers.
    TIMER [
        X OFFSET(16) NUMBITS(16) [],
        Y OFFSET(0) NUMBITS(16) [],
    ],
];

/// Trigger signal for a DMA channel.
#[derive(Copy, Clone)]
#[repr(u32)]
#[allow(non_camel_case_types)]
pub enum TransferRequestSignal {
    PIO0_TX0 = 0,
    PIO0_TX1 = 1,
    PIO0_TX2 = 2,
    PIO0_TX3 = 3,

    PIO0_RX0 = 4,
    PIO0_RX1 = 5,
    PIO0_RX2 = 6,
    PIO0_RX3 = 7,

    PIO1_TX0 = 8,
    PIO1_TX1 = 9,
    PIO1_TX2 = 10,
    PIO1_TX3 = 11,

    PIO1_RX0 = 12,
    PIO1_RX1 = 13,
    PIO1_RX2 = 14,
    PIO1_RX3 = 15,

    SPI0_TX = 16,
    SPI0_RX = 17,
    SPI1_TX = 18,
    SPI1_RX = 19,

    UART0_TX = 20,
    UART0_RX = 21,
    UART1_TX = 22,
    UART1_RX = 23,

    PWM_WRAP0 = 24,
    PWM_WRAP1 = 25,
    PWM_WRAP2 = 26,
    PWM_WRAP3 = 27,
    PWM_WRAP4 = 28,
    PWM_WRAP5 = 29,
    PWM_WRAP6 = 30,
    PWM_WRAP7 = 31,

    I2C0_TX = 32,
    I2C0_RX = 33,
    I2C1_TX = 34,
    I2C1_RX = 35,

    ADC = 36,

    XIP_STREAM = 37,
    XIP_SSITX = 38,
    XIP_SSIRX = 39,

    TIMER_0 = 0x3B,
    TIMER_1 = 0x3C,
    TIMER_2 = 0x3D,
    TIMER_3 = 0x3E,

    PERMANENT_REQUEST = 0x3F
}

/// Whether to wrap writes or to wrap reads.
pub enum TransferAddressWrap {
    None,
    Read(u8),
    Write(u8),
}

/// Size of each DMA transfer operation (32-bit architecture).
#[derive(Copy, Clone)]
#[repr(u32)]
pub enum TransferSize {
    /// One byte.
    Byte = 0,
    /// Two bytes.
    HalfWord = 1,
    /// Four bytes.
    Word = 2,
}

/// NVIC IRQ line.
///
/// The RP2040 contains two interrupt lines for DMA interrupts,
/// `DMA_IRQ0` and `DMA_IRQ1`.
/// The exact usage of these two lines is left to software.
#[derive(Copy, Clone)]
#[allow(non_camel_case_types)]
pub enum InterruptLine {
    IRQ0,
    IRQ1
}

const DMA_BASE_ADDRESS: usize = 0x5000_0000;
const DMA_CHANNELS: StaticRef<[DMAChannel; 12]> = unsafe {
    StaticRef::new(DMA_BASE_ADDRESS as *const [DMAChannel; 12])
};

struct ChannelConfiguration {
    buffer: TakeCell<'static, [usize]>,
    transfer_client: &'static dyn DMAClient,
}

/// Configurable parameters for DMA channels.
pub struct ChannelOptions {
    /// Address to transfer data from.
    pub read_address: u32,
    /// Address to transfer data to.
    pub write_address: u32,
    /// Number of transfers to perform.
    pub transfer_count: u32,
    /// Size of each transfer.
    pub transfer_size: TransferSize,
    /// Transfer initiation signal.
    pub treq_signal: TransferRequestSignal,
    /// Transfer source or destination address wrapping.
    pub ring_config: TransferAddressWrap,
    /// Increase read address by `transfer_size` after each transfer.
    pub increment_on_read: bool,
    /// Increase write address by `transfer_size` after each transfer.
    pub increment_on_write: bool,
    /// Service this channel before non-high priority channels.
    pub high_priority: bool,
    /// Pass interrupts from a channel to a DMA IRQ line.
    pub irq_line: Option<InterruptLine>,
}

/// DMA peripheral interface.
pub struct DMA {
    channel_registers: StaticRef<[DMAChannel; 12]>,
    interrupt_registers: StaticRef<Interrupts>,
    configs: [OptionalCell<ChannelConfiguration>; 12],
}

impl DMA {
    /// Create a new DMA peripheral interface.
    ///
    /// Because this type contains state information about configured DMA channels,
    /// there should only ever be one instance of this type.
    pub unsafe fn new() -> DMA {
        DMA {
            channel_registers: DMA_CHANNELS,
            interrupt_registers: StaticRef::new(DMA_BASE_ADDRESS as *const Interrupts),
            configs: [OptionalCell::empty(),
                      OptionalCell::empty(),
                      OptionalCell::empty(),
                      OptionalCell::empty(),
                      OptionalCell::empty(),
                      OptionalCell::empty(),
                      OptionalCell::empty(),
                      OptionalCell::empty(),
                      OptionalCell::empty(),
                      OptionalCell::empty(),
                      OptionalCell::empty(),
                      OptionalCell::empty()],
        }
    }

    /// Enable DMA interrupts.
    pub fn enable_interrupt(&self, line: InterruptLine) {
        let irq_no = match line {
            InterruptLine::IRQ0 => crate::interrupts::DMA_IRQ_0,
            InterruptLine::IRQ1 => crate::interrupts::DMA_IRQ_1,
        };
        unsafe { Nvic::new(irq_no) }.enable();
    }

    /// Set up a DMA channel.
    ///
    /// Returns the number of the channel that was configured.
    pub fn configure(
        &self,
        client: &'static dyn DMAClient,
        buffer: &'static mut [usize],
        options: &ChannelOptions,
    ) -> Result<usize, ErrorCode>
    {
        for idx in 0..12 {
            if self.configs[idx].is_none() {
                self.configs[idx].set(
                    ChannelConfiguration {
                        buffer: TakeCell::new(buffer),
                        transfer_client: client,
                    }
                );

                if let Some(irq_line) = options.irq_line {
                    let mask = 1 << idx;
                    let inte = match irq_line {
                        InterruptLine::IRQ0 => &self.interrupt_registers.inte0,
                        InterruptLine::IRQ1 => &self.interrupt_registers.inte1,
                    };
                    inte.set(inte.get() | mask);
                }

                self.channel_registers[idx].read_addr.set(options.read_address);
                self.channel_registers[idx].write_addr.set(options.write_address);
                self.channel_registers[idx].trans_count.set(options.transfer_count);

                let (ring_sel, ring_size) = match options.ring_config {
                    TransferAddressWrap::None => (0, 0),
                    TransferAddressWrap::Read(size) => (0, size),
                    TransferAddressWrap::Write(size) => (1, size),
                };

                let ctrl =
                    CTRL::TREQ_SEL.val(options.treq_signal as u32).value
                    | CTRL::RING_SEL.val(ring_sel).value
                    | CTRL::RING_SIZE.val(ring_size as u32).value
                    | CTRL::INCR_WRITE.val(if options.increment_on_write { 1 } else { 0 }).value
                    | CTRL::INCR_READ.val(if options.increment_on_read { 1 } else { 0 }).value
                    | CTRL::DATA_SIZE.val(options.transfer_size as u32).value
                    | CTRL::HIGH_PRIORITY.val(if options.high_priority { 1 } else { 0 }).value
                    | 1;

                self.channel_registers[idx].ctrl.set(ctrl);

                return Ok(idx);
            }
        }

        Err(ErrorCode::BUSY)
    }
}
