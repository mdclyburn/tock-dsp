//! RP2040 Direct Memory Access peripheral.

use core::cell::Cell;

use kernel::ErrorCode;
use kernel::utilities::StaticRef;
use kernel::utilities::registers::{
    register_bitfields,
    register_structs,
    ReadOnly,
    ReadWrite,
    WriteOnly,
};
use kernel::utilities::registers::interfaces::{
    Readable,
    Writeable,
    ReadWriteable,
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
    }
}

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
];

const DMA_BASE_ADDRESS: usize = 0x5000_0000;
const DMA_CHANNELS: StaticRef<[DMAChannel; 12]> = unsafe {
    StaticRef::new(DMA_BASE_ADDRESS as *const [DMAChannel; 12])
};

pub struct DMA {
    registers: StaticRef<[DMAChannel; 12]>,
    state: Cell<u16>,
}

impl DMA {
    pub unsafe fn new() -> DMA {
        DMA {
            registers: DMA_CHANNELS,
            state: Cell::new(0),
        }
    }

    pub fn configure(&self) -> Result<(), ErrorCode> {
        for idx in (0..12) {
            let state = self.state.get();
            let busy_bit = 1 << idx;
            if (state & busy_bit) == 0 {
                self.state.set(state | busy_bit);
                return Ok(())
            }
        }

        Err(ErrorCode::BUSY)
    }
}
