//! RP2040 Direct Memory Access peripheral.

use core::cell::Cell;

use kernel::ErrorCode;
use kernel::hil::dma::DMAClient;
use kernel::utilities::StaticRef;
use kernel::utilities::cells::{OptionalCell, TakeCell};
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
];

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

pub enum TransferAddressWrap {
    None,
    Read(u8),
    Write(u8),
}

#[derive(Copy, Clone)]
#[repr(u32)]
pub enum TransferSize {
    Byte = 0,
    HalfWord = 1,
    Word = 2,
}

const DMA_BASE_ADDRESS: usize = 0x5000_0000;
const DMA_CHANNELS: StaticRef<[DMAChannel; 12]> = unsafe {
    StaticRef::new(DMA_BASE_ADDRESS as *const [DMAChannel; 12])
};

struct ChannelConfiguration {
    buffer: TakeCell<'static, [usize]>,
    transfer_client: &'static dyn DMAClient,
}

pub struct DMA {
    registers: StaticRef<[DMAChannel; 12]>,
    configs: [OptionalCell<ChannelConfiguration>; 12],
}

impl DMA {
    pub unsafe fn new() -> DMA {
        DMA {
            registers: DMA_CHANNELS,
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

    pub fn configure(
        &self,
        client: &'static dyn DMAClient,
        buffer: &'static mut [usize],
        read_address: u32,
        write_address: u32,
        transfer_count: u32,
        treq: TransferRequestSignal,
        ring_config: TransferAddressWrap,
        increment_read: bool,
        increment_write: bool,
        transfer_size: TransferSize,
        high_priority: bool,
    ) -> Result<(), ErrorCode>
    {
        for idx in (0..12) {
            if self.configs[idx].is_none() {
                self.configs[idx].set(
                    ChannelConfiguration {
                        buffer: TakeCell::new(buffer),
                        transfer_client: client,
                    }
                );

                self.registers[idx].read_addr.set(read_address);
                self.registers[idx].write_addr.set(write_address);
                self.registers[idx].trans_count.set(transfer_count);

                let (ring_sel, ring_size) = match ring_config {
                    TransferAddressWrap::None => (0, 0),
                    TransferAddressWrap::Read(size) => (0, size),
                    TransferAddressWrap::Write(size) => (1, size),
                };

                let ctrl =
                    CTRL::TREQ_SEL.val(treq as u32).value
                    | CTRL::RING_SEL.val(ring_sel).value
                    | CTRL::RING_SIZE.val(ring_size as u32).value
                    | CTRL::INCR_WRITE.val(if increment_write { 1 } else { 0 }).value
                    | CTRL::INCR_READ.val(if increment_read { 1 } else { 0 }).value
                    | CTRL::DATA_SIZE.val(transfer_size as u32).value
                    | CTRL::HIGH_PRIORITY.val(if high_priority { 1 } else { 0 }).value
                    | 1;

                self.registers[idx].ctrl.set(ctrl);

                return Ok(());
            }
        }

        Err(ErrorCode::BUSY)
    }
}
