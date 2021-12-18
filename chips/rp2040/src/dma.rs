//! RP2040 Direct Memory Access peripheral.

use cortexm0p::nvic::Nvic;
use kernel::ErrorCode;
use kernel::hil;
use kernel::utilities::StaticRef;
use kernel::utilities::cells::{MapCell, OptionalCell, TakeCell};
use kernel::utilities::registers::{
    register_bitfields,
    register_structs,
    ReadOnly,
    ReadWrite,
};
use kernel::utilities::registers::interfaces::{
    Readable,
    Writeable,
    ReadWriteable,
};

/// Number of DMA channels on the RP2040.
const NO_CHANNELS: usize = 12;

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
        (0x000 => intr: ReadOnly<u32, ()>),
        /// Interrupt enablement for DMA_IRQ0.
        (0x004 => inte0: ReadWrite<u32, ()>),
        /// Force interrupts.
        (0x008 => intf0: ReadWrite<u32, ()>),
        /// Interrupt status for DMA_IRQ0.
        (0x00c => ints0: ReadWrite<u32, ()>),

        (0x010 => _reserved0),

        /// Interrupt enablement for DMA_IRQ1.
        (0x014 => inte1: ReadWrite<u32, ()>),
        /// Force interrupts.
        (0x018 => intf1: ReadWrite<u32, ()>),
        /// Interrupt status for DMA_IRQ0.
        (0x01c => ints1: ReadWrite<u32, ()>),

        (0x020 => @END),
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
const DMA_INTERRUPTS_ADDRESS: usize = 0x5000_0400;

const DMA_CHANNELS: StaticRef<[DMAChannel; 12]> = unsafe {
    StaticRef::new(DMA_BASE_ADDRESS as *const [DMAChannel; 12])
};

/// Single, configured DMA channel.
pub struct Channel {
    channel_no: usize,
    buffer: TakeCell<'static, [usize]>,
    config: MapCell<hil::dma::Parameters>,
    client: OptionalCell<&'static dyn hil::dma::DMAClient>,
}

impl Channel {
    const fn unconfigured(channel_no: usize) -> Channel {
        Channel {
            channel_no,
            buffer: TakeCell::empty(),
            config: MapCell::empty(),
            client: OptionalCell::empty(),
        }
    }

    fn handle_interrupt(&self) {
        if let Some(client) = self.client.extract() {
            if let Some(buffer) = self.buffer.take() {
                client.transfer_done(self, buffer);
            } else {
                panic!("Interrupt handler called but no there was no buffer.");
            }
        }
    }
}

impl hil::dma::DMAChannel for Channel {
    fn channel_no(&self) -> usize {
        self.channel_no
    }

    fn start(&self, buffer: &'static mut [usize]) -> Result<(), ErrorCode> {
        // Get:
        // - amount of data to the channel will transfer.
        // - transfer type that the channel will perform.
        let (len_required, kind) = self.config.map(|c| {
            (c.transfer_count * c.transfer_size as usize,
             c.kind)
        }).unwrap(); // It should not be possible to have a DMA channel that is unconfigured.
        // Number of bytes provided through `buffer`.
        let len_provided = buffer.len() * 4;

        // If we have a buffer here, the channel is busy reading from/writing to it.
        // Check the buffer size to ensure the amount of data we'll be transferring from/to
        // it will not exceed the buffer's size.
        if self.buffer.is_some() {
            Err(ErrorCode::BUSY)
        } else if len_provided < len_required {
            Err(ErrorCode::NOMEM)
        } else {
            // We only support memory-peripheral or peripheral-memory transfers here for now.
            // For memory-memory, we cannot discern whether this new buffer replaces
            // the source buffer or the destination buffer.
            use hil::dma::TransferKind::*;
            let addr = buffer.as_ptr();
            let channel_register = &DMA_CHANNELS[self.channel_no];
            match kind {
                MemoryToPeripheral(_ra, _p) => channel_register.read_addr.set(addr as u32),
                PeripheralToMemory(_p, _wa) => channel_register.write_addr.set(addr as u32),
                _ => return Err(ErrorCode::NOSUPPORT)
            };

            self.buffer.put(Some(buffer));
            channel_register.ctrl.modify(CTRL::EN::Enable);

            Ok(())
        }
    }

    fn poll(&self) -> Option<&'static mut [usize]> {
        let done = DMA_CHANNELS[self.channel_no].trans_count.get() == 0;
        if done {
            self.buffer.take()
        } else {
            None
        }
    }

    fn set_client(&self, client: &'static dyn hil::dma::DMAClient) {
        self.client.set(client)
    }
}

/// DMA peripheral interface.
pub struct DMA {
    channel_registers: StaticRef<[DMAChannel; 12]>,
    interrupt_registers: StaticRef<Interrupts>,
    configs: [Channel; 12],
}

impl DMA {
    /// Create a new DMA peripheral interface.
    ///
    /// Because this type contains state information about configured DMA channels,
    /// there should only ever be one instance of this type.
    pub const fn new() -> DMA {
        DMA {
            channel_registers: DMA_CHANNELS,
            interrupt_registers: unsafe { StaticRef::new((DMA_INTERRUPTS_ADDRESS) as *const Interrupts) },
            configs: [Channel::unconfigured(0),
                      Channel::unconfigured(1),
                      Channel::unconfigured(2),
                      Channel::unconfigured(3),
                      Channel::unconfigured(4),
                      Channel::unconfigured(5),
                      Channel::unconfigured(6),
                      Channel::unconfigured(7),
                      Channel::unconfigured(8),
                      Channel::unconfigured(9),
                      Channel::unconfigured(10),
                      Channel::unconfigured(11)],
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

    /// Handle a DMA interrupt.
    pub fn handle_interrupt(&self, interrupt_no: InterruptLine) {
        // Find out which channel(s) is responsible for the interrupt.
        let ints = match interrupt_no {
            InterruptLine::IRQ0 => &self.interrupt_registers.ints0,
            InterruptLine::IRQ1 => &self.interrupt_registers.ints1,
        };
        let status = ints.get();

        for channel_idx in 0..NO_CHANNELS {
            if (1 << channel_idx) & status == 1 {
                self.configs[channel_idx].handle_interrupt();
            }
        }

        // Clear channel interrupt status.
        ints.set(status);
    }

    /// Set up a DMA channel.
    ///
    /// Returns the number of the channel that was configured.
    pub fn configure(
        &'static self,
        options: &hil::dma::Parameters,
    ) -> Result<&'static dyn hil::dma::DMAChannel, ErrorCode>
    {
        for idx in 0..12 {
            if self.configs[idx].config.is_none() {
                let mask = 1 << idx;
                self.interrupt_registers.inte0.set(self.interrupt_registers.inte0.get() | mask);
                self.interrupt_registers.inte1.set(self.interrupt_registers.inte1.get() | mask);

                let (read_addr, write_addr) = {
                    use hil::dma::TransferKind;

                    match options.kind {
                        TransferKind::MemoryToMemory(ra, wa) => (ra, wa),
                        TransferKind::MemoryToPeripheral(_ra, _p) => unimplemented!(),
                        TransferKind::PeripheralToMemory(p, wa) => (peripheral_source_address(p), wa),
                    }
                };

                self.channel_registers[idx].read_addr.set(read_addr as u32);
                self.channel_registers[idx].write_addr.set(write_addr as u32);
                self.channel_registers[idx].trans_count.set(options.transfer_count as u32);

                // No support for wrapping in HIL.
                let (ring_sel, ring_size) = (0, 0);

                // Translate a peripheral source to an RP2040 DMA TREQ signal source.
                let treq_signal = {
                    use hil::dma::TransferKind;
                    use hil::dma::SourcePeripheral;

                    if let TransferKind::PeripheralToMemory(source_peripheral, _write_addr) = options.kind {
                        match source_peripheral {
                            SourcePeripheral::ADC => TransferRequestSignal::ADC as u32,
                        }
                    } else {
                        // Transfer proceeds as fast as possible.
                        TransferRequestSignal::PERMANENT_REQUEST as u32
                    }
                };

                let ctrl =
                    CTRL::TREQ_SEL.val(treq_signal as u32).value
                    | CTRL::RING_SEL.val(ring_sel).value
                    | CTRL::RING_SIZE.val(ring_size as u32).value
                    | CTRL::INCR_WRITE.val(if options.increment_on_write { 1 } else { 0 }).value
                    | CTRL::INCR_READ.val(if options.increment_on_read { 1 } else { 0 }).value
                    | CTRL::DATA_SIZE.val(options.transfer_size as u32).value
                    | CTRL::HIGH_PRIORITY.val(if options.high_priority { 1 } else { 0 }).value;

                // Hard-code to use DMA_IRQ0 since HIL does not have an equivalent distinction.
                self.enable_interrupt(InterruptLine::IRQ0);

                self.channel_registers[idx].ctrl.set(ctrl);

                self.configs[idx].config.put(*options);

                return Ok(&self.configs[idx]);
            }
        }

        Err(ErrorCode::BUSY)
    }
}

impl hil::dma::DMA for DMA {
    fn configure(&'static self,
                 params: &hil::dma::Parameters)
                 -> Result<&'static dyn hil::dma::DMAChannel, ErrorCode>
    {
        self.configure(params)
    }

    fn stop(&'static self, _channel_no: usize) -> Result<(), ErrorCode> {
        Err(ErrorCode::NOSUPPORT)
    }
}

const fn peripheral_source_address(p: hil::dma::SourcePeripheral) -> usize {
    match p {
        hil::dma::SourcePeripheral::ADC => 0x4004_C00C,
    }
}
