//! Programmable I/O peripheral.

use core::cell::Cell;
use core::default::Default;

use cortexm0p::nvic::Nvic;

use kernel::errorcode::ErrorCode;
use kernel::utilities::cells::OptionalCell;
use kernel::utilities::registers::interfaces::{
    ReadWriteable,
    Readable,
    Writeable
};
use kernel::utilities::registers::{
    register_bitfields,
    register_structs,
    ReadOnly,
    ReadWrite,
    WriteOnly,
};
use kernel::utilities::StaticRef;

#[repr(C)]
struct StateMachineControl {
    clkdiv: ReadWrite<u32, SM_CLKDIV::Register>,
    execctrl: ReadWrite<u32, SM_EXECCTRL::Register>,
    shiftctrl: ReadWrite<u32, SM_SHIFTCTRL::Register>,
    addr: ReadOnly<u32, SM_ADDR::Register>,
    instr: ReadWrite<u32, SM_INSTR::Register>,
    pinctrl: ReadWrite<u32, SM_PINCTRL::Register>,
}

#[repr(C)]
struct InterruptControl {
    inte: ReadWrite<u32, INTx::Register>,
    intf: ReadWrite<u32, INTx::Register>,
    ints: ReadOnly<u32, INTx::Register>,
}

register_structs! {
    PIORegisters {
        // PIO control
        (0x000 => ctrl: ReadWrite<u32, CTRL::Register>),
        // FIFO status
        (0x004 => fstat: ReadOnly<u32, FSTAT::Register>),
        // FIFO debug
        (0x008 => fdebug: ReadWrite<u32, FDEBUG::Register>),
        // FIFO levels
        (0x00c => flevel: ReadOnly<u32, FLEVEL::Register>),
        // TX FIFO entries
        (0x010 => txf: [WriteOnly<u32, TXF::Register>; 4]),
        // RX FIFO entries
        (0x020 => rxf: [ReadOnly<u32, RXF::Register>; 4]),
        // IRQ
        (0x030 => irq: ReadWrite<u32, IRQ::Register>),
        // IRQ force
        (0x034 => irq_force: WriteOnly<u32, IRQ_FORCE::Register>),
        // Input sync bypass
        (0x038 => input_sync_bypass: ReadWrite<u32, INPUT_SYNC_BYPASS::Register>),
        // Debug pad output
        (0x03c => dbg_padout: ReadOnly<u32, DBG_PADOUT::Register>),
        // Debug pad output enable
        (0x040 => dbg_padoe: ReadOnly<u32, DBG_PADOE::Register>),
        // Debug configuration info
        (0x044 => dbg_cfginfo: ReadOnly<u32, DBG_CFGINFO::Register>),
        // Instruction memory
        (0x048 => instr_mem: [WriteOnly<u32, INSTR_MEM::Register>; 32]),
        // State machine control
        (0x0c8 => sm_ctrl: [StateMachineControl; 4]),
        // Raw interrupts
        (0x128 => intr: ReadOnly<u32, INTx::Register>),
        // Interrupt enable/force/status
        (0x12c => int: [InterruptControl; 2]),

        (0x144 => @END),
    }
}

register_bitfields! [
    u32,
    CTRL [
        CLKDIV_RESTART OFFSET(8) NUMBITS(4),
        SM_RESTART OFFSET(4) NUMBITS(4),
        SM_ENABLE OFFSET(0) NUMBITS(4),
    ],

    FSTAT [
        TXEMPTY OFFSET(24) NUMBITS(4),
        TXFULL OFFSET(16) NUMBITS(4),
        RXEMPTY OFFSET(8) NUMBITS(4),
        RXFULL OFFSET(0) NUMBITS(4),
    ],

    FDEBUG [
        TXSTALL OFFSET(24) NUMBITS(4),
        TXOVER OFFSET(16) NUMBITS(4),
        RXUNDER OFFSET(8) NUMBITS(4),
        RXSTALL OFFSET(0) NUMBITS(4),
    ],

    FLEVEL [
        RX3 OFFSET(28) NUMBITS(4),
        TX3 OFFSET(24) NUMBITS(4),
        RX2 OFFSET(20) NUMBITS(4),
        TX2 OFFSET(16) NUMBITS(4),
        RX1 OFFSET(12) NUMBITS(4),
        TX1 OFFSET(8) NUMBITS(4),
        RX0 OFFSET(4) NUMBITS(4),
        TX0 OFFSET(0) NUMBITS(4),
    ],

    TXF [
        ENTRY OFFSET(0) NUMBITS(32),
    ],

    RXF [
        ENTRY OFFSET(0) NUMBITS(32),
    ],

    IRQ [
        FLAGS OFFSET(0) NUMBITS(8),
    ],

    IRQ_FORCE [
        FLAGS OFFSET(0) NUMBITS(8),
    ],

    INPUT_SYNC_BYPASS [
        BITFIELD OFFSET(0) NUMBITS(32),
    ],

    DBG_PADOUT [
        BITFIELD OFFSET(0) NUMBITS(32),
    ],

    DBG_PADOE [
        BITFIELD OFFSET(0) NUMBITS(32),
    ],

    DBG_CFGINFO [
        IMEM_SIZE OFFSET(16) NUMBITS(6),
        SM_COUNT OFFSET(8) NUMBITS(4),
        FIFO_DEPTH OFFSET(0) NUMBITS(6),
    ],

    INSTR_MEM [
        INSTRUCTION OFFSET(0) NUMBITS(16),
    ],

    SM_CLKDIV [
        INT OFFSET(16) NUMBITS(16),
        FRAC OFFSET(8) NUMBITS(8),
    ],

    SM_EXECCTRL [
        EXEC_STALLED OFFSET(31) NUMBITS(1),
        SIDE_EN OFFSET(30) NUMBITS(1),
        SIDE_PINDIR OFFSET(29) NUMBITS(1),
        JMP_PIN OFFSET(24) NUMBITS(5),
        OUT_EN_SEL OFFSET(19) NUMBITS(5),
        INLINE_OUT_EN OFFSET(18) NUMBITS(1),
        OUT_STICKY OFFSET(17) NUMBITS(1),
        WRAP_TOP OFFSET(12) NUMBITS(5),
        WRAP_BOTTOM OFFSET(7) NUMBITS(5),
        STATUS_SEL OFFSET(4) NUMBITS(1),
        STATUS_N OFFSET(0) NUMBITS(4),
    ],

    SM_SHIFTCTRL [
        FJOIN_RX OFFSET(31) NUMBITS(1),
        FJOIN_TX OFFSET(30) NUMBITS(1),
        PULL_THRESH OFFSET(25) NUMBITS(5),
        PUSH_THRESH OFFSET(20) NUMBITS(5),
        OUT_SHIFTDIR OFFSET(19) NUMBITS(1),
        IN_SHIFTDIR OFFSET(18) NUMBITS(1),
        AUTOPULL OFFSET(17) NUMBITS(1),
        AUTOPUSH OFFSET(16) NUMBITS(1),
    ],

    SM_ADDR [
        ADDR OFFSET(0) NUMBITS(5),
    ],

    SM_INSTR [
        INSTR OFFSET(0) NUMBITS(16),
    ],

    SM_PINCTRL [
        SIDESET_COUNT OFFSET(29) NUMBITS(3),
        SET_COUNT OFFSET(26) NUMBITS(3),
        OUT_COUNT OFFSET(20) NUMBITS(6),
        IN_BASE OFFSET(15) NUMBITS(5),
        SIDESET_BASE OFFSET(10) NUMBITS(5),
        SET_BASE OFFSET(5) NUMBITS(5),
        OUT_BASE OFFSET(0) NUMBITS(5),
    ],

    INTx [
        SM3 OFFSET(11) NUMBITS(1),
        SM2 OFFSET(10) NUMBITS(1),
        SM1 OFFSET(9) NUMBITS(1),
        SM0 OFFSET(8) NUMBITS(1),
        SM3_TXNFULL OFFSET(7) NUMBITS(1),
        SM2_TXNFULL OFFSET(6) NUMBITS(1),
        SM1_TXNFULL OFFSET(5) NUMBITS(1),
        SM0_TXNFULL OFFSET(4) NUMBITS(1),
        SM3_RXNEMPTY OFFSET(3) NUMBITS(1),
        SM2_RXNEMPTY OFFSET(3) NUMBITS(1),
        SM1_RXNEMPTY OFFSET(3) NUMBITS(1),
        SM0_RXNEMPTY OFFSET(3) NUMBITS(1),
    ],
];

const PIO0_BASE_ADDRESS: usize = 0x5020_0000;
const PIO1_BASE_ADDRESS: usize = 0x5030_0000;

const PIO0: StaticRef<PIORegisters> = unsafe { StaticRef::new(PIO0_BASE_ADDRESS as *const PIORegisters) };
const PIO1: StaticRef<PIORegisters> = unsafe { StaticRef::new(PIO1_BASE_ADDRESS as *const PIORegisters) };

/// FIFO to use for the MOV x, STATUS instruction in [`Parameters`].
#[derive(Clone, Copy, PartialEq)]
pub enum StatusSelectFIFO {
    /// Use the transmit FIFO.
    Transmit,
    /// Use the receive FIFO.
    Receive,
}

/// How to allocate FIFO space for a state machine.
#[derive(Clone, Copy)]
pub enum FIFOAllocation {
    /// Transmit and receive FIFOs have the same size.
    Balanced,
    /// Transmit FIFO steals the receive FIFO's storage.
    Transmit,
    /// Receive FIFO steals the transmit FIFO's storage.
    Receive,
}

/// Direction to shift data entering/leaving shift registers.
#[derive(Clone, Copy)]
#[repr(u32)]
pub enum ShiftDirection {
    /// Shift data left.
    Left = 0,
    /// Shift data right.
    Right = 1,
}

/// Autopush/autopull configuration.
#[derive(Clone, Copy)]
pub enum Autoshift {
    /// Do not autopush/autopull.
    Off,
    /// Autopush/autopull the specified number of bits.
    On(u8),
}

/// Configuration for a PIO block.
#[derive(Clone, Copy)]
pub struct Parameters {
    // CLOCKDIV
    /// Clock divider for the state machine; 16-bit integer, 8-bit fractional.
    pub clock_divider: (u16, u8),

    // EXECCTRL
    /// Whether to use the MSB of delay/side-set field to make side-setting optional.
    ///
    /// Refer to SMx_EXECCTRL's SIDE_EN bitfield in the datasheet.
    pub side_set_enables: bool,
    /// Whether side-setting affects pin directions instead of pin values.
    pub side_set_pindir: bool,
    /// Number GPIO pin to use as a condition for JMP PIN.
    pub jmp_pin: u8,
    /// Which data bit to use for inline OUT enable.
    pub out_enable_bit: u8,
    /// Use a bit of OUT data as an auxiliary write enable.
    pub inline_out_enable: bool,
    /// Continuously assert the most recent OUT or SET to the pins.
    pub out_sticky: bool,
    /// Address to wrap to `wrap_bottom` from.
    pub wrap_top: u8,
    /// Address to wrap to when execution reaches `wrap_top`.
    pub wrap_bottom: u8,
    /// Comparison used for the MOV x, STATUS instruction.
    ///
    /// Refer to SMx_EXECCTRL's STATUS_SEL bitfield in the datasheet.
    pub status_source: StatusSelectFIFO,
    /// Comparison level for the MOV x, STATUS instruction.
    pub status_source_level: u8,

    // SHIFTCTRL
    /// How to allocate FIFO storage.
    pub fifo_allocation: FIFOAllocation,
    /// Direction to shift out from the OSR.
    pub osr_direction: ShiftDirection,
    /// Direction to shift into the ISR.
    pub isr_direction: ShiftDirection,
    /// Pull data into the output shift register upon reaching a threshold.
    pub autopull: Autoshift,
    /// Push data out of the input shift register upon reaching a threshold.
    pub autopush: Autoshift,

    // PINCTRL
    /// Number of MSBs to use for the for side-set (up to 5).
    pub side_set_count: u8,
    /// Number of pins to assert with a SET instruction.
    pub set_count: u8,
    /// Number of pins to assert with an OUT instruction.
    pub out_count: u8,
    /// First pin number mapped to the LSB of the IN data bus.
    pub in_base_pin: u8,
    /// First pin number mapped to the LSB of side-set data.
    pub side_set_base_pin: u8,
    /// First pin number mapped to the LSB of SET data.
    pub set_base_pin: u8,
    /// First pin number mapped to the LSB of OUT data.
    pub out_base_pin: u8,

    /// Initial pin direction configuration; a bit set means the pin is initially an output.
    pub initial_pindir: u8,
}

impl Default for Parameters {
    /// Produces the default configuration for a PIO block.
    ///
    /// The default values for parameters come from the RP2040 datasheet.
    /// These numbers are meant to be a safe default and not necessarily a usable default.
    fn default() -> Parameters {
        Parameters {
            clock_divider: (0, 0),
            side_set_enables: false,
            side_set_pindir: false,
            jmp_pin: 0,
            out_enable_bit: 0,
            inline_out_enable: false,
            out_sticky: false,
            wrap_top: 0x1f,
            wrap_bottom: 0x00,
            status_source: StatusSelectFIFO::Transmit,
            status_source_level: 0,
            fifo_allocation: FIFOAllocation::Balanced,
            osr_direction: ShiftDirection::Right,
            isr_direction: ShiftDirection::Right,
            autopull: Autoshift::Off,
            autopush: Autoshift::Off,
            side_set_count: 0,
            set_count: 5,
            out_count: 0,
            in_base_pin: 0,
            side_set_base_pin: 0,
            set_base_pin: 0,
            out_base_pin: 0,
            initial_pindir: 0b00000,
        }
    }
}

/// PIO block identifier.
///
/// The PIOs have four interrupt lines:
/// two lines for each of the two blocks.
/// PIO0 controls PIO0_IRQ0 and PIO0_IRQ1,
/// and PIO1 controls PIO1_IRQ0 and PIO1_IRQ1.
/// The pair of this type and [`InterruptLine`] identify one of the four interrupt lines.
#[derive(Clone, Copy)]
#[allow(non_camel_case_types)]
#[repr(u8)]
pub enum BlockID {
    PIO0 = 0,
    PIO1 = 1,
}

/// PIO IRQ line.
///
/// The PIOs have four interrupt lines:
/// two lines for each of the two blocks.
/// PIO0 controls PIO0_IRQ0 and PIO0_IRQ1,
/// and PIO1 controls PIO1_IRQ0 and PIO1_IRQ1.
/// The pair of this type and [`BlockID`] identify one of the four interrupt lines.
#[derive(Clone, Copy)]
#[allow(non_camel_case_types)]
#[repr(u8)]
pub enum InterruptLine {
    IRQ0 = 0,
    IRQ1 = 1,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// PIO state machine identifier.
pub enum StateMachine {
    /// State machine 0.
    SM0 = 0,
    /// State machine 1.
    SM1 = 1,
    /// State machine 2.
    SM2 = 2,
    /// State machine 3.
    SM3 = 3,
}

/// Interrupt context information.
#[derive(Clone, Copy)]
pub enum Interrupt {
    /// A state machine raised an interrupt.
    Flag(StateMachine),
    /// A state machine's transmission FIFO has available space.
    TransmitNotFull(StateMachine),
    /// A state machine's reception FIFO has new data available.
    ReceiveNotEmpty(StateMachine),
}

/// Handler to receive notice of a PIO block's events.
pub trait PIOBlockClient {
    /// Handler called by [`PIOBlock`] when the PIO peripheral raises an interrupt.
    fn interrupt_raised(&self, pio_block: &PIOBlock);
}

/// Binary-encoded SET.
const ENC_INSTR_SET: u16 = 0b111_00000_000_00000;

/// PIO peripheral instance.
pub struct PIOBlock {
    registers: StaticRef<PIORegisters>,
    client: OptionalCell<&'static dyn PIOBlockClient>,
}

impl PIOBlock {
    /// Set up the PIO block state machines.
    fn configure(&self,
                 instructions: &[u16],
                 sm_params: &[Option<&Parameters>; 4],
                 interrupt_when: &[Interrupt],
                 interrupt_on: InterruptLine)
    {
        // SM_ENABLE mask built by the loop.
        let mut enabled_machines = 0;
        for sm_no in 0..4 {
            enabled_machines >>= 1;
            if let Some(params) = sm_params[sm_no] {
                // CLKDIV register setup.
                let clkdiv =
                    SM_CLKDIV::INT.val(params.clock_divider.0 as u32).value
                    | SM_CLKDIV::FRAC.val(params.clock_divider.1 as u32).value;

                // EXECCTRL register setup.
                let execctrl =
                    SM_EXECCTRL::SIDE_EN.val(if params.side_set_enables { 1 } else { 0 }).value
                    | SM_EXECCTRL::SIDE_PINDIR.val(if params.side_set_pindir { 1 } else { 0 }).value
                    | SM_EXECCTRL::JMP_PIN.val(params.jmp_pin as u32).value
                    | SM_EXECCTRL::OUT_EN_SEL.val(params.out_enable_bit as u32).value
                    | SM_EXECCTRL::INLINE_OUT_EN.val(if params.inline_out_enable { 1 } else { 0 }).value
                    | SM_EXECCTRL::OUT_STICKY.val(if params.out_sticky { 1 } else { 0 }).value
                    | SM_EXECCTRL::WRAP_TOP.val(params.wrap_top as u32).value
                    | SM_EXECCTRL::WRAP_BOTTOM.val(params.wrap_bottom as u32).value
                    | SM_EXECCTRL::STATUS_SEL.val(if params.status_source == StatusSelectFIFO::Transmit { 0 } else { 1 }).value
                    | SM_EXECCTRL::STATUS_N.val(params.status_source_level as u32).value;

                // SHIFTCTRL
                let (fjoin_rx, fjoin_tx) = match params.fifo_allocation {
                    FIFOAllocation::Balanced => (0, 0),
                    FIFOAllocation::Receive => (1, 0),
                    FIFOAllocation::Transmit => (0, 1),
                };
                let (autopull, pull_threshold) = match params.autopull {
                    Autoshift::Off => (0, 0),
                    Autoshift::On(threshold) => (1, threshold),
                };
                let (autopush, push_threshold) = match params.autopush {
                    Autoshift::Off => (0, 0),
                    Autoshift::On(threshold) => (1, threshold),
                };

                let shiftctrl =
                    SM_SHIFTCTRL::FJOIN_RX.val(fjoin_rx).value
                    | SM_SHIFTCTRL::FJOIN_TX.val(fjoin_tx).value
                    | SM_SHIFTCTRL::PULL_THRESH.val(pull_threshold as u32).value
                    | SM_SHIFTCTRL::PUSH_THRESH.val(push_threshold as u32).value
                    | SM_SHIFTCTRL::OUT_SHIFTDIR.val(params.osr_direction as u32).value
                    | SM_SHIFTCTRL::IN_SHIFTDIR.val(params.isr_direction as u32).value
                    | SM_SHIFTCTRL::AUTOPULL.val(autopull).value
                    | SM_SHIFTCTRL::AUTOPUSH.val(autopush).value;

                let pinctrl =
                    SM_PINCTRL::SIDESET_COUNT.val(params.side_set_count as u32).value
                    | SM_PINCTRL::SET_COUNT.val(params.set_count as u32).value
                    | SM_PINCTRL::OUT_COUNT.val(params.out_count as u32).value
                    | SM_PINCTRL::IN_BASE.val(params.in_base_pin as u32).value
                    | SM_PINCTRL::SIDESET_BASE.val(params.side_set_base_pin as u32).value
                    | SM_PINCTRL::SET_BASE.val(params.set_base_pin as u32).value
                    | SM_PINCTRL::OUT_BASE.val(params.out_base_pin as u32).value;

                // Apply settings to registers.
                self.registers.sm_ctrl[sm_no].clkdiv.set(clkdiv);
                self.registers.sm_ctrl[sm_no].execctrl.set(execctrl);
                self.registers.sm_ctrl[sm_no].shiftctrl.set(shiftctrl);
                self.registers.sm_ctrl[sm_no].pinctrl.set(pinctrl);

                // Configure the initial pin directions.
                let initial_pindirs_instr =
                    ENC_INSTR_SET
                    | 0b000_00000_100_00000 // Set PINDIRS.
                    | params.initial_pindir as u16;
                self.registers.sm_ctrl[sm_no].instr.set(initial_pindirs_instr as u32);

                enabled_machines |= (1 << 3);
            }
        }

        // Write instructions to instruction memory.
        for (idx, instr) in (0..32).zip(instructions) {
            self.registers.instr_mem[idx].set(*instr as u32)
        }

        // Enable the requested interrupt reasons.
        let mut inte = 0;
        for reason in interrupt_when {
            inte |= match reason {
                Interrupt::Flag(sm_id) => 1 << (8 + *sm_id as u8),
                Interrupt::TransmitNotFull(sm_id) => 1 << (4 + *sm_id as u8),
                Interrupt::ReceiveNotEmpty(sm_id) => 1 << (0 + *sm_id as u8),
            };
        }

        self.registers.int[interrupt_on as usize].inte.set(inte);

        // Enable the state machines that have configurations.
        self.registers.ctrl.modify(CTRL::SM_ENABLE.val(enabled_machines));
    }

    /// Returns the reason(s) for an interrupt.
    ///
    /// A client may call this function multiple times until it returns `None`.
    pub fn next_pending(&self) -> Option<Interrupt> {
        unimplemented!()
    }

    /// Interrupt handler for a specific PIO block.
    fn handle_interrupt(&self, irq_line: InterruptLine) {
        unimplemented!()
    }
}

/// PIO instance.
pub struct PIO {
    allocated: Cell<u8>,
    pio_blocks: [PIOBlock; 2],
}

impl PIO {
    /// Create a new PIO interface.
    pub const fn new() -> PIO {
        PIO {
            allocated: Cell::new(0),
            pio_blocks: [
                PIOBlock {
                    registers: PIO0,
                    client: OptionalCell::empty(),
                },
                PIOBlock {
                    registers: PIO0,
                    client: OptionalCell::empty(),
                }
            ],
        }
    }

    /// Handle a PIO interrupt.
    ///
    /// Passes on interrupt information to the PIO block software context for handling
    /// along with the interrupt line that is responsible for the interrupt.
    pub fn handle_interrupt(&self, interrupting_block: BlockID, irq_line: InterruptLine) {
        self.pio_blocks[interrupting_block as usize].handle_interrupt(irq_line);
    }

    /// Configure a PIO block.
    pub fn configure(&'static self,
                     instructions: &[u16],
                     params: &[Option<&Parameters>; 4],
                     interrupt_when: &[Interrupt],
                     interrupt_on: InterruptLine) -> Result<&'static PIOBlock, ErrorCode>
    {
        let block_no = self.allocated.get();
        if block_no as usize >= self.pio_blocks.len() {
            Err(ErrorCode::BUSY)
        } else {
            let pio_block_idx = self.allocated.get();
            let pio_block = &self.pio_blocks[pio_block_idx as usize];
            pio_block.configure(instructions, params, interrupt_when, interrupt_on);
            self.allocated.set(pio_block_idx+1);
            Ok(pio_block)
        }
    }
}
