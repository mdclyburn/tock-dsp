use cortexm0p::nvic::Nvic;

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
    execctrl: ReadWrite<u32, SM_EXECCTRL::Register>,
    shiftctrl: ReadWrite<u32, SM_SHIFTCTRL::Register>,
    addr: ReadOnly<u32, SM_ADDR::Register>,
    instr: ReadWrite<u32, SM_INSTR::Register>,
    pinctrl: ReadWrite<u32, SM_PINCTRL::Register>,
}

#[repr(C)]
struct InterruptControl {
    intr: ReadOnly<u32, INTx::Register>,
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
        // Interrupts
        (0x128 => int: [InterruptControl; 2]),

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

pub struct PIO {
}
