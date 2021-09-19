//! Control power throughout the RP2040 with the power-on state machine.

use kernel::utilities::registers::{
    interfaces::{
        Readable,
        ReadWriteable
    },
    register_bitfields,
    register_structs,
    ReadOnly,
    ReadWrite
};
use kernel::utilities::StaticRef;

register_structs! {
    PSMRegisters {
        (0x00 => force_on: ReadWrite<u32, FRCE_ON::Register>),
        (0x04 => force_off: ReadWrite<u32, FRCE_OFF::Register>),
        (0x08 => wdsel: ReadWrite<u32, WDSEL::Register>),
        (0x0C => done: ReadOnly<u32, DONE::Register>),

        (0x10 => @END),
    }
}

register_bitfields! [
    u32,
    FRCE_ON [
        PROC1 OFFSET(16) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        PROC0 OFFSET(15) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        SIO OFFSET(14) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        VREG_AND_CHIP_RESET OFFSET(13) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        XIP OFFSET(12) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        SRAM5 OFFSET(11) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        SRAM4 OFFSET(10) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        SRAM3 OFFSET(9) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        SRAM2 OFFSET(8) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        SRAM1 OFFSET(7) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        SRAM0 OFFSET(6) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        ROM OFFSET(5) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        BUSFABRIC OFFSET(4) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        RESETS OFFSET(3) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        CLOCKS OFFSET(2) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        XOSC OFFSET(1) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        ROSC OFFSET(0) NUMBITS(1) [
            Force = 1,
            Release = 0
        ]
    ],

    FRCE_OFF [
        PROC1 OFFSET(16) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        PROC0 OFFSET(15) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        SIO OFFSET(14) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        VREG_AND_CHIP_RESET OFFSET(13) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        XIP OFFSET(12) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        SRAM5 OFFSET(11) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        SRAM4 OFFSET(10) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        SRAM3 OFFSET(9) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        SRAM2 OFFSET(8) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        SRAM1 OFFSET(7) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        SRAM0 OFFSET(6) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        ROM OFFSET(5) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        BUSFABRIC OFFSET(4) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        RESETS OFFSET(3) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        CLOCKS OFFSET(2) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        XOSC OFFSET(1) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        ROSC OFFSET(0) NUMBITS(1) [
            Force = 1,
            Release = 0
        ]
    ],

    WDSEL [
        PROC1 OFFSET(16) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        PROC0 OFFSET(15) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        SIO OFFSET(14) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        VREG_AND_CHIP_RESET OFFSET(13) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        XIP OFFSET(12) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        SRAM5 OFFSET(11) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        SRAM4 OFFSET(10) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        SRAM3 OFFSET(9) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        SRAM2 OFFSET(8) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        SRAM1 OFFSET(7) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        SRAM0 OFFSET(6) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        ROM OFFSET(5) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        BUSFABRIC OFFSET(4) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        RESETS OFFSET(3) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        CLOCKS OFFSET(2) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        XOSC OFFSET(1) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        ROSC OFFSET(0) NUMBITS(1) [
            Force = 1,
            Release = 0
        ]
    ],

    DONE [
        PROC1 OFFSET(16) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        PROC0 OFFSET(15) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        SIO OFFSET(14) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        VREG_AND_CHIP_RESET OFFSET(13) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        XIP OFFSET(12) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        SRAM5 OFFSET(11) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        SRAM4 OFFSET(10) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        SRAM3 OFFSET(9) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        SRAM2 OFFSET(8) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        SRAM1 OFFSET(7) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        SRAM0 OFFSET(6) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        ROM OFFSET(5) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        BUSFABRIC OFFSET(4) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        RESETS OFFSET(3) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        CLOCKS OFFSET(2) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        XOSC OFFSET(1) NUMBITS(1) [
            Force = 1,
            Release = 0
        ],
        ROSC OFFSET(0) NUMBITS(1) [
            Force = 1,
            Release = 0
        ]
    ],
];

const PSM_BASE_ADDRESS: usize = 0x40010000;
const PSM: StaticRef<PSMRegisters> = unsafe {
    StaticRef::new(PSM_BASE_ADDRESS as *const PSMRegisters)
};

pub struct PowerOnStateMachine {
    registers: StaticRef<PSMRegisters>,
}

impl PowerOnStateMachine {
    pub const fn new() -> PowerOnStateMachine {
        PowerOnStateMachine {
            registers: PSM,
        }
    }

    /// Power cycle core1.
    #[inline]
    pub fn reset_core1(&self) {
        self.registers.force_off.modify(FRCE_OFF::PROC1::Force);
        while self.registers.force_off.read(FRCE_OFF::PROC1) != 1 {  }
        self.registers.force_off.modify(FRCE_OFF::PROC1::Release);
    }
}
