//! RP2040 multi-core support.
//!
//! The RP2040 microcontroller contains two ARM Cortex-M0+ processors.
//! The two processors are referred to as core0 (or processor0) and core1 (or processor1)  in the documentation.
//! Code exported from the module is consistent with this scheme.
//! Support for working with the second core is offered by this module.

use crate::gpio::SIO;
use crate::psm::PowerOnStateMachine;

use cortexm0p::support;

/// Start core1 with a given vector table, stack location, and entry point.
///
/// At startup, core1 enters a low-power state and awaits a vector table,
/// a stack pointer, and an entry point from core0. This startup procedure
/// provides this data to core1 through the FIFO structure and SEV
/// instruction to synchronize with core0.
///
/// In case core1 is not already awaiting data from core0,
/// core1 is forced into reset and then forced out of reset.
///
/// This code is adapted from the Pico SDK code.
pub unsafe fn launch_core1(psm: &PowerOnStateMachine,
                           sio: &SIO,
                           vector_table_addr: usize,
                           stack_pointer: usize,
                           entry: usize)
{
    let commands = [0u32,
                    0u32,
                    1u32,
                    vector_table_addr as u32,
                    stack_pointer as u32,
                    (entry as u32) | 0b1];

    support::atomic(|| {
        // Reset core1 and wait for it to signal readiness through the FIFO.
        psm.reset_core1();
        while !sio.fifo_valid() {  }
        let _core1_ready_0 = sio.read_fifo();

        // Communicate data to core1.
        // Responses from core1 should always be the data last sent
        // from core0 through the FIFO. Otherwise, the two cores are
        // not in sync. Sending 0s causes core1's launch state machine
        // to restart.
        let mut seq = 0;
        while seq < commands.len() {
            let command = commands[seq];
            if command == 0 {
                // Drain the FIFO.
                while sio.fifo_valid() {
                    let _ = sio.read_fifo();
                }
                support::sev();
            }

            while !sio.fifo_ready() {  }
            sio.write_fifo(command);
            support::sev();

            while !sio.fifo_valid() { support::wfe(); }

            // Incorrect response; restart launch.
            if sio.read_fifo() != command {
                seq = 0;
            } else {
                seq += 1;
            }
        }
    });
}
