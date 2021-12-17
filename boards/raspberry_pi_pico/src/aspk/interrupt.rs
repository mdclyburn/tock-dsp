//! Interrupt servicing.

use cortexm0p;

/// Core1 vector addresses.
#[used]
#[link_section = ".core1_vectors"]
pub static mut VECTORS: [usize; 16] = [0x0000_0000; 16];

/// Core1 interrupt handlers
#[used]
#[link_section = ".core1_irqs"]
pub static mut IRQS: [usize; 32] = [0x0000_0000; 32];

/// Provide an initial configuration for all interrupts and exceptions.
pub unsafe fn configure() {
    // Initialize all vectors and IRQ handlers to something.
    // Skip the first two since that is the initial stack
    // pointer value and the reset vector.
    for (no, vec) in (0..).zip(&mut VECTORS[2..]) {
        *vec = match no {
            2 | 3 | 11 | 14 => fail_interrupt as usize,
            _ => ignored_interrupt as  usize,
        };
    };

    for (no, handler) in (0..).zip(&mut IRQS) {
        *handler = match no {
            // UART0, we let core0 handle servicing this.
            // However, if core1 sends something through the UART,
            // the HAL enables the interrupt, so we ignore it and disable it.
            20 => ignore_disable_interrupt_handler as usize,
            _ => fail_interrupt as usize,
        }
    }
}

/// "Handler" for interrupts that are not being explicitly handled by ASPK.
#[allow(dead_code)]
#[no_mangle]
pub unsafe extern "C" fn ignored_interrupt() {
    asm!(
        "mrs r0, ipsr",
    );

    loop { cortexm0p::support::wfe(); }
}

/// "Handler" for interrupts that should be handled;
/// ignoring them is dangerous.
#[allow(dead_code)]
#[no_mangle]
unsafe fn fail_interrupt() {
    asm!(
        "mrs r0, ipsr"
    );

    loop { cortexm0p::support::wfe(); }
}

/// A minimal, context-sensitive interrupt handler.
#[allow(dead_code)]
#[naked]
#[no_mangle]
unsafe extern "C" fn thin_cs_interrupt_handler() {
    asm!(
        "cpsid i",

        // Find out which interrupt this is.
        "mrs r0, ipsr",
        "movs r1, #0b11111",
        "ands r0, r1",
        "subs r0, #16", // r0 = interrupt number

        // Clear the interrupt.
        "movs r1, #1",
        "lsls r1, r0", // r1 = interrupt mask
        "ldr r2, BICER",
        "str r1, [r2]",

        // Now, decide what we do about this.
        // We decide based on where we came from.
        "ldr r0, PROG_EXC_RETURN",
        "cmp lr, r0",
        "bne __back_to_kernel_no_stack",

        // Stack r4, r5, r6, r7, r8, r9, r10, r11, r12
        "mov r2, sp",
        "movs r3, #36",
        "subs r0, r2, r3",
        "stm r0!, {{r4-r7}}",

        "mov r4, r8",
        "mov r5, r9",
        "mov r6, r10",
        "mov r7, r11",
        "stm r0!, {{r4-r7}}",
        "mov r4, r12",
        "push {{r4, r5}}",
        "mov sp, r0",

        "__back_to_kernel_no_stack:",
        "ldr r0, MAIN_EXC_RETURN",
        "mov lr, r0",

        "cpsie i",
        "bx lr",

        ".align 4",
        "BICER: .word 0xe000e180",
        "MAIN_EXC_RETURN: .word 0xfffffff9",
        "PROG_EXC_RETURN: .word 0xfffffffd",

        options(noreturn)
    )
}

#[allow(dead_code)]
#[naked]
#[no_mangle]
unsafe extern "C" fn ignore_disable_interrupt_handler() {
    asm!(
        "cpsid i",

        // Find out which interrupt number this is.
        "mrs r0, ipsr",
        "movs r1, #0b11111",
        "ands r0, r1",
        "subs r0, #16",
        // r0 = interrupt number

        // Clear the interrupt.
        // Get the mask for the interrupt.
        "movs r1, #1",
        "lsls r1, r0",
        // r1 = mask for the interrupt number
        // Write to ICPR to clear it.
        "ldr r2, ICPR",
        "str r1, [r2]",

        // Disable the interrupt.
        "ldr r2, ICER",
        "str r1, [r2]",

        // Return to whatever the system was doing prior to the interrupt.
        "bx lr",

        ".align 4",
        "ICPR: .word 0xe000e200",
        "ICER: .word 0xe000e180",

        options(noreturn)
    )
}
