//! Interrupt servicing specially crafted for ASPK.

use cortexm0p;
use rp2040::dma::{self, DMA};

/// Core1 vector addresses.
#[used]
#[link_section = ".core1_vectors"]
pub static mut VECTORS: [usize; 16] = [0x0000_0000; 16];

/// Core1 interrupt handlers
#[used]
#[link_section = ".core1_irqs"]
pub static mut IRQS: [usize; 32] = [0x0000_0000; 32];

static mut DMA: Option<&'static DMA> = None;

/// Provide an initial configuration for all interrupts and exceptions.
pub unsafe fn configure(dma: &'static DMA) {
    // Initialize all vectors and IRQ handlers to something.
    // Skip the first two since that is the initial stack
    // pointer value and the reset vector.
    for (no, vec) in (0..).zip(&mut VECTORS[2..]) {
        *vec = match no {
            2 | 3 | 11 | 14 => fail_interrupt as usize,
            _ => ignored_interrupt as  usize,
        };
    };

    // Set up the specially-handled interrupts.
    DMA = Some(dma);

    for (no, handler) in (0..).zip(&mut IRQS) {
        *handler = match no {
            // DMA0
            11 => dma_interrupt_handler as usize,
            12 => dma_interrupt_handler as usize,
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

/// Handler to service DMA immediately.
#[allow(dead_code)]
#[no_mangle]
unsafe extern "C" fn dma_interrupt_handler() {
    let dma_irq_no: usize;
    asm!(
        // Disable interrupts.
        "cpsid i",

        // Get the interrupt number.
        // We need to see whether this was DMA IRQ 0 or 1.
        "mrs r0, ipsr",
        "subs r0, #16",

        out("r0") dma_irq_no
    );
    let dma_irq_line = if dma_irq_no == 11 {
        dma::InterruptLine::IRQ0
    } else {
        dma::InterruptLine::IRQ1
    };
    DMA.unwrap().handle_interrupt(dma_irq_line);

    asm!("cpsie i");
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
