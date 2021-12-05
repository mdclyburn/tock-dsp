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

    for irq in &mut IRQS {
        *irq = ignored_interrupt as usize;
    }
}

/// "Handler" for interrupts that are not being explicitly handled by ASPK.
#[allow(dead_code)]
pub unsafe extern "C" fn ignored_interrupt() {
    loop { cortexm0p::support::wfe(); }
}

/// "Handler" for interrupts that should be handled;
/// ignoring them is dangerous.
#[allow(dead_code)]
unsafe fn fail_interrupt() {
    loop { cortexm0p::support::wfe(); }
}
