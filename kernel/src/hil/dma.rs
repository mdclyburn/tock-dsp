//! Direct memory access.

/// DMA-related callbacks.
pub trait DMAClient {
    fn transfer_done(&self, buffer: &'static mut [usize]);
}
