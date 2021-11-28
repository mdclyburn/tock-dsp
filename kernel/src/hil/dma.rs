//! Direct memory access.

use crate::errorcode::ErrorCode;
use crate::utilities::cells::TakeCell;

/// Size of each DMA transfer operation.
#[derive(Copy, Clone)]
#[repr(u32)]
pub enum TransferSize {
    /// One byte.
    Byte,
    /// Two bytes.
    HalfWord,
    /// Four bytes.
    Word,
}

#[derive(Copy, Clone)]
pub enum SourcePeripheral {
    /// Analog-to-digial converter.
    ADC,
}

#[derive(Copy, Clone)]
pub enum TargetPeripheral {  }

#[derive(Copy, Clone)]
pub enum TransferKind {
    /// Memory-to-memory transfer (e.g. RAM to RAM, ROM to RAM, etc).
    MemoryToMemory(usize, usize),
    /// Memory-to-peripheral (e.g., RAM to DAC).
    MemoryToPeripheral(usize, TargetPeripheral),
    /// Peripheral-to-memory.
    PeripheralToMemory(SourcePeripheral, usize),
}

/// Configurable parameters for DMA channels.
pub struct Parameters {
    /// Transfer source and target.
    pub kind: TransferKind,
    /// Number of transfers to perform.
    pub transfer_count: usize,
    /// Size of each transfer.
    pub transfer_size: TransferSize,
    /// Increase read address by `transfer_size` after each transfer.
    pub increment_on_read: bool,
    /// Increase write address by `transfer_size` after each transfer.
    pub increment_on_write: bool,
    /// Service this channel before non-high priority channels.
    pub high_priority: bool,
}

pub struct ChannelConfiguration {
    buffer: TakeCell<'static, [usize]>,
    transfer_client: Option<&'static dyn DMAClient>,
}

impl ChannelConfiguration {
    pub fn new(buffer: &'static mut [usize],
               client: Option<&'static dyn DMAClient>) -> ChannelConfiguration
    {
        ChannelConfiguration {
            buffer: TakeCell::new(buffer),
            transfer_client: client,
        }
    }
}

pub trait DMA {
    /// Configure and enable a DMA channel.
    fn configure(&self, params: &Parameters) -> Result<usize, ErrorCode>;

    /// Stop and disable a DMA channel.
    fn stop(&self, channel_no: usize) -> Result<(), ErrorCode>;
}

/// DMA-related callbacks.
pub trait DMAClient {
    fn transfer_done(&self, buffer: &'static mut [usize]);
}
