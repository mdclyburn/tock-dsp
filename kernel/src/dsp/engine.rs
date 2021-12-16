//! Signal processing core logic.

use core::cell::Cell;
use core::iter::Iterator;

use crate::config;
use crate::dsp::buffer::{AudioBuffer, BufferState};
use crate::dsp::component;
use crate::dsp::link::{Link, LinkIt};
use crate::hil::adc::Adc;
use crate::hil::dma::{self, DMA, DMAChannel};
use crate::platform::KernelResources;
use crate::platform::chip::Chip;
use crate::syscall::UserspaceKernelBoundary;

pub struct DSPEngine {
    /// Processing chain.
    chain: &'static dyn Link,
    /// Chain processor stack pointer.
    processing_stack: *const u8,
    /// Buffers for incoming audio samples.
    in_buffers: [AudioBuffer; config::SAMPLE_BUFFERS],
    /// Buffers for outgoing audio samples.
    out_buffers: [AudioBuffer; config::SAMPLE_BUFFERS],
}

impl DSPEngine {
    /// Create a new `DSPEngine` instance.
    pub unsafe fn new(chain: &'static dyn Link, processing_stack: *const u8) -> DSPEngine {
        DSPEngine {
            chain,
            processing_stack,
            in_buffers: [AudioBuffer::new(),
                         AudioBuffer::new(),
                         AudioBuffer::new()],
            out_buffers: [AudioBuffer::new(),
                          AudioBuffer::new(),
                          AudioBuffer::new()],
        }
    }

    /// Returns the sampling rate of the engine.
    pub fn sampling_rate() -> usize { config::SAMPLING_RATE }

    /// Run the digital signal processing loop.
    ///
    /// Performs initial configuration and starts the DSP loop.
    /// Once configured, the DSP side of the kernel will respond to a short list of interrupts:
    /// SIO, for interprocessor messaging;
    /// DMA, for sample processing and output.
    pub fn run<C: Chip, R: KernelResources<C>>(
        &'static self,
        chip: &C,
        resources: &R,
        dma: &'static dyn DMA,
    ) -> !
    {
        // Configure ADC sampling flow.
        let adc_dma_channel = {
            let params = dma::Parameters {
                kind: dma::TransferKind::PeripheralToMemory(dma::SourcePeripheral::ADC, 0),
                transfer_count: config::NO_SAMPLES,
                // Need to optimize this to allow transferring a halfword.
                transfer_size: dma::TransferSize::Word,
                increment_on_read: false,
                increment_on_write: true,
                high_priority: true,
            };
            dma.configure(&params).expect("failed configuring DMA channel")
        };
        adc_dma_channel.set_client(self);
        // Configure output.
        // TODO: implement.

        // Prepare a new context for the processing chain.
        let mut processing_loop_context = {
            let processing_loop_fn = component::configure_loop(self.chain, &self.in_buffers, &self.out_buffers);
            unsafe {
                chip.userspace_kernel_boundary()
                    .create_context(self.processing_stack, processing_loop_fn)
            }
        }.unwrap();

        // Create cyclic iterators over the audio buffers.
        let mut input_buffer_it = self.in_buffers.iter().cycle();
        let mut output_buffer_it = self.out_buffers.iter().cycle();
        let mut current_input_buffer = input_buffer_it.next()
            // cyclic iterator over a slice; shouldn't fail
            .unwrap();
        let mut _current_output_buffer = output_buffer_it.next()
            // cyclic iterator over a slice; shouldn't fail
            .unwrap();

        // Start sample collection.
        adc_dma_channel.start(current_input_buffer.take(BufferState::Collecting).unwrap())
            .expect("failed to start ADC sampling DMA");

        loop {
            // Switch to the processing loop.
            // chip.userspace_kernel_boundary()
            //     .switch_to_context(&mut processing_loop_context);

            // Find out why the context changed back to the engine loop.

            if chip.has_pending_interrupts() {
                chip.service_pending_interrupts();
            }
        }
    }
}

impl dma::DMAClient for DSPEngine {
    fn transfer_done(&self, channel_no: usize, buffer: &'static mut [usize]) {
        loop {  }
    }
}
