//! Signal processing core logic.

use core::cell::Cell;
use core::iter::{Cycle, Iterator, Peekable};
use core::slice::Iter as SliceIter;

use crate::config;
use crate::debug;
use crate::dsp::buffer::{AudioBuffer, BufferState};
use crate::dsp::link::Chain;
use crate::errorcode::ErrorCode;
use crate::hil::dma::{self, DMA, DMAChannel};
use crate::hil::time::{self, Time, ConvertTicks};
use crate::platform::KernelResources;
use crate::platform::chip::Chip;
use crate::utilities::cells::MapCell;

type CyclicBufferIter = Peekable<Cycle<SliceIter<'static, AudioBuffer>>>;

pub struct DSPEngine {
    /// Buffers for incoming audio samples.
    in_buffers: [AudioBuffer; config::SAMPLE_BUFFERS],
    /// Buffers for outgoing audio samples.
    out_buffers: [AudioBuffer; config::SAMPLE_BUFFERS],
    /// ADC DMA channel number.
    adc_dma_channel_no: Cell<u8>,
    /// Cyclical input buffer iterator.
    in_buffer_iter: MapCell<CyclicBufferIter>,
}

impl DSPEngine {
    /// Create a new `DSPEngine` instance.
    pub unsafe fn new() -> DSPEngine {
        DSPEngine {
            in_buffers: [AudioBuffer::new(),
                         AudioBuffer::new(),
                         AudioBuffer::new()],
            out_buffers: [AudioBuffer::new(),
                          AudioBuffer::new(),
                          AudioBuffer::new()],
            adc_dma_channel_no: Cell::new(99),
            in_buffer_iter: MapCell::empty(),
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
    pub fn run<C: Chip,
               R: KernelResources<C>,
               F: time::Frequency,
               T: time::Ticks>(
        &'static self,
        _chip: &C,
        _resources: &R,
        dma: &'static dyn DMA,
        time: &dyn Time<Frequency = F, Ticks = T>,
        chain: &Chain,
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
        self.adc_dma_channel_no.set(adc_dma_channel.channel_no() as u8);
        // Configure output.
        // TODO: implement.

        // Create cyclic iterators over the audio buffers.
        let mut process_in_buffer_it = self.in_buffers.iter().cycle();
        let mut process_out_buffer_it = self.out_buffers.iter().cycle();

        // Start sample collection.
        self.initiate_sampling(adc_dma_channel)
            .expect("failed to start ADC sampling DMA");

        let mut t_post = 1;
        // Depending on the length of the signal chain and the processing strategy,
        // the samples go back and forth between buffers as we go through the chain.
        // If there are an even number of processors in the chain,
        // then we must bounce buffer ownership between the AudioBuffers to avoid copying samples.
        let exchange_buffers_after_processing = {
            let link_count = chain.into_iter().count();
            link_count % 2 == 0
        };
        loop {
            // Obtain the next unprocessed sequence of audio samples.
            // Obtain the next free output buffer for processed samples.
            let input_buffer = process_in_buffer_it.next().unwrap();
            let output_buffer = process_out_buffer_it.next().unwrap();
            while input_buffer.state() != BufferState::Unprocessed {  }
            while output_buffer.state() != BufferState::Free {  }

            // Start timing the DSP loop.
            let loop_start = time.ticks_to_us(time.now());

            // Grab the buffers.
            let (proc_buf_a, proc_buf_b) = (input_buffer.take(BufferState::Processing)
                                            .expect("buffer for unprocessed input missing"),
                                            output_buffer.take(BufferState::Processing)
                                            .expect("buffer for processed output missing"));

            // Iterate through all links in the chain and run their processors.
            // Input samples buffer → signal processor → output samples buffer.
            for (link_no, link) in (0..).zip(chain) {
                if link_no % 2 == 0 {
                    link.processor().process(&*proc_buf_a, proc_buf_b);
                } else {
                    link.processor().process(&*proc_buf_b, proc_buf_a);
                }
            }

            // Stop timing the DSP loop.
            let loop_end = time.ticks_to_us(time.now());
            if t_post * 500_000 < loop_end {
                t_post += 1;
                debug!("DSP loop latency: {}μs", loop_end - loop_start);
            }

            // Replace the buffers.
            // The output buffer needs to go to BufferState::Ready when we implement playing processed samples.
            if exchange_buffers_after_processing {
                input_buffer.put(proc_buf_a, BufferState::Free);
                output_buffer.put(proc_buf_b, BufferState::Free);
            } else {
                input_buffer.put(proc_buf_b, BufferState::Free);
                output_buffer.put(proc_buf_a, BufferState::Free);
            }
        }
    }

    fn initiate_sampling(&'static self, dma_channel: &dyn DMAChannel) -> Result<(), ErrorCode> {
        let buffer_iter = self.in_buffers.iter().cycle().peekable();
        self.in_buffer_iter.put(buffer_iter);
        let buffer = self.in_buffer_iter
            .map(|iter| iter.peek().unwrap().take(BufferState::Collecting).unwrap())
            .unwrap();

        dma_channel.start(buffer)
    }
}

impl dma::DMAClient for DSPEngine {
    /// Restore incoming sample buffer and restart a transfer.
    ///
    /// The DSP engine receives this callback for completed transfers from the ADC and to the I2S-PIO.
    /// In both cases, we replace the buffer to its `AudioBuffer` container and initiate another transfer.
    fn transfer_done(&self, channel: &dyn DMAChannel, buffer: &'static mut [usize]) {
        let channel_no = channel.channel_no() as u8;
        if channel_no == self.adc_dma_channel_no.get() {
            // The transfer that completed is for the ADC samples.
            self.in_buffer_iter.map(move |iter| {
                // Replace the buffer as an unprocessed buffer.
                let vacant_container = iter.peek().unwrap();
                vacant_container.put(buffer, BufferState::Unprocessed);

                // Re-initiate transfer from the ADC to the next buffer.
                // This buffer must be free, otherwise, that means we fail;
                // we've caught up to the oldest buffer that still hasn't completed processing.
                // We hard-fail here because it means the DSP cycle is delayed.
                let _ = iter.next();
                let next_container = iter.peek().unwrap();
                if next_container.state() != BufferState::Free {
                    debug!("Sampling has filled all buffers!");
                    for buffer in self.in_buffers.iter() {
                        debug!("({:#010X}) {:?}", buffer as *const _ as usize, buffer.state());
                    }
                    panic!("All input buffers exhausted.");
                } else {
                    let buffer = next_container.take(BufferState::Collecting).unwrap();
                    channel.start(buffer)
                        .expect("could not start sampling DMA channel");
                }
            });
        } else {
            panic!()
        }
    }
}
