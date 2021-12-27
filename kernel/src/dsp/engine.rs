//! Signal processing core logic.

use core::cell::Cell;
use core::iter::{Cycle, Iterator, Peekable};
use core::slice::Iter as SliceIter;

use crate::config;
use crate::debug;
use crate::dsp::buffer::{SampleContainer, BufferState};
use crate::dsp::link::Chain;
use crate::errorcode::ErrorCode;
use crate::hil::dma::{self, DMA, DMAChannel};
use crate::hil::time::{self, Time, ConvertTicks};
use crate::platform::KernelResources;
use crate::platform::chip::Chip;
use crate::sync::{Lockable, Mutex};
use crate::utilities::cells::MapCell;

pub struct Resources<'a, C: Chip, F: time::Frequency, T: time::Ticks> {
    pub chip: &'a C,
    pub dma: &'static dyn DMA,
    pub time: &'a dyn Time<Frequency = F, Ticks = T>,
}

/// Iterator to cycle over buffer containers endlessly.
type CyclicContainerIter = Peekable<Cycle<SliceIter<'static, SampleContainer>>>;

/// Returns the sampling rate of the engine.
pub fn sampling_rate() -> usize { config::SAMPLING_RATE }

/// Orchestrator of the digital signal processing cycle.
///
/// This is the heart of the DSP.
/// Start signal processing with [`DSPEngine::run()`].
pub struct DSPEngine<L: Lockable> {
    /// Runtime statistics.
    stats: Mutex<L, Statistics>,
    /// Buffers for incoming audio samples.
    in_containers: [SampleContainer; config::SAMPLE_BUFFERS],
    /// Buffers for outgoing audio samples.
    out_containers: [SampleContainer; config::SAMPLE_BUFFERS],
    /// Source DMA channel number.
    source_dma_channel_no: Cell<u8>,
    /// Sink DMA channel number.
    sink_dma_channel_no: Cell<u8>,
    /// Cyclical input buffer iterator.
    in_container_iter: MapCell<CyclicContainerIter>,
    /// Cyclical output buffer iterator.
    out_container_iter: MapCell<CyclicContainerIter>,
    /// Whether playback DMA is suspended due to ready buffers being unavailable.
    playback_stalled: Cell<bool>,
}

impl<L: Lockable> DSPEngine<L> {
    /// Create a new `DSPEngine` instance.
    pub unsafe fn new(stats: Mutex<L, Statistics>) -> DSPEngine<L> {
        DSPEngine {
            stats,
            in_containers: [SampleContainer::new(),
                         SampleContainer::new(),
                         SampleContainer::new(),
                         SampleContainer::new(),
                         SampleContainer::new()],
            out_containers: [SampleContainer::new(),
                          SampleContainer::new(),
                          SampleContainer::new(),
                          SampleContainer::new(),
                          SampleContainer::new()],
            source_dma_channel_no: Cell::new(99),
            sink_dma_channel_no: Cell::new(99),
            in_container_iter: MapCell::empty(),
            out_container_iter: MapCell::empty(),
            playback_stalled: Cell::new(true),
        }
    }

    /// Run the digital signal processing loop.
    ///
    /// Performs initial configuration and starts the DSP loop.
    /// Once configured, the DSP side of the kernel will respond to a short list of interrupts:
    /// SIO, for interprocessor messaging;
    /// DMA, for sample processing and output.
    pub fn run<'a,
               C: Chip,
               F: time::Frequency,
               T: time::Ticks>(
        &'static self,
        resources: &Resources<'a, C, F, T>,
        chain: &Chain,
        source: dma::SourcePeripheral,
        sink: dma::TargetPeripheral,
    ) -> !
    {
        // Configure sample input DMA.
        let source_dma_channel = {
            let params = dma::Parameters {
                kind: dma::TransferKind::PeripheralToMemory(source, 0),
                transfer_count: config::NO_SAMPLES,
                // Need to optimize this to allow transferring a halfword.
                transfer_size: dma::TransferSize::Word,
                increment_on_read: false,
                increment_on_write: true,
                high_priority: true,
            };
            resources.dma.configure(&params)
                .expect("failed configuring source DMA channel")
        };
        source_dma_channel.set_client(self);
        self.source_dma_channel_no.set(source_dma_channel.channel_no() as u8);

        // Configure sample output DMA.
        let sink_dma_channel = {
            let params = dma::Parameters {
                kind: dma::TransferKind::MemoryToPeripheral(0, sink),
                transfer_count: config::NO_SAMPLES,
                transfer_size: dma::TransferSize::Word,
                increment_on_read: true,
                increment_on_write: false,
                high_priority: true,
            };
            resources.dma.configure(&params)
                .expect("failed configuring sink DMA channel")
        };
        sink_dma_channel.set_client(self);
        self.sink_dma_channel_no.set(sink_dma_channel.channel_no() as u8);

        // Create cyclic iterators over the audio buffers.
        let mut process_in_buffer_it = self.in_containers.iter().cycle();
        let mut process_out_buffer_it = self.out_containers.iter().cycle();

        let out_buffer_it = self.out_containers.iter().cycle().peekable();
        self.out_container_iter.put(out_buffer_it);

        // Start sample collection.
        self.initiate_sampling(source_dma_channel)
            .expect("failed to start ADC sampling DMA");

        // Depending on the length of the signal chain and the processing strategy,
        // the samples go back and forth between buffers as we go through the chain.
        // If there are an even number of processors in the chain,
        // then we must bounce buffer ownership between the SampleContainers to avoid copying samples.
        let exchange_buffers_after_processing = {
            let link_count = chain.into_iter().count();
            link_count % 2 == 0
        };
        loop {
            // Obtain the next container of unprocessed sequence of audio samples.
            // Obtain the next container of free output buffer for processed samples.
            let input_buffer = process_in_buffer_it.next().unwrap();
            let output_buffer = process_out_buffer_it.next().unwrap();

            // Grab the buffers.
            let (proc_buf_a, proc_buf_b): (&'static mut [usize], _) = {
                let (mut in_buffer, mut out_buffer) = (None, None);

                while in_buffer.is_none() {
                    in_buffer = unsafe {
                        resources.chip.atomic(|| {
                            if input_buffer.state() == BufferState::Unprocessed {
                                let buffer = input_buffer.take(BufferState::Processing)
                                    .expect("buffer unexpectedly missing");
                                Some(buffer)
                            } else {
                                None
                            }
                        })
                    };
                }

                while out_buffer.is_none() {
                    out_buffer = unsafe {
                        resources.chip.atomic(|| {
                            if output_buffer.state() == BufferState::Free {
                                let buffer = output_buffer.take(BufferState::Processing)
                                    .expect("buffer unexpectedly missing");
                                Some(buffer)
                            } else {
                                None
                            }
                        })
                    };
                }

                (in_buffer.unwrap(), out_buffer.unwrap())
            };

            // Iterate through all links in the chain and run their processors.
            // Input samples buffer → signal processor → output samples buffer.
            for (link_no, link) in (0..).zip(chain) {
                if link_no % 2 == 0 {
                    link.processor().process(&*proc_buf_a, proc_buf_b);
                } else {
                    link.processor().process(&*proc_buf_b, proc_buf_a);
                }
            }

            // Replace the buffers.
            // The output buffer needs to go to BufferState::Ready when we implement playing processed samples.
            let other_buf = if exchange_buffers_after_processing {
                input_buffer.put(proc_buf_b, BufferState::Free);
                proc_buf_a
            } else {
                input_buffer.put(proc_buf_a, BufferState::Free);
                proc_buf_b
            };
            output_buffer.put(other_buf, BufferState::Ready);
            // Disable playback and mark buffer as free.
            // output_buffer.put(other_buf, BufferState::Free);

            if self.playback_stalled.get() {
                self.playback_stalled.set(false);
                let other_buf = output_buffer.take(BufferState::Playing).unwrap();
                sink_dma_channel.start(other_buf);
            }
        }
    }

    /// Start pulling samples from the signal source.
    fn initiate_sampling(&'static self, dma_channel: &dyn DMAChannel) -> Result<(), ErrorCode> {
        let buffer_iter = self.in_containers.iter().cycle().peekable();
        self.in_container_iter.put(buffer_iter);
        let buffer = self.in_container_iter
            .map(|iter| iter.peek().unwrap().take(BufferState::Collecting).unwrap())
            .unwrap();

        dma_channel.start(buffer)
    }
}

impl<L: Lockable> dma::DMAClient for DSPEngine<L> {
    /// Restore incoming sample buffer and restart a transfer.
    ///
    /// The DSP engine receives this callback for completed transfers from the sample source and sink.
    /// This callback replaces the buffer to its `SampleContainer` container and initiate another transfer.
    fn transfer_done(&self, channel: &dyn DMAChannel, buffer: &'static mut [usize]) {
        let channel_no = channel.channel_no() as u8;
        if channel_no == self.source_dma_channel_no.get() {
            // The transfer that completed is for the ADC samples.
            self.in_container_iter.map(move |iter| {
                // Replace the buffer as an unprocessed buffer.
                let vacant_container = iter.peek().unwrap();
                vacant_container.put(buffer, BufferState::Unprocessed);

                // Re-initiate transfer from the source to the next buffer.
                // This buffer must be free, otherwise, that means we fail;
                // we've caught up to the oldest buffer that still hasn't completed processing.
                // We hard-fail here because it means the DSP cycle is delayed.
                let _ = iter.next();
                let next_container = iter.peek().unwrap();
                if next_container.state() != BufferState::Free {
                    debug!("Sampling has filled all buffers!");
                    for buffer in self.in_containers.iter() {
                        debug!("({:#010X}) {:?}", buffer as *const _ as usize, buffer.state());
                    }
                    for buffer in self.out_containers.iter() {
                        debug!("({:#010X}) {:?}", buffer as *const _ as usize, buffer.state());
                    }
                    panic!("All input buffers exhausted.");
                } else {
                    let buffer = next_container.take(BufferState::Collecting).unwrap();
                    channel.start(buffer)
                        .expect("could not start DMA from source");
                }
            });
        } else if channel_no == self.sink_dma_channel_no.get() {
            // The transfer that completed is for the sink.
            self.out_container_iter.map(move |iter| {
                // Replace the buffer as a free buffer.
                let vacant_container = iter.peek().unwrap();
                vacant_container.put(buffer, BufferState::Free);

                // Re-initiate transfer to the sink from the next buffer.
                // This buffer must be ready, otherwise, that means playback is noncontiguous;
                // we've completed playback of the last buffer and no new buffers are available.
                let _ = iter.next();
                let next_container = iter.peek().unwrap();
                if next_container.state() != BufferState::Ready {
                    self.playback_stalled.set(true);
                } else {
                    let buffer = next_container.take(BufferState::Playing).unwrap();
                    channel.start(buffer)
                        .expect("could not start DMA to sink");
                }
            });
        } else {
            // We received a callback from a channel we were not expecting.
            // This was neither the source nor the sink DMA.
            panic!();
        }
    }
}

#[derive(Default)]
pub struct Statistics {
    processing_loop_us: usize,
}
