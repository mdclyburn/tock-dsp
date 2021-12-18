//! Sample data workers.

use core::iter::Cycle;
use core::slice::Iter as SliceIter;

use crate::config;
use crate::dsp::buffer::{AudioBuffer, BufferState};
use crate::dsp::link::{Chain, Link};
use crate::errorcode::ErrorCode;
use crate::hil::dma::{self, DMA, DMAChannel};
use crate::utilities::cells::OptionalCell;

pub fn processing_loop() -> ! {
    // We do not call `processing_loop` until we have set the processing chain.
    let chain = unsafe { PROCESSING_CHAIN.unwrap() };
    let (mut input_buffers_it, mut output_buffers_it) = unsafe {
        if let Some(proc_buffers) = SAMPLE_BUFFERS {
            (proc_buffers.input_buffers.iter().cycle(),
             proc_buffers.output_buffers.iter().cycle())
        } else {
            panic!("Buffers not configured through configure_loop()");
        }
    };

    loop {
        // Obtain the next unprocessed sequence of audio samples.
        // Obtain the next free output buffer for processed samples.
        let input_buffer = input_buffers_it.next().unwrap();
        let output_buffer = output_buffers_it.next().unwrap();
        while input_buffer.state() != BufferState::Unprocessed {  }
        while output_buffer.state() != BufferState::Free {  }

        // Iterate through all links in the processing chain and apply them.
        let in_samples = input_buffer.take(BufferState::Processing).unwrap();
        let out_samples = output_buffer.take(BufferState::Processing).unwrap();

        for link in &chain {  }

        input_buffer.put(in_samples, BufferState::Free);
        output_buffer.put(out_samples, BufferState::Ready);
    }
}
