/*! Digital signal processing.

Support for performing real-time digital signal processing (DSP).
Core logic for performing processing is in [`engine::DSPEngine`].
Provide a source, a sink, and a signal chain to run processing.
*/

pub mod buffer;
pub mod engine;
pub mod link;
