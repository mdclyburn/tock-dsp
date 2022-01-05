/// Sample size in bits.
const SAMPLE_SIZE: usize = 16;
/// Number of samples collect in one second.
const SAMPLING_RATE: usize = 44_100;
/// How many milliseconds worth of samples each buffer will hold.
const BUFFER_LEN_MS: usize = 20;
/// Number of samples each buffer holds.
const NO_SAMPLES: usize = SAMPLING_RATE * BUFFER_LEN_MS / 1000;

/// Bit size of each sample.
pub const fn sample_size() -> usize { SAMPLE_SIZE }

/// Returns the sampling rate of the engine.
pub const fn sampling_rate() -> usize { SAMPLING_RATE }

/// Number of samples in each sample buffer.
pub const fn buffer_len_samples() -> usize { NO_SAMPLES }

/// Length of time each buffer has samples for, in milliseconds.
pub const fn buffer_len_ms() -> usize { BUFFER_LEN_MS }
