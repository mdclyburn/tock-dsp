/*! Processors for managing signal amplitude, affecting output volume.
 */

use crate::signal::SignalProcessor;

/// Scale samples by a constant factor.
///
/// Applies a scaling factor to all samples by performing a multiply and then a divide
/// (to avoid floating point operations).
/// This means that the factor is expressed as a fraction (improper, even).
/// For example, to create a filter to scale the input samples by 140%, use
/// `Scale::new(14, 10)` or `Scale::new(7, 5)`.
pub struct Scale {
    nominator: u8,
    denominator: u8,
}

impl Scale {
    /// Create a constant scaling processor.
    pub fn new(nominator: u8, denominator: u8) -> Scale {
        Scale {
            nominator,
            denominator,
        }
    }
}

impl SignalProcessor for Scale {
    fn process(&self, in_samples: &[i16], out_samples: &mut [i16]) {
        let samples_it = in_samples.iter().zip(out_samples);
        for (in_sample, out_sample) in samples_it {
            *out_sample = *in_sample
                           * self.nominator as i16
                           / self.denominator as i16
        }
    }
}
