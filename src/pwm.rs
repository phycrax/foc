//! Algorithms to convert a value from a two-phase stationary orthogonal
//! reference frame to a value suitable to be used for PWM generation.
//!
//! The resulting waveforms of the PWM generation methods are shown below.
//! ![PWM Methods](https://raw.githubusercontent.com/phycrax/foc/main/docs/pwm_methods.png)

use crate::{park_clarke::TwoPhaseReferenceFrame, SQRT_3};

/// Trait to generalize converting a value from a two-phase stationary orthogonal
/// reference frame to a value suitable to be used for PWM generation.
pub trait Modulation {
    /// Generate PWM values based on a specific implementation.
    ///
    /// Returns a value between -1 and 1 for each channel.
    fn modulate(value: TwoPhaseReferenceFrame) -> [f32; 3];

    /// Module the value, returning the result as a value between 0 and the specified
    /// maximum value inclusive.
    fn as_compare_value(value: TwoPhaseReferenceFrame, max: u16) -> [u16; 3] {
        Self::modulate(value)
            .map(|val| (((val + 1.0) * (max as f32 + 1.0)) / 2.0).clamp(0.0, max as f32) as u16)
    }
}

/// Generate PWM values based on a space-vector method.
///
/// This method results in a waveform that is more efficient than sinusoidal
/// PWM while having better current ripple than the other methods. However, it
/// comes at the expense of a more complex computation.
///
/// Returns a value between -1 and 1 for each channel.
pub struct SpaceVector;

impl Modulation for SpaceVector {
    fn modulate(value: TwoPhaseReferenceFrame) -> [f32; 3] {
        // Convert alpha/beta to x/y/z
        let sqrt_3_alpha = SQRT_3 * value.alpha;
        let beta = value.beta;
        let x = beta;
        let y = (beta + sqrt_3_alpha) / 2.0;
        let z = (beta - sqrt_3_alpha) / 2.0;

        // Calculate which sector the value falls in
        let sector: u8 = match (
            x.is_sign_positive(),
            y.is_sign_positive(),
            z.is_sign_positive(),
        ) {
            (true, true, false) => 1,
            (_, true, true) => 2,
            (true, false, true) => 3,
            (false, false, true) => 4,
            (_, false, false) => 5,
            (false, true, false) => 6,
        };

        // Map a,b,c values to three phase
        let (ta, tb, tc);
        match sector {
            1 | 4 => {
                ta = x - z;
                tb = x + z;
                tc = -x + z;
            }
            2 | 5 => {
                ta = y - z;
                tb = y + z;
                tc = -y - z;
            }
            3 | 6 => {
                ta = y - x;
                tb = -y + x;
                tc = -y - x;
            }
            _ => unreachable!("invalid sector"),
        }

        [ta, tb, tc]
    }
}

/// Generate PWM values based on a sinusoidal waveform.
///
/// While this method is very simple (and fast) it is less efficient than SVPWM
/// as it does not utilise the bus voltage as well.
///
/// Returns a value between -1 and 1 for each channel.
pub struct Sinusoidal;

impl Modulation for Sinusoidal {
    fn modulate(value: TwoPhaseReferenceFrame) -> [f32; 3] {
        let voltages = crate::park_clarke::inverse_clarke(value);

        [voltages.a, voltages.b, voltages.c]
    }
}

/// Generate PWM values based on a trapezoidal wave.
///
/// Note that for this method to work properly, when the output is 0 the
/// resective channel should be disabled/set as high impedance.
///
/// Returns a value between -1 and 1 for each channel.
pub struct Trapezoidal;

impl Modulation for Trapezoidal {
    fn modulate(value: TwoPhaseReferenceFrame) -> [f32; 3] {
        let voltages = crate::park_clarke::inverse_clarke(value);

        [
            (voltages.a * 2.0).signum(),
            (voltages.b * 2.0).signum(),
            (voltages.c * 2.0).signum(),
        ]
    }
}

/// Generate PWM values based on a square wave.
///
/// Returns a value between -1 and 1 for each channel.
pub struct Square;

impl Modulation for Square {
    fn modulate(value: TwoPhaseReferenceFrame) -> [f32; 3] {
        let voltages = crate::park_clarke::inverse_clarke(value);

        [
            voltages.a.signum(),
            voltages.b.signum(),
            voltages.c.signum(),
        ]
    }
}
