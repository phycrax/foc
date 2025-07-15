//! Park and Clarke transformations (along with their inverses).
//!
//! The algorithms implemented here are based on [Microsemi's suggested implementation](https://www.microsemi.com/document-portal/doc_view/132799-park-inverse-park-and-clarke-inverse-clarke-transformations-mss-software-implementation-user-guide)

use crate::{FRAC_1_SQRT_3, SQRT_3};

/// A value in a reference frame that moves with the electrical angle of the
/// motor. The two axes are orthogonal.
#[derive(Debug, Clone)]
pub struct RotatingReferenceFrame {
    /// Direct axis component aligned with the rotor flux
    pub d: f32,
    /// Quadrature axis component perpendicular to the rotor flux
    pub q: f32,
}

/// A value in a reference frame that is stationary. The two axes are
/// orthogonal.
#[derive(Debug, Clone)]
pub struct TwoPhaseReferenceFrame {
    /// Alpha component aligned with phase A
    pub alpha: f32,
    /// Beta component perpendicular to alpha
    pub beta: f32,
}

/// A three-phase value in a stationary reference frame. The values do not
/// necessarily sum to 0.
#[derive(Debug, Clone)]
pub struct ThreePhaseReferenceFrame {
    /// Phase A component
    pub a: f32,
    /// Phase B component
    pub b: f32,
    /// Phase C component
    pub c: f32,
}

/// A three-phase value in a stationary reference frame, where the three values
/// sum to 0. As such, the third value is not given.
#[derive(Debug, Clone)]
pub struct ThreePhaseBalancedReferenceFrame {
    /// Phase A component
    pub a: f32,
    /// Phase B component
    pub b: f32,
}

/// Clarke transform
///
/// Implements equations 1-4 from the Microsemi guide.
pub fn clarke(inputs: ThreePhaseBalancedReferenceFrame) -> TwoPhaseReferenceFrame {
    TwoPhaseReferenceFrame {
        // Eq3
        alpha: inputs.a,
        // Eq4
        beta: FRAC_1_SQRT_3 * (inputs.a + 2.0 * inputs.b),
    }
}

/// Inverse Clarke transform
///
/// Implements equations 5-7 from the Microsemi guide.
pub fn inverse_clarke(inputs: TwoPhaseReferenceFrame) -> ThreePhaseReferenceFrame {
    ThreePhaseReferenceFrame {
        // Eq5
        a: inputs.alpha,
        // Eq6
        b: (-inputs.alpha + SQRT_3 * inputs.beta) / 2.0,
        // Eq7
        c: (-inputs.alpha - SQRT_3 * inputs.beta) / 2.0,
    }
}

/// Park transform
///
/// Implements equations 8 and 9 from the Microsemi guide.
pub fn park(
    cos_angle: f32,
    sin_angle: f32,
    inputs: TwoPhaseReferenceFrame,
) -> RotatingReferenceFrame {
    RotatingReferenceFrame {
        // Eq8
        d: cos_angle * inputs.alpha + sin_angle * inputs.beta,
        // Eq9
        q: cos_angle * inputs.beta - sin_angle * inputs.alpha,
    }
}

/// Inverse Park transform
///
/// Implements equations 10 and 11 from the Microsemi guide.
pub fn inverse_park(
    cos_angle: f32,
    sin_angle: f32,
    inputs: RotatingReferenceFrame,
) -> TwoPhaseReferenceFrame {
    TwoPhaseReferenceFrame {
        // Eq10
        alpha: cos_angle * inputs.d - sin_angle * inputs.q,
        // Eq11
        beta: sin_angle * inputs.d + cos_angle * inputs.q,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[track_caller]
    fn clark_e_round_trip(a: f32, b: f32) {
        let input = ThreePhaseBalancedReferenceFrame {
            a,
            b,
        };
        let two_phase = clarke(input.clone());
        let result = inverse_clarke(two_phase);

        assert!((result.a - input.a).abs() < 0.0001);
        assert!((result.b - input.b).abs() < 0.0001);
    }

    #[test]
    fn clarke_round_trip_zero() {
        clark_e_round_trip(0., 0.);
    }

    #[test]
    fn clarke_round_trip_two_inputs() {
        clark_e_round_trip(0., 1.);
        clark_e_round_trip(1., 0.);
        clark_e_round_trip(-0.5, -0.5);
        clark_e_round_trip(-0.1, -0.2);
        clark_e_round_trip(13., 21.);
    }

    #[test]
    fn park_round_trip() {
        let angle = 0.82;
        let (sin_angle, cos_angle) = libm::sincosf(angle);

        let input = TwoPhaseReferenceFrame {
            alpha: 2.0,
            beta: 3.0,
        };
        let moving_reference = park(cos_angle, sin_angle, input.clone());
        let result = inverse_park(cos_angle, sin_angle, moving_reference);

        assert!((result.alpha - input.alpha).abs() < 0.0001);
        assert!((result.beta - input.beta).abs() < 0.0001);
    }
}
