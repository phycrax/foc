//! Floating-point PID controller.

/// A floating-point PID controller.
///
/// Uses the derivative-on-measurement technique to avoid derivative kicks on
/// setpoint changes.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PIDController {
    p: ProportionalComponent,
    i: IntegralComponent,
    d: DerivativeComponent,
}

impl PIDController {
    /// Create a new PID controller with the given gains.
    pub const fn new(k_p: f32, k_i: f32, k_d: f32) -> Self {
        let p = ProportionalComponent { gain: k_p };

        let i = IntegralComponent {
            gain: k_i,
            integral: 0.0,
        };

        let d = DerivativeComponent {
            gain: k_d,
            last_measurement: None,
        };

        Self { p, i, d }
    }

    /// Update the PID controller, returning the new output value.
    pub fn update(&mut self, setpoint: f32, measurement: f32, dt: f32) -> f32 {
        let error = setpoint - measurement;

        let p = self.p.update(error);
        let i = self.i.update(error, dt);
        let d = self.d.update(measurement, dt);

        p + i + d
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct ProportionalComponent {
    gain: f32,
}

impl ProportionalComponent {
    fn update(&mut self, error: f32) -> f32 {
        self.gain * error
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct IntegralComponent {
    gain: f32,
    integral: f32,
}

impl IntegralComponent {
    fn update(&mut self, error: f32, dt: f32) -> f32 {
        self.integral += self.gain * error * dt;
        self.integral
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct DerivativeComponent {
    gain: f32,
    last_measurement: Option<f32>,
}

impl DerivativeComponent {
    fn update(&mut self, measurement: f32, dt: f32) -> f32 {
        let derivative = self
            .last_measurement
            .map(|last| (measurement - last) / dt)
            .unwrap_or_default();

        self.last_measurement = Some(measurement);

        self.gain * derivative
    }
}
