//! Floating-point PI and PID controllers.

/// A floating-point PI controller.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PIController {
    k_p: f32,
    integral: IntegralComponent,
}

impl PIController {
    /// Create a new PI controller with the given gains.
    pub const fn new(k_p: f32, k_i: f32) -> Self {
        Self {
            k_p,
            integral: IntegralComponent {
                k_i,
                integral: 0.0,
            },
        }
    }

    /// Update the PI controller, returning the new output value.
    pub fn update(&mut self, setpoint: f32, measurement: f32, dt: f32) -> f32 {
        let error = setpoint - measurement;
        self.k_p * error + self.integral.update(error, dt)
    }
}

/// A floating-point PID controller.
///
/// Uses the derivative-on-measurement technique to avoid derivative kicks on
/// setpoint changes.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PIDController {
    k_p: f32,
    integral: IntegralComponent,
    derivative: DerivativeComponent,
}

impl PIDController {
    /// Create a new PID controller with the given gains.
    pub const fn new(k_p: f32, k_i: f32, k_d: f32) -> Self {
        Self {
            k_p,
            integral: IntegralComponent {
                k_i,
                integral: 0.0,
            },
            derivative: DerivativeComponent {
                k_d,
                last_measurement: None,
            },
        }
    }

    /// Update the PID controller, returning the new output value.
    pub fn update(&mut self, setpoint: f32, measurement: f32, dt: f32) -> f32 {
        let error = setpoint - measurement;
        self.k_p * error + self.integral.update(error, dt) + self.derivative.update(measurement, dt)
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct IntegralComponent {
    k_i: f32,
    integral: f32,
}

impl IntegralComponent {
    fn update(&mut self, error: f32, dt: f32) -> f32 {
        self.integral += self.k_i * error * dt;
        self.integral
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct DerivativeComponent {
    k_d: f32,
    last_measurement: Option<f32>,
}

impl DerivativeComponent {
    fn update(&mut self, measurement: f32, dt: f32) -> f32 {
        let derivative = self
            .last_measurement
            .map(|last| (measurement - last) / dt)
            .unwrap_or(0.0);

        self.last_measurement = Some(measurement);

        self.k_d * derivative
    }
}
