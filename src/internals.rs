use super::answers::*;
use std::time::Duration;

/// Default timeout duration for waiting for responses from the RPLIDAR.
pub const RPLIDAR_DEFAULT_TIMEOUT: Duration = Duration::from_secs(1);

/// Default capacity for the internal buffer caching received scan points.
pub const RPLIDAR_DEFAULT_CACHE_DEPTH: usize = 8192;

/// Default PWM value used when starting the motor with `start_motor()`.
pub const RPLIDAR_DEFAULT_MOTOR_PWM: u16 = 600;

/// Internal enum to store the previous measurement capsule for angle calculation in capsuled modes.
#[derive(Debug, Clone, PartialEq)]
pub enum CachedPrevCapsule {
    /// No previous capsule stored (initial state or after non-capsuled data).
    None,
    /// Stores the previous standard capsuled measurement response.
    Capsuled(RplidarResponseCapsuleMeasurementNodes),
    /// Stores the previous ultra capsuled measurement response.
    UltraCapsuled(RplidarResponseUltraCapsuleMeasurementNodes),
    /// Stores the previous dense capsuled measurement response.
    DenseCapsuled(RplidarResponseDenseCapsuleMeasurementNodes),
}
