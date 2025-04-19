use super::answers::RPLIDAR_RESP_HQ_FLAG_SYNCBIT;
use std::cmp::Ordering;
use std::f32::consts::PI;

/// Represents a single measurement point from a laser scan.
///
/// Contains angle, distance, quality, and sync flag information.
/// Note: The internal representation uses fixed-point values for efficiency.
/// Use the provided methods (`angle()`, `distance()`, etc.) for floating-point access.
#[derive(Debug, Clone, Eq)]
pub struct ScanPoint {
    /// Angle measurement in Q14 fixed-point format relative to the scanner's zero direction.
    /// Represents (angle_degrees / 360.0) * (1 << 14) * 2.
    pub angle_z_q14: u16,
    /// Distance measurement in Q2 fixed-point format, in millimeters.
    /// Represents distance_meters * 4000.0.
    pub dist_mm_q2: u32,
    /// Quality indicator of the measurement (0-255). Higher values generally mean better quality.
    /// A value of 0 indicates an invalid measurement.
    pub quality: u8,
    /// Flag associated with the measurement. The least significant bit indicates if this is a sync/start point of a scan frame.
    pub flag: u8,
}

impl ScanPoint {
    /// Returns the angle of the scan point in radians (0 to 2*PI).
    #[inline]
    pub fn angle(&self) -> f32 {
        (self.angle_z_q14 as f32) / 16384f32 / 2f32 * PI
    }

    /// Sets the angle of the scan point from a radian value.
    ///
    /// # Arguments
    ///
    /// * `angle` - The angle in radians (0 to 2*PI).
    #[inline]
    pub fn set_angle(&mut self, angle: f32) {
        self.angle_z_q14 = (angle * 16384f32 * 2f32 / PI) as u16;
    }

    /// Returns the distance of the scan point in meters.
    #[inline]
    pub fn distance(&self) -> f32 {
        (self.dist_mm_q2 as f32) / 4000f32
    }

    /// Sets the distance of the scan point from a meter value.
    ///
    /// # Arguments
    ///
    /// * `dist` - The distance in meters.
    #[inline]
    pub fn set_distance(&mut self, dist: f32) {
        self.dist_mm_q2 = (dist * 4000f32) as u32;
    }

    /// Returns `true` if this scan point marks the start of a new 360-degree scan frame.
    #[inline]
    pub fn is_sync(&self) -> bool {
        (self.flag & RPLIDAR_RESP_HQ_FLAG_SYNCBIT) == RPLIDAR_RESP_HQ_FLAG_SYNCBIT
    }

    /// Returns `true` if the scan point measurement is considered valid (non-zero quality and distance).
    #[inline]
    pub fn is_valid(&self) -> bool {
        self.quality != 0 && self.dist_mm_q2 != 0
    }
}

impl Ord for ScanPoint {
    /// Compares `ScanPoint`s based on their angle (`angle_z_q14`).
    fn cmp(&self, other: &ScanPoint) -> Ordering {
        self.angle_z_q14.cmp(&other.angle_z_q14)
    }
}

impl PartialOrd for ScanPoint {
    /// Partially compares `ScanPoint`s based on their angle.
    fn partial_cmp(&self, other: &ScanPoint) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for ScanPoint {
    /// Checks for equality based on all fields.
    fn eq(&self, other: &ScanPoint) -> bool {
        self.angle_z_q14 == other.angle_z_q14
            && self.dist_mm_q2 == other.dist_mm_q2
            && self.quality == other.quality
            && self.flag == other.flag
    }
}

/// Describes the characteristics of a specific RPLIDAR scan mode.
#[derive(Debug, Clone, PartialEq)]
pub struct ScanMode {
    /// A unique identifier for this scan mode.
    pub id: u16,

    /// The time taken for a single measurement sample in microseconds.
    pub us_per_sample: f32,

    /// The maximum reliable measurement distance in meters for this mode.
    pub max_distance: f32,

    /// The response command code (`ANS_TYPE`) associated with measurements in this mode.
    pub ans_type: u8,

    /// A human-readable name for the scan mode (e.g., "Standard", "Express").
    pub name: String,
}

/// Options for configuring a scan operation.
#[derive(Debug, Clone, PartialEq)]
pub struct ScanOptions {
    /// Optionally specifies the desired scan mode ID to use.
    /// If `None`, the device's typical scan mode will be used.
    pub scan_mode: Option<u16>,

    /// If `true`, forces the LIDAR to start sending scan data even if the motor is not reported as stable/spinning.
    /// Use with caution.
    pub force_scan: bool,

    /// Reserved for future use. Should be set to 0.
    pub options: u32,
}

impl ScanOptions {
    /// Creates `ScanOptions` specifying a particular scan mode ID.
    ///
    /// # Arguments
    ///
    /// * `scan_mode` - The ID of the desired scan mode.
    pub fn with_mode(scan_mode: u16) -> ScanOptions {
        ScanOptions {
            scan_mode: Some(scan_mode),
            force_scan: false,
            options: 0,
        }
    }

    /// Creates `ScanOptions` that enable forced scanning.
    pub fn force_scan() -> ScanOptions {
        ScanOptions {
            scan_mode: None,
            force_scan: true,
            options: 0,
        }
    }

    /// Creates `ScanOptions` specifying a scan mode ID and enabling forced scanning.
    ///
    /// # Arguments
    ///
    /// * `scan_mode` - The ID of the desired scan mode.
    pub fn force_scan_with_mode(scan_mode: u16) -> ScanOptions {
        ScanOptions {
            scan_mode: Some(scan_mode),
            force_scan: true,
            options: 0,
        }
    }
}

impl Default for ScanOptions {
    /// Creates default `ScanOptions`: use typical scan mode, no forced scanning.
    fn default() -> ScanOptions {
        ScanOptions {
            scan_mode: None,
            force_scan: false,
            options: 0,
        }
    }
}

/// Represents the health status reported by the RPLIDAR device.
#[derive(Debug, Clone, PartialEq)]
pub enum Health {
    /// The device reports it is operating correctly.
    Healthy,
    /// The device reports a warning condition, but may still be operational. Contains the warning code.
    Warning(u16),
    /// The device reports a fatal error and is likely not operational. Contains the error code.
    Error(u16),
}
