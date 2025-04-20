//! # Rplidar Driver
//!
//! `rplidar` is a driver for Slamtec RPLIDAR series laser sensors, built upon the `rpos-drv` infrastructure.
//! It provides high-level access to RPLIDAR functionalities like getting device info, health, starting/stopping scans,
//! and retrieving scan data.

extern crate byteorder;
extern crate crc32fast;
extern crate log;

mod answers;
pub mod base;
mod checksum;
mod cmds;
mod internals;
mod parsers;
mod protocol;
pub mod types;
pub mod utils;

// Re-export device info structure for convenience.
pub use crate::answers::RplidarResponseDeviceInfo;

use crate::parsers::capsuled_parser::parse_capsuled;
use crate::parsers::dense_capsuled_parser::parse_dense_capsuled;
use crate::parsers::ultra_capsuled_parser::parse_ultra_capsuled;

use crate::answers::*;
pub use crate::base::{Channel, Error, Message, Result};
use crate::checksum::Checksum;
use crate::cmds::*;
use crate::internals::*;
pub use crate::protocol::RplidarHostProtocol;
use byteorder::{ByteOrder, LittleEndian};
use log::{error, trace, warn};
use std::collections::VecDeque;
use std::io::{Read, Write};
use std::mem::transmute_copy;
use std::time::{Duration, Instant};

// Add specific use statements for types used internally in this file
use crate::types::{Health, ScanMode, ScanOptions, ScanPoint};

/// Represents a connection to and control interface for an RPLIDAR device.
///
/// This struct provides methods to interact with the LIDAR, such as retrieving device information,
/// controlling the motor, starting and stopping scans, and grabbing scan data points or full frames.
///
/// It requires a `Channel` configured with `RplidarHostProtocol` for communication.
#[derive(Debug)]
pub struct RplidarDevice<T: ?Sized> {
    channel: Channel<RplidarHostProtocol, T>,
    cached_measurement_nodes: VecDeque<ScanPoint>,
    cached_prev_capsule: CachedPrevCapsule,
}

macro_rules! parse_resp_data {
    ($x:expr, $t:ty) => {{
        const SIZE: usize = std::mem::size_of::<$t>();
        if $x.len() != SIZE {
            Err(Error::ProtocolError {
                description: format!(
                    "invalid data size for {}: expected {}, got {}",
                    stringify!($t),
                    SIZE,
                    $x.len()
                ),
            })
        } else {
            Ok(unsafe {
                std::mem::transmute_copy::<[u8; SIZE], $t>(
                    $x.as_slice().try_into().unwrap_unchecked(),
                )
            })
        }
    }};
}

macro_rules! parse_resp {
    ($x:expr, $t:ty) => {
        parse_resp_data!($x.data, $t)
    };
}

macro_rules! handle_resp {
    ($ans:expr, $x:expr, $t:ty) => {
        if $x.cmd != $ans {
            Err(Error::ProtocolError {
                description: format!(
                    "unexpected response type: expected {:02X}, got {:02X}",
                    $ans, $x.cmd
                ),
            })
        } else {
            parse_resp!($x, $t)
        }
    };
}

impl From<RplidarResponseMeasurementNodeHq> for ScanPoint {
    /// Converts a high-quality measurement node response into a standard `ScanPoint`.
    fn from(p: RplidarResponseMeasurementNodeHq) -> ScanPoint {
        ScanPoint {
            angle_z_q14: p.angle_z_q14,
            dist_mm_q2: p.dist_mm_q2,
            quality: p.quality,
            flag: p.flag,
        }
    }
}

impl<T: ?Sized> RplidarDevice<T>
where
    T: Read + Write,
{
    /// Constructs a new `RplidarDevice` using an existing `Channel`.
    ///
    /// # Arguments
    ///
    /// * `channel` - A `Channel` instance configured with `RplidarHostProtocol` and connected to the RPLIDAR's communication stream (e.g., serial port).
    ///
    /// # Example
    /// ```ignore
    /// # use rplidar::{RplidarDevice, RplidarHostProtocol, Channel};
    /// # use std::io::{Read, Write};
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// let serial_port_name = "/dev/ttyUSB0"; // Or "COM3" on Windows
    /// let mut serial_port = serialport::new(serial_port_name, 115200).open()?;
    /// let channel = Channel::new(RplidarHostProtocol::new(), serial_port);
    /// let mut rplidar_device = RplidarDevice::new(channel);
    /// // Use rplidar_device...
    /// # Ok(())
    /// # }
    /// ```
    pub fn new(channel: Channel<RplidarHostProtocol, T>) -> RplidarDevice<T> {
        trace!("Creating new RplidarDevice");
        RplidarDevice {
            channel,
            cached_measurement_nodes: VecDeque::with_capacity(RPLIDAR_DEFAULT_CACHE_DEPTH),
            cached_prev_capsule: CachedPrevCapsule::None,
        }
    }

    /// Constructs a new `RplidarDevice` directly from a communication stream (e.g., a serial port).
    ///
    /// This is a convenience method that creates the `Channel` and `RplidarHostProtocol` internally.
    ///
    /// # Arguments
    ///
    /// * `stream` - A boxed `Read + Write` object representing the communication stream.
    ///
    /// # Example
    /// ```ignore
    /// # use rplidar::RplidarDevice;
    /// # use std::io::{Read, Write};
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// let serial_port_name = "/dev/ttyUSB0"; // Or "COM3" on Windows
    /// let serial_port = serialport::new(serial_port_name, 115200).open()?;
    /// let mut rplidar_device = RplidarDevice::with_stream(serial_port);
    /// // Use rplidar_device...
    /// # Ok(())
    /// # }
    /// ```
    pub fn with_stream(stream: Box<T>) -> RplidarDevice<T> {
        trace!("Creating new RplidarDevice with stream");
        RplidarDevice::<T>::new(Channel::new(RplidarHostProtocol::new(), stream))
    }

    /// Gets the device information (model, firmware, hardware, serial number) of the RPLIDAR.
    /// Uses the default timeout (`RPLIDAR_DEFAULT_TIMEOUT`).
    pub fn get_device_info(&mut self) -> Result<RplidarResponseDeviceInfo> {
        trace!("Getting device info with default timeout");
        self.get_device_info_with_timeout(RPLIDAR_DEFAULT_TIMEOUT)
    }

    /// Gets the device information with a specified timeout.
    ///
    /// # Arguments
    ///
    /// * `timeout` - The maximum duration to wait for a response.
    pub fn get_device_info_with_timeout(
        &mut self,
        timeout: Duration,
    ) -> Result<RplidarResponseDeviceInfo> {
        trace!("Getting device info with timeout: {:?}", timeout);
        let cmd = RPLIDAR_CMD_GET_DEVICE_INFO;
        let msg = Message::new(cmd);
        trace!("Invoking command: {:02X}", cmd);
        match self.channel.invoke(&msg, timeout) {
            Ok(Some(resp_msg)) => {
                trace!(
                    "Received response for {:02X}, type: {:02X}, len: {}",
                    cmd,
                    resp_msg.cmd,
                    resp_msg.data.len()
                );
                let result = handle_resp!(
                    RPLIDAR_ANS_TYPE_DEVINFO,
                    resp_msg,
                    RplidarResponseDeviceInfo
                );
                if result.is_ok() {
                    trace!("Successfully parsed device info");
                } else {
                    error!(
                        "Failed to parse device info response: {:?}",
                        result.as_ref().err()
                    );
                }
                result
            }
            Ok(None) => {
                warn!("Timeout waiting for device info response");
                Err(Error::OperationTimeout)
            }
            Err(e) => {
                error!("Error invoking get_device_info: {:?}", e);
                Err(e)
            }
        }
    }

    /// Sends the stop command to the RPLIDAR, halting any ongoing measurements.
    pub fn stop(&mut self) -> Result<()> {
        trace!("Sending STOP command ({:02X})", RPLIDAR_CMD_STOP);
        self.channel.write(&Message::new(RPLIDAR_CMD_STOP))?;
        trace!("STOP command sent successfully");
        Ok(())
    }

    /// Sends the reset command to the RPLIDAR core.
    /// This usually requires re-initializing the device afterwards.
    pub fn core_reset(&mut self) -> Result<()> {
        trace!("Sending RESET command ({:02X})", RPLIDAR_CMD_RESET);
        self.channel.write(&Message::new(RPLIDAR_CMD_RESET))?;
        trace!("RESET command sent successfully");
        Ok(())
    }

    /// Sets the motor's Pulse Width Modulation (PWM) duty cycle.
    /// This controls the motor speed. Requires an accessory board or a model with built-in motor control.
    ///
    /// # Arguments
    ///
    /// * `pwm` - The PWM value (typically 0-1023, but check device specifics). 0 usually stops the motor.
    pub fn set_motor_pwm(&mut self, pwm: u16) -> Result<()> {
        trace!("Setting motor PWM to {}", pwm);
        let mut payload = [0; 2];
        LittleEndian::write_u16(&mut payload, pwm);

        let cmd = RPLIDAR_CMD_SET_MOTOR_PWM;
        let msg = Message::with_data(cmd, &payload);
        trace!(
            "Sending SET_MOTOR_PWM command ({:02X}) with payload: {:?}",
            cmd,
            payload
        );
        self.channel.write(&msg)?;
        trace!("SET_MOTOR_PWM command sent successfully");
        Ok(())
    }

    /// Stops the RPLIDAR's motor by setting PWM to 0.
    /// Convenience function for `set_motor_pwm(0)`.
    pub fn stop_motor(&mut self) -> Result<()> {
        trace!("Stopping motor (set_motor_pwm(0))");
        self.set_motor_pwm(0)
    }

    /// Starts the RPLIDAR's motor using the default PWM value (`RPLIDAR_DEFAULT_MOTOR_PWM`).
    /// Convenience function for `set_motor_pwm(RPLIDAR_DEFAULT_MOTOR_PWM)`.
    pub fn start_motor(&mut self) -> Result<()> {
        trace!(
            "Starting motor with default PWM ({})",
            RPLIDAR_DEFAULT_MOTOR_PWM
        );
        self.set_motor_pwm(RPLIDAR_DEFAULT_MOTOR_PWM)
    }

    /// get lidar config with timeout
    fn get_lidar_conf_with_timeout(
        &mut self,
        config_type: u32,
        timeout: Duration,
    ) -> Result<Vec<u8>> {
        trace!(
            "Getting lidar config type {:08X} with timeout {:?}",
            config_type,
            timeout
        );
        self.get_lidar_conf_with_param_and_timeout(config_type, &[], timeout)
    }

    /// get lidar config with parameter and timeout
    fn get_lidar_conf_with_param_and_timeout(
        &mut self,
        config_type: u32,
        param: &[u8],
        timeout: Duration,
    ) -> Result<Vec<u8>> {
        trace!(
            "Getting lidar config type {:08X} with param {:?} and timeout {:?}",
            config_type,
            param,
            timeout
        );
        let mut data = Vec::with_capacity(4 + param.len());
        data.extend_from_slice(&config_type.to_le_bytes());
        data.extend_from_slice(param);
        let cmd = RPLIDAR_CMD_GET_LIDAR_CONF;
        let msg = Message { cmd, data };

        trace!(
            "Invoking GET_LIDAR_CONF ({:02X}) with payload: {:?}",
            cmd,
            msg.data
        );
        let response = self.channel.invoke(&msg, timeout);

        match response {
            Ok(Some(mut response_msg)) => {
                trace!(
                    "Received response for GET_LIDAR_CONF, type: {:02X}, len: {}",
                    response_msg.cmd,
                    response_msg.data.len()
                );
                if response_msg.cmd != RPLIDAR_ANS_TYPE_GET_LIDAR_CONF {
                    error!(
                        "Unexpected response type for GET_LIDAR_CONF: got {:02X}, expected {:02X}",
                        response_msg.cmd, RPLIDAR_ANS_TYPE_GET_LIDAR_CONF
                    );
                    Err(Error::ProtocolError {
                        description: format!(
                            "unexpected response type for GET_LIDAR_CONF: got {:02X}",
                            response_msg.cmd
                        ),
                    })
                } else if response_msg.data.len() < 4 {
                    error!(
                        "GET_LIDAR_CONF response too short: len {}",
                        response_msg.data.len()
                    );
                    Err(Error::ProtocolError {
                        description: "GET_LIDAR_CONF response too short".to_owned(),
                    })
                } else {
                    let resp_config_type = LittleEndian::read_u32(&response_msg.data[0..4]);
                    if resp_config_type != config_type {
                        error!("GET_LIDAR_CONF response config type mismatch: got {:08X}, expected {:08X}", resp_config_type, config_type);
                        Err(Error::ProtocolError {
                            description: "response config type mismatch".to_owned(),
                        })
                    } else {
                        let result_data = response_msg.data.split_off(4);
                        trace!("Successfully parsed GET_LIDAR_CONF response for type {:08X}, data len: {}", config_type, result_data.len());
                        Ok(result_data)
                    }
                }
            }
            Ok(None) => {
                warn!(
                    "Timeout waiting for GET_LIDAR_CONF response (type {:08X})",
                    config_type
                );
                Err(Error::OperationTimeout)
            }
            Err(e) => {
                error!(
                    "Error invoking GET_LIDAR_CONF (type {:08X}): {:?}",
                    config_type, e
                );
                Err(e)
            }
        }
    }

    /// Gets the ID of the typical scan mode for the connected LIDAR.
    /// Uses the default timeout.
    pub fn get_typical_scan_mode(&mut self) -> Result<u16> {
        trace!("Getting typical scan mode with default timeout");
        self.get_typical_scan_mode_with_timeout(RPLIDAR_DEFAULT_TIMEOUT)
    }

    /// Gets the ID of the typical scan mode with a specified timeout.
    ///
    /// # Arguments
    ///
    /// * `timeout` - The maximum duration to wait for responses.
    pub fn get_typical_scan_mode_with_timeout(&mut self, timeout: Duration) -> Result<u16> {
        trace!("Getting typical scan mode with timeout {:?}", timeout);
        let scan_mode_data =
            self.get_lidar_conf_with_timeout(RPLIDAR_CONF_SCAN_MODE_TYPICAL, timeout)?;
        trace!("Received typical scan mode data: {:?}", scan_mode_data);
        let result = parse_resp_data!(scan_mode_data, u16);
        if let Ok(mode_id) = result {
            trace!("Parsed typical scan mode ID: {}", mode_id);
        } else {
            error!(
                "Failed to parse typical scan mode data: {:?}",
                result.as_ref().err()
            );
        }
        result
    }

    /// get lidar sample duration
    fn get_scan_mode_us_per_sample_with_timeout(
        &mut self,
        scan_mode: u16,
        timeout: Duration,
    ) -> Result<f32> {
        trace!(
            "Getting us_per_sample for scan mode {} with timeout {:?}",
            scan_mode,
            timeout
        );
        let mut param = [0; 2];
        LittleEndian::write_u16(&mut param, scan_mode);
        let us_per_sample_data = self.get_lidar_conf_with_param_and_timeout(
            RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE,
            &param,
            timeout,
        )?;
        trace!("Received us_per_sample data: {:?}", us_per_sample_data);
        let result = parse_resp_data!(us_per_sample_data, u32);
        match result {
            Ok(raw_val) => {
                let us_per_sample = (raw_val as f32) / 256f32;
                trace!(
                    "Parsed us_per_sample for mode {}: {}",
                    scan_mode,
                    us_per_sample
                );
                Ok(us_per_sample)
            }
            Err(e) => {
                error!(
                    "Failed to parse us_per_sample data for mode {}: {:?}",
                    scan_mode, e
                );
                Err(e)
            }
        }
    }

    /// get lidar scan mode max distance
    fn get_scan_mode_max_distance_with_timeout(
        &mut self,
        scan_mode: u16,
        timeout: Duration,
    ) -> Result<f32> {
        trace!(
            "Getting max_distance for scan mode {} with timeout {:?}",
            scan_mode,
            timeout
        );
        let mut param = [0; 2];
        LittleEndian::write_u16(&mut param, scan_mode);
        let max_distance_data = self.get_lidar_conf_with_param_and_timeout(
            RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE,
            &param,
            timeout,
        )?;
        trace!("Received max_distance data: {:?}", max_distance_data);
        let result = parse_resp_data!(max_distance_data, u32);
        match result {
            Ok(raw_val) => {
                let max_distance = (raw_val as f32) / 256f32;
                trace!(
                    "Parsed max_distance for mode {}: {}",
                    scan_mode,
                    max_distance
                );
                Ok(max_distance)
            }
            Err(e) => {
                error!(
                    "Failed to parse max_distance data for mode {}: {:?}",
                    scan_mode, e
                );
                Err(e)
            }
        }
    }

    /// get scan mode answer type
    fn get_scan_mode_ans_type_with_timeout(
        &mut self,
        scan_mode: u16,
        timeout: Duration,
    ) -> Result<u8> {
        trace!(
            "Getting ans_type for scan mode {} with timeout {:?}",
            scan_mode,
            timeout
        );
        let mut param = [0; 2];
        LittleEndian::write_u16(&mut param, scan_mode);
        let ans_type_data = self.get_lidar_conf_with_param_and_timeout(
            RPLIDAR_CONF_SCAN_MODE_ANS_TYPE,
            &param,
            timeout,
        )?;
        trace!("Received ans_type data: {:?}", ans_type_data);
        let result = parse_resp_data!(ans_type_data, u8);
        if let Ok(ans_type) = result {
            trace!("Parsed ans_type for mode {}: {:02X}", scan_mode, ans_type);
        } else {
            error!(
                "Failed to parse ans_type data for mode {}: {:?}",
                scan_mode,
                result.as_ref().err()
            );
        }
        result
    }

    /// get scan mode name
    fn get_scan_mode_name_with_timeout(
        &mut self,
        scan_mode: u16,
        timeout: Duration,
    ) -> Result<String> {
        trace!(
            "Getting name for scan mode {} with timeout {:?}",
            scan_mode,
            timeout
        );
        let mut param = [0; 2];
        LittleEndian::write_u16(&mut param, scan_mode);
        let name_data = self.get_lidar_conf_with_param_and_timeout(
            RPLIDAR_CONF_SCAN_MODE_NAME,
            &param,
            timeout,
        )?;
        trace!("Received name data ({} bytes)", name_data.len());

        match std::str::from_utf8(&name_data) {
            Ok(name_str) => {
                let name = name_str.trim_matches('\0').to_owned();
                trace!("Parsed name for mode {}: '{}'", scan_mode, name);
                Ok(name)
            }
            Err(e) => {
                error!(
                    "Invalid UTF-8 in scan mode name for mode {}: {}",
                    scan_mode, e
                );
                Err(Error::ProtocolError {
                    description: "invalid scan mode name".to_owned(),
                })
            }
        }
    }

    /// get scan mode count
    fn get_scan_mode_count_with_timeout(&mut self, timeout: Duration) -> Result<u16> {
        trace!("Getting scan mode count with timeout {:?}", timeout);
        let scan_mode_count_data =
            self.get_lidar_conf_with_timeout(RPLIDAR_CONF_SCAN_MODE_COUNT, timeout)?;
        trace!("Received scan mode count data: {:?}", scan_mode_count_data);
        let result = parse_resp_data!(scan_mode_count_data, u16);
        if let Ok(count) = result {
            trace!("Parsed scan mode count: {}", count);
        } else {
            error!(
                "Failed to parse scan mode count data: {:?}",
                result.as_ref().err()
            );
        }
        result
    }

    /// get scan mode of specific scan mode id
    fn get_scan_mode_with_timeout(
        &mut self,
        scan_mode: u16,
        timeout: Duration,
    ) -> Result<ScanMode> {
        trace!(
            "Getting full scan mode info for mode {} with timeout {:?}",
            scan_mode,
            timeout
        );
        Ok(ScanMode {
            id: scan_mode,
            us_per_sample: self.get_scan_mode_us_per_sample_with_timeout(scan_mode, timeout)?,
            max_distance: self.get_scan_mode_max_distance_with_timeout(scan_mode, timeout)?,
            ans_type: self.get_scan_mode_ans_type_with_timeout(scan_mode, timeout)?,
            name: self.get_scan_mode_name_with_timeout(scan_mode, timeout)?,
        })
    }

    /// Retrieves information about all scan modes supported by the connected LIDAR.
    /// Uses the default timeout.
    pub fn get_all_supported_scan_modes(&mut self) -> Result<Vec<ScanMode>> {
        trace!("Getting all supported scan modes with default timeout");
        self.get_all_supported_scan_modes_with_timeout(RPLIDAR_DEFAULT_TIMEOUT)
    }

    /// Retrieves information about all scan modes supported by the connected LIDAR with a specified timeout.
    ///
    /// # Arguments
    ///
    /// * `timeout` - The maximum duration to wait for responses.
    pub fn get_all_supported_scan_modes_with_timeout(
        &mut self,
        timeout: Duration,
    ) -> Result<Vec<ScanMode>> {
        trace!(
            "Getting all supported scan modes with timeout {:?}",
            timeout
        );
        let scan_mode_count = self.get_scan_mode_count_with_timeout(timeout)?;
        let mut output: Vec<ScanMode> = Vec::with_capacity(scan_mode_count as usize);
        for i in 0..scan_mode_count {
            match self.get_scan_mode_with_timeout(i, timeout) {
                Ok(mode_info) => {
                    trace!("Successfully retrieved info for scan mode {}", i);
                    output.push(mode_info);
                }
                Err(e) => {
                    error!("Failed to retrieve info for scan mode {}: {:?}", i, e);
                    return Err(e);
                }
            }
        }
        trace!(
            "Successfully retrieved info for all {} scan modes",
            scan_mode_count
        );
        Ok(output)
    }

    /// Starts scanning using the typical scan mode and default options.
    /// Returns information about the scan mode that was actually started.
    pub fn start_scan(&mut self) -> Result<ScanMode> {
        trace!("Starting scan with default options");
        self.start_scan_with_options(&ScanOptions::default())
    }

    /// Starts scanning using the typical scan mode and default options, with a specified timeout.
    /// Returns information about the scan mode that was actually started.
    ///
    /// # Arguments
    ///
    /// * `timeout` - The maximum duration to wait for responses during mode negotiation.
    pub fn start_scan_with_timeout(&mut self, timeout: Duration) -> Result<ScanMode> {
        trace!(
            "Starting scan with default options and timeout {:?}",
            timeout
        );
        self.start_scan_with_options_and_timeout(&ScanOptions::default(), timeout)
    }

    /// Starts scanning with the specified options.
    /// If `options.scan_mode` is `None`, the typical scan mode is used.
    /// Returns information about the scan mode that was actually started.
    /// Uses the default timeout for mode negotiation.
    ///
    /// # Arguments
    ///
    /// * `options` - The `ScanOptions` to use for starting the scan.
    pub fn start_scan_with_options(&mut self, options: &ScanOptions) -> Result<ScanMode> {
        trace!("Starting scan with options: {:?}", options);
        self.start_scan_with_options_and_timeout(options, RPLIDAR_DEFAULT_TIMEOUT)
    }

    /// Starts scanning with the specified options and timeout.
    /// If `options.scan_mode` is `None`, the typical scan mode is used.
    /// Returns information about the scan mode that was actually started.
    ///
    /// # Arguments
    ///
    /// * `options` - The `ScanOptions` to use for starting the scan.
    /// * `timeout` - The maximum duration to wait for responses during mode negotiation.
    pub fn start_scan_with_options_and_timeout(
        &mut self,
        options: &ScanOptions,
        timeout: Duration,
    ) -> Result<ScanMode> {
        trace!(
            "Starting scan with options: {:?} and timeout {:?}",
            options,
            timeout
        );
        trace!("Resetting previous capsule cache");
        self.cached_prev_capsule = CachedPrevCapsule::None;

        let scan_mode = match options.scan_mode {
            Some(mode) => {
                trace!("Using specified scan mode ID: {}", mode);
                mode
            }
            None => {
                trace!("No scan mode specified, getting typical scan mode");
                let typical_mode = self.get_typical_scan_mode_with_timeout(timeout)?;
                trace!("Using typical scan mode ID: {}", typical_mode);
                typical_mode
            }
        };

        trace!("Getting full info for selected scan mode ID: {}", scan_mode);
        let scan_mode_info = self.get_scan_mode_with_timeout(scan_mode, timeout)?;
        trace!("Scan mode info retrieved: {:?}", scan_mode_info);

        match scan_mode {
            0 => {
                trace!(
                    "Using legacy start scan command (force={})",
                    options.force_scan
                );
                self.legacy_start_scan(options.force_scan)?
            }
            _ => {
                let payload = RplidarPayloadExpressScan {
                    work_mode: scan_mode as u8,
                    work_flags: options.options as u16,
                    param: 0,
                };
                trace!(
                    "Using express start scan command with payload: {:?}",
                    payload
                );
                self.start_express_scan(&payload)?;
            }
        }

        trace!(
            "Scan started successfully in mode '{}'",
            scan_mode_info.name
        );
        Ok(scan_mode_info)
    }

    /// use legacy command to start scan
    fn legacy_start_scan(&mut self, force_scan: bool) -> Result<()> {
        let cmd = if force_scan {
            RPLIDAR_CMD_FORCE_SCAN
        } else {
            RPLIDAR_CMD_SCAN
        };
        trace!("Sending legacy scan command: {:02X}", cmd);
        self.channel.write(&Message::new(cmd))?;
        trace!("Legacy scan command sent successfully");
        Ok(())
    }

    /// start express scan with options
    fn start_express_scan(&mut self, options: &RplidarPayloadExpressScan) -> Result<()> {
        let data = unsafe {
            transmute_copy::<
                RplidarPayloadExpressScan,
                [u8; std::mem::size_of::<RplidarPayloadExpressScan>()],
            >(options)
        };
        let cmd = RPLIDAR_CMD_EXPRESS_SCAN;
        trace!(
            "Sending express scan command ({:02X}) with payload: {:?}",
            cmd,
            data
        );
        self.channel.write(&Message::with_data(cmd, &data))?;
        trace!("Express scan command sent successfully");
        Ok(())
    }

    /// when hq measurement node received
    fn on_measurement_node_hq(&mut self, node: RplidarResponseMeasurementNodeHq) {
        // Copy fields from packed struct before logging
        let angle_z_q14 = node.angle_z_q14;
        let dist_mm_q2 = node.dist_mm_q2;
        let quality = node.quality;
        let flag = node.flag;
        trace!(
            "Received HQ node: angle_q14={}, dist_q2={}, quality={}, flag={:02X}",
            angle_z_q14,
            dist_mm_q2,
            quality,
            flag
        );
        let scan_point = ScanPoint::from(node);
        trace!(
            "Converted to ScanPoint: angle={:.2}, dist={:.4}, quality={}, sync={}",
            scan_point.angle(),
            scan_point.distance(),
            scan_point.quality,
            scan_point.is_sync()
        );
        self.cached_measurement_nodes.push_back(scan_point);
        trace!(
            "Added ScanPoint to cache (new size: {})",
            self.cached_measurement_nodes.len()
        );
    }

    /// when measurement node received
    fn on_measurement_node(&mut self, node: RplidarResponseMeasurementNode) {
        // Copy fields from packed struct before logging
        let sync_quality = node.sync_quality;
        let angle_q6_checkbit = node.angle_q6_checkbit;
        let distance_q2 = node.distance_q2;
        trace!(
            "Received legacy node: sync_q={}, angle_q6c={}, dist_q2={}",
            sync_quality,
            angle_q6_checkbit,
            distance_q2
        );
        // Conversion logic is complex, rely on on_measurement_node_hq logging
        self.on_measurement_node_hq(RplidarResponseMeasurementNodeHq {
            angle_z_q14: ((((node.angle_q6_checkbit as u32)
                >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT as u32)
                << 8)
                / 90) as u16,
            dist_mm_q2: node.distance_q2 as u32,
            flag: node.sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT,
            quality: (node.sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT as u8)
                << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT as u8,
        });
    }

    /// when capsuled measurement msg received
    fn on_measurement_capsuled_msg(&mut self, msg: &Message) -> Result<()> {
        trace!(
            "Received capsuled measurement message (type {:02X}, len {})",
            msg.cmd,
            msg.data.len()
        );
        check_sync_and_checksum(msg)?;
        trace!("Capsuled message sync and checksum OK");
        let parsed_capsule = parse_resp!(msg, RplidarResponseCapsuleMeasurementNodes)?;
        // Copy field from packed struct before logging
        let start_angle_sync_q6 = parsed_capsule.start_angle_sync_q6;
        trace!(
            "Parsed capsuled message: start_angle_q6={}",
            start_angle_sync_q6
        );
        self.on_measurement_capsuled(parsed_capsule);
        Ok(())
    }

    /// when capsuled measurement response received
    fn on_measurement_capsuled(&mut self, nodes: RplidarResponseCapsuleMeasurementNodes) {
        trace!("Processing parsed capsuled measurement response");
        let (parsed_nodes, new_cached_capsuled) = parse_capsuled(&self.cached_prev_capsule, nodes);
        trace!(
            "Parsed {} HQ nodes from previous capsule",
            parsed_nodes.len()
        );
        self.cached_prev_capsule = new_cached_capsuled;
        trace!("Updated cached_prev_capsule to Capsuled");

        for node in parsed_nodes {
            // on_measurement_node_hq already logs
            self.on_measurement_node_hq(node);
        }
    }

    /// when ultra capsuled measurement msg received
    fn on_measurement_ultra_capsuled_msg(&mut self, msg: &Message) -> Result<()> {
        trace!(
            "Received ultra capsuled measurement message (type {:02X}, len {})",
            msg.cmd,
            msg.data.len()
        );
        check_sync_and_checksum(msg)?;
        trace!("Ultra capsuled message sync and checksum OK");
        let parsed_capsule = parse_resp!(msg, RplidarResponseUltraCapsuleMeasurementNodes)?;
        // Copy field from packed struct before logging
        let start_angle_sync_q6 = parsed_capsule.start_angle_sync_q6;
        trace!(
            "Parsed ultra capsuled message: start_angle_q6={}",
            start_angle_sync_q6
        );
        self.on_measurement_ultra_capsuled(parsed_capsule);
        Ok(())
    }

    /// when ultra capsuled measurement response received
    fn on_measurement_ultra_capsuled(
        &mut self,
        nodes: RplidarResponseUltraCapsuleMeasurementNodes,
    ) {
        trace!("Processing parsed ultra capsuled measurement response");
        let (parsed_nodes, new_cached_capsuled) =
            parse_ultra_capsuled(&self.cached_prev_capsule, nodes);
        trace!(
            "Parsed {} HQ nodes from previous ultra capsule",
            parsed_nodes.len()
        );
        self.cached_prev_capsule = new_cached_capsuled;
        trace!("Updated cached_prev_capsule to UltraCapsuled");

        for node in parsed_nodes {
            // on_measurement_node_hq already logs
            self.on_measurement_node_hq(node);
        }
    }

    /// when hq capsuled measurement msg received
    fn on_measurement_hq_capsuled_msg(&mut self, msg: &Message) -> Result<()> {
        trace!(
            "Received HQ capsuled measurement message (type {:02X}, len {})",
            msg.cmd,
            msg.data.len()
        );
        check_sync_and_checksum_hq(msg)?;
        trace!("HQ capsuled message sync and checksum OK");
        let parsed_capsule = parse_resp!(msg, RplidarResponseHqCapsuledMeasurementNodes)?;
        // Copy field from packed struct before logging
        let timestamp = parsed_capsule.timestamp;
        trace!("Parsed HQ capsuled message: timestamp={}", timestamp);
        self.on_measurement_hq_capsuled(parsed_capsule);
        Ok(())
    }

    /// when hq capsuled measurement response received
    fn on_measurement_hq_capsuled(&mut self, nodes: RplidarResponseHqCapsuledMeasurementNodes) {
        trace!(
            "Processing parsed HQ capsuled measurement response ({} nodes)",
            nodes.nodes.len()
        );
        self.cached_prev_capsule = CachedPrevCapsule::None; // Reset cache as HQ is self-contained
        trace!("Reset cached_prev_capsule to None");
        for node in nodes.nodes.iter() {
            // on_measurement_node_hq already logs
            self.on_measurement_node_hq(*node);
        }
    }

    /// when dense capsuled measurement msg received
    fn on_measurement_dense_capsuled_msg(&mut self, msg: &Message) -> Result<()> {
        trace!(
            "Received dense capsuled measurement message (type {:02X}, len {})",
            msg.cmd,
            msg.data.len()
        );
        // Dense capsule uses the same sync/checksum mechanism as standard capsule
        check_sync_and_checksum(msg)?;
        trace!("Dense capsuled message sync and checksum OK");
        let parsed_capsule = parse_resp!(msg, RplidarResponseDenseCapsuleMeasurementNodes)?;
        // Copy field from packed struct before logging
        let start_angle_sync_q6 = parsed_capsule.start_angle_sync_q6;
        trace!(
            "Parsed dense capsuled message: start_angle_q6={}",
            start_angle_sync_q6
        );
        self.on_measurement_dense_capsuled(parsed_capsule);
        Ok(())
    }

    /// when dense capsuled measurement response received
    fn on_measurement_dense_capsuled(
        &mut self,
        nodes: RplidarResponseDenseCapsuleMeasurementNodes,
    ) {
        trace!("Processing parsed dense capsuled measurement response");
        let (parsed_nodes, new_cached_capsuled) =
            parse_dense_capsuled(&self.cached_prev_capsule, nodes);
        trace!(
            "Parsed {} HQ nodes from previous dense capsule",
            parsed_nodes.len()
        );
        self.cached_prev_capsule = new_cached_capsuled;
        trace!("Updated cached_prev_capsule to DenseCapsuled");

        for node in parsed_nodes {
            // on_measurement_node_hq already logs
            self.on_measurement_node_hq(node);
        }
    }

    /// Waits for and processes the next incoming packet of scan data.
    ///
    /// This method reads from the communication channel, decodes the message,
    /// and updates the internal cache of `ScanPoint`s based on the received data type.
    /// It handles different measurement response types (legacy, capsuled, ultra-capsuled, hq).
    ///
    /// # Arguments
    ///
    /// * `timeout` - The maximum duration to wait for a data packet.
    fn wait_scan_data_with_timeout(&mut self, timeout: Duration) -> Result<()> {
        trace!("Waiting for scan data with timeout {:?}", timeout);
        match self.channel.read_until(timeout) {
            Ok(Some(msg)) => {
                trace!(
                    "Received message type {:02X} while waiting for scan data",
                    msg.cmd
                );
                match msg.cmd {
                    RPLIDAR_ANS_TYPE_MEASUREMENT => {
                        trace!("Handling legacy measurement node");
                        match parse_resp!(msg, RplidarResponseMeasurementNode) {
                            Ok(node) => self.on_measurement_node(node),
                            Err(e) => {
                                error!("Failed to parse legacy measurement node: {:?}", e);
                                return Err(e);
                            }
                        }
                    }
                    RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED => {
                        trace!("Handling capsuled measurement message");
                        if let Err(e) = self.on_measurement_capsuled_msg(&msg) {
                            error!("Error handling capsuled message: {:?}", e);
                            return Err(e);
                        }
                    }
                    RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED_ULTRA => {
                        trace!("Handling ultra capsuled measurement message");
                        if let Err(e) = self.on_measurement_ultra_capsuled_msg(&msg) {
                            error!("Error handling ultra capsuled message: {:?}", e);
                            return Err(e);
                        }
                    }
                    RPLIDAR_ANS_TYPE_MEASUREMENT_HQ => {
                        trace!("Handling HQ capsuled measurement message");
                        if let Err(e) = self.on_measurement_hq_capsuled_msg(&msg) {
                            error!("Error handling HQ capsuled message: {:?}", e);
                            return Err(e);
                        }
                    }
                    SL_LIDAR_ANS_TYPE_MEASUREMENT_DENSE_CAPSULED => {
                        trace!("Handling dense capsuled measurement message");
                        if let Err(e) = self.on_measurement_dense_capsuled_msg(&msg) {
                            error!("Error handling dense capsuled message: {:?}", e);
                            return Err(e);
                        }
                    }
                    _ => {
                        error!("Unexpected response type during scan: {:02X}", msg.cmd);
                        return Err(Error::ProtocolError {
                            description: format!(
                                "unexpected response type during scan: {:02X}",
                                msg.cmd
                            ),
                        });
                    }
                }
                trace!("Finished handling received scan data message");
            }
            Ok(None) => {
                trace!("Timeout waiting for scan data, no message received");
            }
            Err(e) => {
                error!("Error waiting for scan data: {:?}", e);
                return Err(e);
            }
        }
        Ok(())
    }

    /// Grabs the next available scan point from the internal cache.
    ///
    /// If the cache is empty, this method will attempt to wait for and process new scan data
    /// using the default timeout.
    /// Returns an error if no scan point is available after waiting.
    pub fn grab_scan_point(&mut self) -> Result<ScanPoint> {
        trace!("Grabbing scan point with default timeout");
        self.grab_scan_point_with_timeout(RPLIDAR_DEFAULT_TIMEOUT)
    }

    /// Grabs the next available scan point from the internal cache with a specified timeout.
    ///
    /// If the cache is empty, this method will attempt to wait for and process new scan data
    /// using the provided `timeout`.
    /// Returns an error if no scan point is available after waiting.
    ///
    /// # Arguments
    ///
    /// * `timeout` - The maximum duration to wait for new scan data if the cache is empty.
    pub fn grab_scan_point_with_timeout(&mut self, timeout: Duration) -> Result<ScanPoint> {
        trace!("Grabbing scan point with timeout {:?}", timeout);
        if self.cached_measurement_nodes.is_empty() {
            trace!("Scan point cache is empty, waiting for data...");
            self.wait_scan_data_with_timeout(timeout)?;

            if self.cached_measurement_nodes.is_empty() {
                warn!("Timeout grabbing scan point: cache still empty after waiting");
                return Err(Error::OperationTimeout);
            }
            trace!(
                "Data received, cache size: {}",
                self.cached_measurement_nodes.len()
            );
        } else {
            trace!(
                "Scan point cache has {} items",
                self.cached_measurement_nodes.len()
            );
        }

        let point = self.cached_measurement_nodes.pop_front().unwrap();
        trace!(
            "Popped scan point from cache: angle={:.2}, dist={:.4}",
            point.angle(),
            point.distance()
        );
        Ok(point)
    }

    /// Grabs a complete 360-degree scan frame.
    ///
    /// This method collects scan points until a sync point (marking the start of a new revolution)
    /// is encountered after the first one. It uses a default timeout internally for waiting for data.
    /// The returned points are typically *not* sorted by angle. Use `utils::sort_scan` if needed.
    pub fn grab_scan(&mut self) -> Result<Vec<ScanPoint>> {
        let timeout = RPLIDAR_DEFAULT_TIMEOUT * 5;
        trace!("Grabbing full scan frame with timeout {:?}", timeout);
        self.grab_scan_with_timeout(timeout)
    }

    /// Grabs a complete 360-degree scan frame with a specified overall timeout.
    ///
    /// This method collects scan points until a sync point is encountered after the first one,
    /// or until the total `timeout` duration expires.
    /// The returned points are typically *not* sorted by angle. Use `utils::sort_scan` if needed.
    ///
    /// # Arguments
    ///
    /// * `timeout` - The maximum total duration to wait for a complete scan frame.
    pub fn grab_scan_with_timeout(&mut self, timeout: Duration) -> Result<Vec<ScanPoint>> {
        trace!(
            "Grabbing full scan frame with overall timeout {:?}",
            timeout
        );
        let deadline = Instant::now() + timeout;
        let mut first_sync_found = false;
        let mut scan_points = Vec::with_capacity(RPLIDAR_DEFAULT_CACHE_DEPTH);
        let mut points_since_last_sync = 0;
        let max_points_without_sync = 2000; // Adjust based on your LiDAR's expected points per revolution
        
        while Instant::now() < deadline {
            if self.cached_measurement_nodes.is_empty() {
                let remaining_time = deadline.saturating_duration_since(Instant::now());
                let read_timeout = std::cmp::min(remaining_time, RPLIDAR_DEFAULT_TIMEOUT / 5);
                if read_timeout == Duration::ZERO {
                    break;
                }
                
                trace!("Cache empty, waiting for data (timeout {:?})...", read_timeout);
                if let Err(e) = self.wait_scan_data_with_timeout(read_timeout) {
                    if !matches!(e, Error::OperationTimeout) {
                        error!("Error during wait_scan_data_with_timeout: {:?}", e);
                        return Err(e);
                    }
                    trace!("wait_scan_data_with_timeout timed out, continuing loop");
                    continue;
                }
                
                if self.cached_measurement_nodes.is_empty() {
                    continue;
                }
            }
            
            // Process all available points in the cache
            let mut i = 0;
            while i < self.cached_measurement_nodes.len() {
                let point = &self.cached_measurement_nodes[i];
                
                if point.is_sync() {
                    trace!("Found sync point at index {}", i);
                    
                    if !first_sync_found {
                        // First sync point found - start collecting points
                        first_sync_found = true;
                        scan_points.clear(); // Reset any previously collected points
                        
                        // Remove all points before and including this sync point
                        self.cached_measurement_nodes.drain(..=i);
                        i = 0; // Reset index since we've modified the cache
                        points_since_last_sync = 0;
                    } else {
                        // We found a second sync point after collecting some points
                        if scan_points.len() >= 50 { // Minimum reasonable number of points for a scan
                            // Include points up to but not including this sync point
                            for _ in 0..i {
                                if let Some(point) = self.cached_measurement_nodes.pop_front() {
                                    scan_points.push(point);
                                }
                            }
                            trace!("Full scan frame completed with {} points", scan_points.len());
                            return Ok(scan_points);
                        } else {
                            // Too few points between sync signals - this might be a false positive
                            // or a duplicate sync. Treat this as a new first sync.
                            trace!("Too few points ({}) between sync signals, treating as new first sync", scan_points.len());
                            scan_points.clear();
                            self.cached_measurement_nodes.drain(..=i);
                            i = 0;
                            points_since_last_sync = 0;
                        }
                    }
                } else if first_sync_found {
                    // Add point to our scan collection
                    scan_points.push(self.cached_measurement_nodes[i].clone());
                    self.cached_measurement_nodes.remove(i);
                    points_since_last_sync += 1;
                    
                    // Safety check - if we've collected a lot of points without seeing a second sync,
                    // we might have missed a sync signal. Return what we have.
                    if points_since_last_sync >= max_points_without_sync {
                        trace!("Collected {} points without finding second sync point, returning anyway", points_since_last_sync);
                        return Ok(scan_points);
                    }
                } else {
                    // No sync found yet, just advance
                    i += 1;
                }
            }
            
            // If we get here, we've processed all points in the cache without finding a complete frame
            if first_sync_found && !scan_points.is_empty() && Instant::now() > (deadline - timeout/10) {
                // If we're near the timeout and have some points collected, return them
                trace!("Near timeout with {} points collected, returning partial frame", scan_points.len());
                return Ok(scan_points);
            }
        }
        
        // If we have points but hit the deadline, return what we have
        if !scan_points.is_empty() {
            trace!("Timeout reached, returning {} collected points", scan_points.len());
            return Ok(scan_points);
        }
        
        warn!("Timeout grabbing full scan frame, no points collected");
        Err(Error::OperationTimeout)
    }

    /// Gets the health status of the RPLIDAR device.
    /// Uses the default timeout.
    pub fn get_device_health(&mut self) -> Result<Health> {
        trace!("Getting device health with default timeout");
        self.get_device_health_with_timeout(RPLIDAR_DEFAULT_TIMEOUT)
    }

    /// Gets the health status of the RPLIDAR device with a specified timeout.
    ///
    /// # Arguments
    ///
    /// * `timeout` - The maximum duration to wait for the health response.
    pub fn get_device_health_with_timeout(&mut self, timeout: Duration) -> Result<Health> {
        trace!("Getting device health with timeout {:?}", timeout);
        let cmd = RPLIDAR_CMD_GET_DEVICE_HEALTH;
        let msg = Message::new(cmd);
        trace!("Invoking command: {:02X}", cmd);
        match self.channel.invoke(&msg, timeout) {
            Ok(Some(resp_msg)) => {
                trace!(
                    "Received response for {:02X}, type: {:02X}, len: {}",
                    cmd,
                    resp_msg.cmd,
                    resp_msg.data.len()
                );
                let parsed_resp = handle_resp!(
                    RPLIDAR_ANS_TYPE_DEVHEALTH,
                    resp_msg,
                    RplidarResponseDeviceHealth
                );
                match parsed_resp {
                    Ok(resp) => {
                        // Copy fields from packed struct before logging
                        let status = resp.status;
                        let error_code = resp.error_code;
                        trace!(
                            "Parsed device health: status={}, error_code={:04X}",
                            status,
                            error_code
                        );
                        let health_status = match resp.status {
                            RPLIDAR_HEALTH_STATUS_OK => Health::Healthy,
                            RPLIDAR_HEALTH_STATUS_WARNING => Health::Warning(resp.error_code),
                            RPLIDAR_HEALTH_STATUS_ERROR => Health::Error(resp.error_code),
                            _ => {
                                warn!("Unknown health status code: {}", resp.status);
                                Health::Healthy
                            }
                        };
                        trace!("Converted to Health enum: {:?}", health_status);
                        Ok(health_status)
                    }
                    Err(e) => {
                        error!("Failed to parse device health response: {:?}", e);
                        Err(e)
                    }
                }
            }
            Ok(None) => {
                warn!("Timeout waiting for device health response");
                Err(Error::OperationTimeout)
            }
            Err(e) => {
                error!("Error invoking get_device_health: {:?}", e);
                Err(e)
            }
        }
    }

    /// Checks if the connected LIDAR system (including accessory board, if present)
    /// supports motor control via PWM commands. Uses the default timeout.
    pub fn check_motor_ctrl_support(&mut self) -> Result<bool> {
        trace!("Checking motor control support with default timeout");
        self.check_motor_ctrl_support_with_timeout(RPLIDAR_DEFAULT_TIMEOUT)
    }

    /// Checks if the connected LIDAR system supports motor control with a specified timeout.
    ///
    /// # Arguments
    ///
    /// * `timeout` - The maximum duration to wait for the response.
    pub fn check_motor_ctrl_support_with_timeout(&mut self, timeout: Duration) -> Result<bool> {
        trace!("Checking motor control support with timeout {:?}", timeout);
        let mut data = [0u8; 4];
        LittleEndian::write_u32(&mut data, 0u32); // Payload is typically zero for this command

        let cmd = RPLIDAR_CMD_GET_ACC_BOARD_FLAG;
        let msg = Message::with_data(cmd, &data);
        trace!("Invoking command {:02X} with payload {:?}", cmd, data);

        match self.channel.invoke(&msg, timeout) {
            Ok(Some(resp_msg)) => {
                trace!(
                    "Received response for {:02X}, type: {:02X}, len: {}",
                    cmd,
                    resp_msg.cmd,
                    resp_msg.data.len()
                );
                let parsed_resp = handle_resp!(RPLIDAR_ANS_TYPE_ACC_BOARD_FLAG, resp_msg, u32);
                match parsed_resp {
                    Ok(support_flag) => {
                        trace!("Parsed accessory board flag: {:08X}", support_flag);
                        let supported = (support_flag
                            & RPLIDAR_RESP_ACC_BOARD_FLAG_MOTOR_CTRL_SUPPORT_MASK)
                            == RPLIDAR_RESP_ACC_BOARD_FLAG_MOTOR_CTRL_SUPPORT_MASK;
                        trace!("Motor control support: {}", supported);
                        Ok(supported)
                    }
                    Err(e) => {
                        error!("Failed to parse accessory board flag response: {:?}", e);
                        Err(e)
                    }
                }
            }
            Ok(None) => {
                warn!("Timeout waiting for accessory board flag response");
                Err(Error::OperationTimeout)
            }
            Err(e) => {
                error!("Error invoking get_acc_board_flag: {:?}", e);
                Err(e)
            }
        }
    }
}

fn check_sync_and_checksum(msg: &Message) -> Result<()> {
    trace!("Checking sync and checksum for standard/ultra/dense capsule");
    if msg.data.len() < 2 {
        error!(
            "Data too short for sync/checksum check: len {}",
            msg.data.len()
        );
        return Err(Error::ProtocolError {
            description: "data too short".to_owned(),
        });
    }

    let sync1 = msg.data[0] >> 4;
    let sync2 = msg.data[1] >> 4;
    trace!("Sync bytes: {:X} {:X}", sync1, sync2);

    if sync1 != RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1 {
        error!(
            "Miss sync 1: expected {:X}, got {:X}",
            RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1, sync1
        );
        return Err(Error::ProtocolError {
            description: "miss sync 1".to_owned(),
        });
    }

    if sync2 != RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2 {
        error!(
            "Miss sync 2: expected {:X}, got {:X}",
            RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2, sync2
        );
        return Err(Error::ProtocolError {
            description: "miss sync 2".to_owned(),
        });
    }

    let recv_checksum = (msg.data[0] & 0xf) | (msg.data[1] << 4);
    let mut checksum = Checksum::new();
    checksum.push_slice(&msg.data[2..]);
    let calculated_checksum = checksum.checksum();
    trace!(
        "Checksum: received {:02X}, calculated {:02X}",
        recv_checksum,
        calculated_checksum
    );

    if calculated_checksum != recv_checksum {
        error!(
            "Checksum mismatch: received {:02X}, calculated {:02X}",
            recv_checksum, calculated_checksum
        );
        Err(Error::ProtocolError {
            description: "checksum mismatch".to_owned(),
        })
    } else {
        trace!("Sync and checksum OK");
        Ok(())
    }
}

fn check_sync_and_checksum_hq(msg: &Message) -> Result<()> {
    trace!("Checking sync and checksum for HQ capsule");
    let expected_len = std::mem::size_of::<RplidarResponseHqCapsuledMeasurementNodes>();
    if msg.data.len() != expected_len {
        error!(
            "HQ capsule data length mismatch: expected {}, got {}",
            expected_len,
            msg.data.len()
        );
        return Err(Error::ProtocolError {
            description: "HQ capsule data length mismatch".to_owned(),
        });
    }

    let sync_byte = msg.data[0];
    trace!("Sync byte: {:02X}", sync_byte);
    if sync_byte != RPLIDAR_RESP_MEASUREMENT_HQ_SYNC {
        error!(
            "HQ capsule sync mismatch: expected {:02X}, got {:02X}",
            RPLIDAR_RESP_MEASUREMENT_HQ_SYNC, sync_byte
        );
        return Err(Error::ProtocolError {
            description: "HQ capsule sync mismatch".to_owned(),
        });
    }

    let data_len = msg.data.len();
    let checksum_payload = &msg.data[..data_len - 4];
    let calculated_checksum = crc32fast::hash(checksum_payload);
    let received_checksum = LittleEndian::read_u32(&msg.data[data_len - 4..]);
    trace!(
        "CRC32 Checksum: received {:08X}, calculated {:08X}",
        received_checksum,
        calculated_checksum
    );

    if calculated_checksum != received_checksum {
        error!(
            "HQ capsule checksum mismatch: received {:08X}, calculated {:08X}",
            received_checksum, calculated_checksum
        );
        Err(Error::ProtocolError {
            description: "HQ capsule checksum mismatch".to_owned(),
        })
    } else {
        trace!("HQ Sync and checksum OK");
        Ok(())
    }
}
