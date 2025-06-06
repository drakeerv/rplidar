use std::f64::consts::PI;

use crate::answers::{
    RplidarResponseMeasurementNodeHq, RplidarResponseUltraCapsuleMeasurementNodes,
};
use crate::parsers::capsuled_parser::{angle_diff_q8, check_sync, generate_flag, generate_quality};
use crate::CachedPrevCapsule;
use log::trace;
use log::warn;

struct ParsedNode {
    pub dist_q2: u32,
    pub angle_offset_q16: i32,
}

#[inline]
fn get_start_angle_q8(nodes: &RplidarResponseUltraCapsuleMeasurementNodes) -> u32 {
    ((nodes.start_angle_sync_q6 & 0x7fffu16) as u32) << 2
}

#[inline]
fn deg_to_rad_q16(deg: f64) -> f64 {
    deg * PI * 65536f64 / 180f64
}

// Note: calc_angle_offset_q16 involves floating point and might not benefit much from inlining,
// but we add it for consistency.
#[inline]
fn calc_angle_offset_q16(dist: u32) -> i32 {
    if dist >= 50 * 4 {
        const K1: i32 = 98361;
        let k2 = K1 / (dist as i32);

        (deg_to_rad_q16(8f64) - ((k2 << 6) as f64) - (((k2 * k2 * k2) / 98304) as f64)) as i32
    } else {
        deg_to_rad_q16(7.5f64) as i32
    }
}

const RPLIDAR_VARBIT_SCALE_TARGET_BASES: [u32; 5] = [1 << 14, 1 << 12, 1 << 11, 1 << 9, 0];

const RPLIDAR_VARBIT_SCALE_SCALE_BASES: [i32; 5] = [3328, 1792, 1280, 512, 0];

const RPLIDAR_VARBIT_SCALE_LEVELS: [u32; 5] = [4, 3, 2, 1, 0];

/// decode varbit encoded distance to flat distance and scale level
#[inline]
fn varbit_scale_decode(scaled: u32) -> (u32, u32) {
    for i in 0..RPLIDAR_VARBIT_SCALE_SCALE_BASES.len() {
        let scale_base = RPLIDAR_VARBIT_SCALE_SCALE_BASES[i];
        let target_base = RPLIDAR_VARBIT_SCALE_TARGET_BASES[i];
        let scale_level = RPLIDAR_VARBIT_SCALE_LEVELS[i];

        let remain = (scaled as i32) - scale_base;
        if remain >= 0 {
            let decoded_val = target_base + ((remain << scale_level) as u32);
            return (decoded_val, scale_level);
        }
    }
    (0, 0)
}

/// parse ultra capsuled cabin to (major, predict1, predict2)
#[inline]
fn parse_cabin(cabin: u32) -> (u32, i32, i32) {
    let major = cabin & 0xfffu32;
    let predict1 = ((cabin << 10) as i32) >> 22;
    let predict2 = (cabin as i32) >> 22;
    (major, predict1, predict2)
}

#[inline]
fn predict(base: u32, predict: i32, scale_lvl: u32) -> u32 {
    if ((predict as u32) == 0xfffffe00u32) || (predict == 0x1ffi32) {
        0u32
    } else {
        (((predict << scale_lvl) + (base as i32)) as u32) << 2 // Result is Q2
    }
}

// Note: generate_nodes is larger, but still primarily calculation. Inlining might help.
#[inline]
fn generate_nodes(
    dist_major: u32,
    next_major: u32,
    dist_predict1: i32,
    dist_predict2: i32,
) -> [ParsedNode; 3] {
    let (dist_major, scale_lvl_1) = varbit_scale_decode(dist_major);
    let (next_major, scale_lvl_2) = varbit_scale_decode(next_major);

    let (dist_base1, dist_base2, scale_lvl_1) = if (dist_major == 0) && (next_major != 0) {
        (next_major, next_major, scale_lvl_2)
    } else {
        (dist_major, next_major, scale_lvl_1)
    };

    let dist0 = dist_major << 2;
    let dist1 = predict(dist_base1, dist_predict1, scale_lvl_1);
    let dist2 = predict(dist_base2, dist_predict2, scale_lvl_2);

    [
        ParsedNode {
            dist_q2: dist0,
            angle_offset_q16: calc_angle_offset_q16(dist0),
        },
        ParsedNode {
            dist_q2: dist1,
            angle_offset_q16: calc_angle_offset_q16(dist1),
        },
        ParsedNode {
            dist_q2: dist2,
            angle_offset_q16: calc_angle_offset_q16(dist2),
        },
    ]
}

#[inline]
fn angle_q16_to_angle_z_q14(angle_q16: u32) -> u16 {
    ((angle_q16 / 90) >> 2) as u16
}

#[inline]
fn to_hq(
    node: &ParsedNode,
    cur_angle_raw_q16: u32,
    angle_inc_q16: u32,
) -> RplidarResponseMeasurementNodeHq {
    let angle_q16 =
        (cur_angle_raw_q16 as i32) - (node.angle_offset_q16 as f64 * 180f64 / PI) as i32;
    let sync = check_sync(cur_angle_raw_q16, angle_inc_q16);

    RplidarResponseMeasurementNodeHq {
        angle_z_q14: angle_q16_to_angle_z_q14(angle_q16 as u32),
        dist_mm_q2: node.dist_q2,
        quality: generate_quality(node.dist_q2),
        flag: generate_flag(sync),
    }
}

/// Parses a `RplidarResponseUltraCapsuleMeasurementNodes` response packet.
///
/// Requires the previously received ultra capsule (`cached_prev`) to calculate angular increments and decode distances.
/// Returns a vector of `RplidarResponseMeasurementNodeHq` points derived from the *previous* ultra capsule,
/// and the current ultra capsule to be cached for the next iteration.
///
/// # Arguments
///
/// * `cached_prev` - The `CachedPrevCapsule` containing the data from the immediately preceding ultra capsuled response packet.
/// * `nodes` - The current `RplidarResponseUltraCapsuleMeasurementNodes` packet to parse.
pub fn parse_ultra_capsuled(
    cached_prev: &CachedPrevCapsule,
    nodes: RplidarResponseUltraCapsuleMeasurementNodes,
) -> (Vec<RplidarResponseMeasurementNodeHq>, CachedPrevCapsule) {
    let start_angle_sync_q6 = nodes.start_angle_sync_q6;
    trace!(
        "Parsing ultra capsule: start_angle_q6={:04X}",
        start_angle_sync_q6
    );
    if let CachedPrevCapsule::UltraCapsuled(prev_capsule) = cached_prev {
        let prev_start_angle_sync_q6 = prev_capsule.start_angle_sync_q6;
        trace!(
            "Using previous ultra capsule: start_angle_q6={:04X}",
            prev_start_angle_sync_q6
        );
        let mut output_nodes: Vec<RplidarResponseMeasurementNodeHq> = Vec::with_capacity(32 * 3);

        let cur_start_angle_q8 = get_start_angle_q8(&nodes);
        let prev_start_angle_q8 = get_start_angle_q8(prev_capsule);
        trace!(
            "Current start angle Q8: {}, Previous start angle Q8: {}",
            cur_start_angle_q8,
            prev_start_angle_q8
        );

        let diff_angle_q8 = angle_diff_q8(prev_start_angle_q8, cur_start_angle_q8);
        trace!("Difference angle Q8: {}", diff_angle_q8);

        let angle_inc_q16 = (diff_angle_q8 << 3) / 3;
        let mut cur_angle_raw_q16 = prev_start_angle_q8 << 8;
        trace!(
            "Angle increment Q16 (per node): {}, Initial angle Q16: {}",
            angle_inc_q16,
            cur_angle_raw_q16
        );

        let (mut cur_major, mut cur_predict1, mut cur_predict2) =
            parse_cabin(prev_capsule.ultra_cabins[0]);

        const CABIN_COUNT: usize = 32;

        for i in 0..CABIN_COUNT {
            let next_cabin_idx = if i == CABIN_COUNT - 1 { 0 } else { i + 1 };
            let next_cabin_val = if i == CABIN_COUNT - 1 {
                nodes.ultra_cabins[0]
            } else {
                prev_capsule.ultra_cabins[next_cabin_idx]
            };

            let (next_major, next_predict1, next_predict2) = parse_cabin(next_cabin_val);

            let parsed_nodes = generate_nodes(cur_major, next_major, cur_predict1, cur_predict2);

            for node in parsed_nodes.iter() {
                output_nodes.push(to_hq(node, cur_angle_raw_q16, angle_inc_q16));
                cur_angle_raw_q16 += angle_inc_q16;
            }

            cur_major = next_major;
            cur_predict1 = next_predict1;
            cur_predict2 = next_predict2;
        }
        trace!(
            "Finished parsing ultra capsule, generated {} nodes",
            output_nodes.len()
        );
        (output_nodes, CachedPrevCapsule::UltraCapsuled(nodes))
    } else {
        warn!("Previous capsule was not 'UltraCapsuled' (was {:?}), cannot calculate angles/distances for current capsule. Caching current and returning empty.", cached_prev);
        (Vec::new(), CachedPrevCapsule::UltraCapsuled(nodes))
    }
}
