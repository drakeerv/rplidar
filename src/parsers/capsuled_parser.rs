use crate::answers::*;
use crate::CachedPrevCapsule;
use log::trace;
use log::warn;

pub const ANGLE_360_Q8: u32 = 360u32 << 8;
pub const ANGLE_360_Q16: u32 = 360u32 << 16;

#[inline]
fn get_start_angle_q8(nodes: &RplidarResponseCapsuleMeasurementNodes) -> u32 {
    ((nodes.start_angle_sync_q6 & 0x7fffu16) as u32) << 2
}

#[inline]
pub fn angle_diff_q8(prev_q8: u32, cur_q8: u32) -> u32 {
    if prev_q8 > cur_q8 {
        ANGLE_360_Q8 + cur_q8 - prev_q8
    } else {
        cur_q8 - prev_q8
    }
}

pub struct ParsedNode {
    pub dist_q2: u32,
    pub angle_offset_q3: u32,
}

#[inline]
fn parse_cabin(cabin: &RplidarResponseCabinNodes) -> [ParsedNode; 2] {
    // trace!("Parsing standard cabin: dist_angle1={:04X}, dist_angle2={:04X}, offset_angles={:02X}", cabin.distance_angle_1, cabin.distance_angle_2, cabin.offset_angles_q3);
    let dist_q2_1 = cabin.distance_angle_1 & 0xfffc;
    let dist_q2_2 = cabin.distance_angle_2 & 0xfffc;

    let angle_offset_q3_1 =
        (cabin.offset_angles_q3 & 0xf) as u16 | ((cabin.distance_angle_1 & 0x3) << 4);
    let angle_offset_q3_2 =
        (cabin.offset_angles_q3 >> 4) as u16 | ((cabin.distance_angle_2 & 0x3) << 4);

    // trace!("  -> Node 1: dist_q2={}, offset_q3={}", dist_q2_1, angle_offset_q3_1);
    // trace!("  -> Node 2: dist_q2={}, offset_q3={}", dist_q2_2, angle_offset_q3_2);

    [
        ParsedNode {
            dist_q2: dist_q2_1 as u32,
            angle_offset_q3: angle_offset_q3_1 as u32,
        },
        ParsedNode {
            dist_q2: dist_q2_2 as u32,
            angle_offset_q3: angle_offset_q3_2 as u32,
        },
    ]
}

#[inline]
pub fn check_sync(cur_angle_q16: u32, angle_inc_q16: u32) -> bool {
    ((cur_angle_q16 + angle_inc_q16) % ANGLE_360_Q16) < angle_inc_q16
}

#[inline]
pub fn angle_q6_to_angle_z_q14(angle_q6: u32) -> u16 {
    ((angle_q6 << 8) / 90) as u16
}

#[inline]
pub fn generate_quality(dist_q2: u32) -> u8 {
    if dist_q2 != 0 {
        0x2fu8 << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT
    } else {
        0u8
    }
}

#[inline]
pub fn generate_flag(sync: bool) -> u8 {
    if sync {
        1u8
    } else {
        0u8
    }
}

#[inline]
pub fn to_hq(
    node: &ParsedNode,
    cur_angle_raw_q16: u32,
    angle_inc_q16: u32,
) -> RplidarResponseMeasurementNodeHq {
    // trace!("Converting ParsedNode (dist_q2={}, offset_q3={}) to HQ node", node.dist_q2, node.angle_offset_q3);
    // trace!("  cur_angle_raw_q16={}, angle_inc_q16={}", cur_angle_raw_q16, angle_inc_q16);
    let angle_q6 = cur_angle_raw_q16 - ((node.angle_offset_q3 << 13) >> 10);
    let sync = check_sync(cur_angle_raw_q16, angle_inc_q16);
    // trace!("  Calculated angle_q6={}, sync={}", angle_q6, sync);

    // trace!("  Result HQ node: angle_z_q14={}, dist_mm_q2={}, quality={}, flag={:02X}", hq_node.angle_z_q14, hq_node.dist_mm_q2, hq_node.quality, hq_node.flag);
    RplidarResponseMeasurementNodeHq {
        angle_z_q14: angle_q6_to_angle_z_q14(angle_q6),
        dist_mm_q2: node.dist_q2,
        quality: generate_quality(node.dist_q2),
        flag: generate_flag(sync),
    }
}

pub fn parse_capsuled(
    cached_prev: &CachedPrevCapsule,
    nodes: RplidarResponseCapsuleMeasurementNodes,
) -> (Vec<RplidarResponseMeasurementNodeHq>, CachedPrevCapsule) {
    // Copy field before logging
    let start_angle_sync_q6 = nodes.start_angle_sync_q6;
    trace!(
        "Parsing standard capsule: start_angle_q6={:04X}",
        start_angle_sync_q6
    );
    if let CachedPrevCapsule::Capsuled(prev_capsule) = cached_prev {
        // Copy field before logging
        let prev_start_angle_sync_q6 = prev_capsule.start_angle_sync_q6;
        trace!(
            "Using previous standard capsule: start_angle_q6={:04X}",
            prev_start_angle_sync_q6
        );
        let mut output_nodes: Vec<RplidarResponseMeasurementNodeHq> = Vec::with_capacity(32);

        let cur_start_angle_q8 = get_start_angle_q8(&nodes);
        let prev_start_angle_q8 = get_start_angle_q8(prev_capsule);
        trace!(
            "Current start angle Q8: {}, Previous start angle Q8: {}",
            cur_start_angle_q8,
            prev_start_angle_q8
        );

        let diff_angle_q8 = angle_diff_q8(prev_start_angle_q8, cur_start_angle_q8);
        trace!("Difference angle Q8: {}", diff_angle_q8);

        let angle_inc_q16 = diff_angle_q8 << 3; // Angle increment per node pair (cabin)
        let mut cur_angle_raw_q16 = prev_start_angle_q8 << 8;
        trace!(
            "Angle increment Q16 (per node pair): {}, Initial angle Q16: {}",
            angle_inc_q16,
            cur_angle_raw_q16
        );

        for cabin in prev_capsule.cabins.iter() {
            // Prefix i with _
            // trace!("Processing cabin {}", _i);
            let parsed_nodes = parse_cabin(cabin); // Logs internally

            for node in parsed_nodes.iter() {
                // Prefix j with _
                // trace!("  Generating HQ node {} for cabin {}", _j, _i);
                output_nodes.push(to_hq(node, cur_angle_raw_q16, angle_inc_q16)); // Logs internally
                cur_angle_raw_q16 += angle_inc_q16; // Increment angle for each *node*
                                                    // trace!("  Incremented cur_angle_raw_q16 to {}", cur_angle_raw_q16);
            }
        }
        trace!(
            "Finished parsing standard capsule, generated {} nodes",
            output_nodes.len()
        );
        (output_nodes, CachedPrevCapsule::Capsuled(nodes))
    } else {
        warn!("Previous capsule was not 'Capsuled' (was {:?}), cannot calculate angles for current capsule. Caching current and returning empty.", cached_prev);
        (Vec::new(), CachedPrevCapsule::Capsuled(nodes))
    }
}
