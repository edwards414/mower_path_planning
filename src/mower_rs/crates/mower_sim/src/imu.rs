//! The WIT IMU (/dev/imu_usb, 9600 baud): 0x51 acceleration, 0x52 angular
//! velocity, 0x53 angle, 0x54 magnetometer, ten cycles a second, computed
//! from the truth world (not from /odom, which is what fake_sensors.py
//! --follow-odom closes its loop on).
//!
//! The conventions are fake_sensors.py's, which the production EKFs were
//! run against closed loop: roll and pitch read 0 and the accelerometer +1 g
//! on z (with the EKF's gravity removal that leaves the horizontal
//! acceleration), and the mount comes from the URDF (geometry::Mount). For
//! today's URDF (imu_link upside down under a base_link turned +90 deg) that
//! is fake_sensors.py's rule: base yaw = IMU yaw + 90 deg, base yaw rate =
//! -IMU z rate. In general the IMU's heading reads the robot's minus the
//! mount's yaw when upside down (plus it when upright), and rates and
//! accelerations are the body's, rotated into imu_link.

use crate::geometry::{mul_vec, transpose, Mount};
use crate::world::World;

/// The driver's scales (mower_imu/src/wit.rs).
pub const G: f64 = 9.8;
pub const ACCEL_FULL_SCALE: f64 = 16.0 * G;
pub const GYRO_FULL_SCALE_DEG_S: f64 = 2000.0;

pub fn frame(kind: u8, regs: [i16; 4]) -> [u8; 11] {
    let mut f = [0u8; 11];
    f[0] = 0x55;
    f[1] = kind;
    for (i, r) in regs.iter().enumerate() {
        f[2 + 2 * i..4 + 2 * i].copy_from_slice(&r.to_le_bytes());
    }
    f[10] = f[..10].iter().map(|b| *b as u32).sum::<u32>() as u8;
    f
}

fn reg(value: f64, full_scale: f64) -> i16 {
    (value / full_scale * 32768.0).round().clamp(-32767.0, 32767.0) as i16
}

/// The IMU's heading, degrees in [-180, 180).
pub fn heading_deg(world: &World, mount: &Mount) -> f64 {
    let sign = if mount.imu_flipped { -1.0 } else { 1.0 };
    let deg = world.yaw.to_degrees() + sign * mount.imu_yaw.to_degrees();
    (deg + 180.0).rem_euclid(360.0) - 180.0
}

/// One cycle: the four frames, 44 bytes.
pub fn cycle(world: &World, mount: &Mount) -> Vec<u8> {
    let to_imu = transpose(&mount.imu);
    let a = mul_vec(&to_imu, [world.accel[0], world.accel[1], 0.0]);
    let w = mul_vec(&to_imu, [0.0, 0.0, world.w]);
    let accel = [reg(a[0], ACCEL_FULL_SCALE), reg(a[1], ACCEL_FULL_SCALE), reg(G, ACCEL_FULL_SCALE), 0];
    let gyro = [0, 1, 2].map(|i| reg(w[i].to_degrees(), GYRO_FULL_SCALE_DEG_S));
    let angle = [0, 0, reg(heading_deg(world, mount), 180.0), 0];
    [frame(0x51, accel), frame(0x52, [gyro[0], gyro[1], gyro[2], 0]), frame(0x53, angle), frame(0x54, [100, -200, 300, 0])].concat()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Wheels;
    use crate::world::Battery;

    use crate::wit_parser as wit;

    const REPO_URDF: &str = include_str!("../../../../mower_description/mower_robot/robot.urdf.xacro");

    fn parse(bytes: &[u8]) -> (wit::Parser, Vec<wit::Frame>) {
        let mut p = wit::Parser::default();
        let frames = bytes.iter().filter_map(|b| p.push(*b)).collect();
        (p, frames)
    }

    #[test]
    fn a_cycle_is_what_the_driver_reads_and_fake_sensors_wrote() {
        let mount = Mount::from_urdf(REPO_URDF).unwrap();
        let mut world = World::new(Wheels::from_params(None).unwrap(), 120f64.to_radians(), Battery::at_rest(24.3, 10.0));
        world.w = 0.5;
        world.accel = [0.3, 0.1];
        let bytes = cycle(&world, &mount);
        assert_eq!(bytes.len(), 44, "within the driver's 88-byte backlog limit");
        let (p, frames) = parse(&bytes);
        assert_eq!(frames, [wit::Frame::Acceleration, wit::Frame::AngularVelocity, wit::Frame::Angle, wit::Frame::Magnetometer]);
        assert_eq!(p.checksum_failures, 0);
        // fake_sensors.py's YAW_DEG = 30 for a robot heading 120 deg (ENU)
        assert!((p.angle_degree[2] - 30.0).abs() < 0.01, "{:?}", p.angle_degree);
        assert_eq!((p.angle_degree[0], p.angle_degree[1]), (0.0, 0.0));
        // upside down: the z rate is the body's, negated
        assert!((p.angular_velocity[2] + 0.5).abs() < 0.002, "{:?}", p.angular_velocity);
        assert!(p.angular_velocity[0].abs() < 1e-9 && p.angular_velocity[1].abs() < 1e-9);
        // +1 g on z; forward / left acceleration rotated into imu_link
        // (base_footprint x is imu_link y, base_footprint y is imu_link x)
        assert!((p.acceleration[2] - 9.8).abs() < 0.01);
        assert!((p.acceleration[1] - 0.3).abs() < 0.01 && (p.acceleration[0] - 0.1).abs() < 0.01, "{:?}", p.acceleration);
        assert_eq!(p.magnetometer, [100, -200, 300]);
    }

    #[test]
    fn the_heading_wraps_and_an_upright_mount_adds_its_yaw() {
        let mut m = Mount::from_urdf(REPO_URDF).unwrap();
        let mut world = World::new(Wheels::from_params(None).unwrap(), -100f64.to_radians(), Battery::at_rest(24.3, 10.0));
        // the URDF's 1.5708 rad is 90.0002 deg
        assert!((heading_deg(&world, &m) - 170.0).abs() < 1e-3, "{}", heading_deg(&world, &m));
        m.imu_flipped = false;
        world.yaw = 100f64.to_radians();
        assert!((heading_deg(&world, &m) + 170.0).abs() < 1e-3);
        let (p, _) = parse(&cycle(&world, &m));
        assert!((p.angle_degree[2] + 170.0).abs() < 0.01);
    }
}
