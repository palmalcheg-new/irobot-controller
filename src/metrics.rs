use serde::{Deserialize, Serialize};
use std::fmt::Display;

pub type Meters = f64;
pub type Radians = f64;
pub type Seconds = f64;

#[derive(Debug, Serialize, Deserialize, Clone, Copy)]
pub struct Position {
    pub x: Meters,
    pub y: Meters,
}

impl Display for Position {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "x: {} meters, y: {} meters", self.x, self.y)
    }
}

impl Position {
    pub fn new() -> Self {
        Self { x: 0.0, y: 0.0 }
    }

    pub fn update(&mut self, delta_distance: Meters, theta: Radians) {
        self.x += delta_distance * theta.cos();
        self.y += delta_distance * theta.sin();
    }

    pub fn reset(&mut self) {
        self.x = 0.0;
        self.y = 0.0;
    }
}

#[derive(Debug)]
pub struct Orientation {
    pub theta: Radians,
}

impl Display for Orientation {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "theta: {} radians", self.theta)
    }
}

impl Orientation {
    pub fn new() -> Self {
        Self { theta: 0.0 }
    }

    pub fn update(&mut self, delta_theta: Radians) {
        self.theta += delta_theta;
    }

    pub fn reset(&mut self) {
        self.theta = 0.0;
    }
}

#[derive(Debug)]
pub struct WheelState {
    pub left_angle: Meters,
    pub right_angle: Meters,
    pub wheel_base: Meters,
    pub wheel_radius: Meters,
}

impl WheelState {
    pub fn new(wheel_base: Meters, wheel_radius: Meters) -> Self {
        Self {
            left_angle: 0.0,
            right_angle: 0.0,
            wheel_base,
            wheel_radius,
        }
    }

    pub fn update_angles(
        &mut self,
        new_left_angle: Radians,
        new_right_angle: Radians,
    ) -> (Meters, Meters) {
        let left_delta_angle = new_left_angle - self.left_angle;
        let right_delta_angle = new_right_angle - self.right_angle;

        self.left_angle = new_left_angle;
        self.right_angle = new_right_angle;

        let left_delta_distance = left_delta_angle * self.wheel_radius;
        let right_delta_distance = right_delta_angle * self.wheel_radius;

        (left_delta_distance, right_delta_distance)
    }

    pub fn reset(&mut self) {
        self.left_angle = 0.0;
        self.right_angle = 0.0;
    }
}
#[derive(Debug)]
pub struct Odometry {
    pub position: Position,
    pub orientation: Orientation,
    pub wheel_state: WheelState,
}

impl Display for Odometry {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "[{} , {}]", self.position, self.orientation)
    }
}

impl Odometry {
    pub fn new(wheel_radius: Meters, wheel_base: Meters) -> Self {
        Self {
            position: Position::new(),
            orientation: Orientation::new(),
            wheel_state: WheelState::new(wheel_base, wheel_radius),
        }
    }

    pub fn update(&mut self, new_left_angle: Radians, new_right_angle: Radians) {
        let (left_delta_distance, right_delta_distance) = self
            .wheel_state
            .update_angles(new_left_angle, new_right_angle);

        let linear_displacement = (left_delta_distance + right_delta_distance) / 2.0;
        let angular_displacement =
            (right_delta_distance - left_delta_distance) / self.wheel_state.wheel_base;
        self.update_pose(linear_displacement, angular_displacement);
    }

    fn update_pose(&mut self, linear_displacement: Meters, angular_displacement: Radians) {
        self.orientation.update(angular_displacement);
        self.position
            .update(linear_displacement, self.orientation.theta);
    }

    pub fn reset(&mut self) {
        self.position.reset();
        self.orientation.reset();
        // self.wheel_state.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_odometry_update() {
        let mut odometry = Odometry::new(0.032, 0.3); // Assuming wheel radius is 0.032 meters
        odometry.update(1.0, 1.0);
        assert!((odometry.position.x - 0.032).abs() < 1e-5);
        assert!((odometry.position.y).abs() < 1e-5);
        assert!((odometry.orientation.theta).abs() < 1e-5);
    }

    #[test]
    fn test_odometry_reset() {
        let mut odometry = Odometry::new(0.032, 0.3);
        odometry.update(1.0, 1.0);
        odometry.reset();
        assert_eq!(odometry.position.x, 0.0);
        assert_eq!(odometry.position.y, 0.0);
        assert_eq!(odometry.orientation.theta, 0.0);
    }

    #[test]
    fn test_update_pose() {
        let mut odometry = Odometry::new(0.032, 0.3);
        let linear_displacement = 0.032;
        let angular_displacement = std::f64::consts::PI / 4.0;
        odometry.update_pose(linear_displacement, angular_displacement);
        assert!((odometry.orientation.theta - angular_displacement).abs() < 1e-5);
        assert!(
            (odometry.position.x - linear_displacement * angular_displacement.cos()).abs() < 1e-5
        );
        assert!(
            (odometry.position.y - linear_displacement * angular_displacement.sin()).abs() < 1e-5
        );
    }
}
