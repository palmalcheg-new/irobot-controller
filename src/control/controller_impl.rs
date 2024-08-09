use crate::metrics::*;
use crate::control::*;
use kfilter::Kalman1M;
use core::f64;
use std::{
    fmt::*,
    sync::{Arc, Mutex},
};

use na::{
    allocator::Allocator,
    convert,
    dimension::{DimMin, U2, U4},
    DefaultAllocator, OMatrix, RealField,
    Matrix1x2, Matrix1x4, Matrix4, Vector2, Vector4
};

#[derive(Debug)]
pub struct PID {
    kp: f64,
    ki: f64,
    kd: f64,
    integral: f64,
    previous_error: f64,
}

type MyType = f64;

pub struct KalmalFilter<R: RealField> {
    motion:  motion_model::ConstantVelocity2DModel<R>,
    measurement: linear_observation_model::PositionObservationModel<R>,
    previous_estimate: StateAndCovariance<R, U4>,
}

impl KalmalFilter<MyType> {
    fn new(&self, var : MyType) -> Self {
        let true_initial_state = Vector4::<MyType>::new(0.0, 0.0, 10.0, -5.0);
        #[rustfmt::skip]
        let initial_covariance 
               = Matrix4::<MyType>::new(0.1, 0.0, 0.0, 0.0,
                                        0.0, 0.1, 0.0, 0.0,
                                        0.0, 0.0, 0.1, 0.0,
                                        0.0, 0.0, 0.0, 0.1);

         Self {
            motion :  motion_model::ConstantVelocity2DModel::new(var, 100.0),
            measurement : linear_observation_model::PositionObservationModel::new(var),
            previous_estimate: adskalman::StateAndCovariance::new(
                true_initial_state,
                initial_covariance,
            ),
        }
    }    
    fn predict(&mut self, linear_velocity: f64, angular_velocity: f64, dt: f64) {
        let kf: KalmanFilterNoControl<f64, na::U4, na::U2> =
            KalmanFilterNoControl::new(&self.motion, &self.measurement);

            

            kf.step(previous_estimate, observation);
        // let mut kalman_state = self.kalman_filter.state().clone();

        // kalman_state.0 += linear_velocity * dt * kalman_state.2.cos();
        // kalman_state.1 += linear_velocity * dt * kalman_state.2.sin();
        // kalman_state.2 += angular_velocity * dt;

        // kalman_state.p = kalman_state.p + Matrix3::from_diagonal_element(0.1); // Process noise

        // self.kalman_filter.predict(|state, _control| state.clone());

        // println!(
        //     "Predicted position: x = {}, y = {}, theta = {}",
        //     kalman_state.x[0], kalman_state.x[1], kalman_state.x[2]
        // );
    }

    fn correct(&mut self) {
        // let measured_theta = imu_data.orientation.z; // Assuming the IMU provides orientation in radians
        // let measured_x = ...; // Obtain x from other sensor or method
        // let measured_y = ...; // Obtain y from other sensor or method

        // let z = Vector3::new(measured_x, measured_y, measured_theta);
        // self.kalman_filter.update(&z);

        // let state = self.kalman_filter.state();
        // println!("Corrected position: x = {}, y = {}, theta = {}", state[0], state[1], state[2]);
    }

}

impl PID {
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            previous_error: 0.0,
        }
    }

    pub fn update(&mut self, error: f64, delta_time: Seconds) -> f64 {
        self.integral += error * delta_time;
        let derivative = (error - self.previous_error) / delta_time;
        self.previous_error = error;
        self.kp * error + self.ki * self.integral + self.kd * derivative
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.previous_error = 0.0;
    }
}
#[derive(Debug)]
pub struct RobotController {
    odometry: Odometry,
    linear_pid: PID,
    angular_pid: PID,
    target_position: Position,
    max_velocity: f64,
}

impl Display for RobotController {
    fn fmt(&self, f: &mut Formatter) -> Result {
        write!(f, "{}", self.odometry)
    }
}

impl RobotController {
    pub fn new(
        wheel_radius: Meters,
        wheel_base: Meters,
        kp: f64,
        ki: f64,
        kd: f64,
        max_vel: f64,
    ) -> Self {
        Self {
            odometry: Odometry::new(wheel_radius, wheel_base),
            linear_pid: PID::new(kp, ki, kd),
            angular_pid: PID::new(kp, ki, kd),
            target_position: Position::new(),
            max_velocity: max_vel,
            // kalman_filter: kalman::KalmanFilter::new(),
        }
    }

    pub fn set_line(&mut self, kp: f64, ki: f64, kd: f64) {
        self.linear_pid = PID::new(kp, ki, kd);
    }

    pub fn set_angle(&mut self, kp: f64, ki: f64, kd: f64) {
        self.angular_pid = PID::new(kp, ki, kd);
    }

    pub fn update(
        &mut self,
        new_left_angle: Radians,
        new_right_angle: Radians,
        delta_time: Seconds,
    ) -> (f64, f64) {
        self.odometry.update(new_left_angle, new_right_angle);
        self.control(delta_time)
    }

    fn fix_angle(&self, angle: Radians) -> Radians {
        angle.sin().atan2(angle.cos())
    }

    pub fn set_target(&mut self, target_position: Position) {
        self.target_position = target_position;
        // self.target_orientation = target_orientation;
    }

    fn get_error(&self) -> (Meters, Radians) {
        let x_err = self.target_position.x - self.odometry.position.x;
        let y_err = self.target_position.y - self.odometry.position.y;
        let position_error = y_err.hypot(x_err); // (x_err.powi(2) + y_err.powi(2)).sqrt();
        let theta = y_err.atan2(x_err);
        let orientation_error: f64 = theta - self.odometry.orientation.theta;
        (position_error, orientation_error)
    }

    fn control(&mut self, delta_time: Seconds) -> (f64, f64) {
        let wheels = &self.odometry.wheel_state;

        let (position_error, orientation_error) = self.get_error();
        let mut angular_velocity = self.angular_pid.update(orientation_error, delta_time);
        angular_velocity = self.fix_angle(angular_velocity);

        let linear_velocity = self.linear_pid.update(position_error, delta_time);

        println!("d_err {position_error}, o_err {orientation_error}, l_v {linear_velocity}, a_v {angular_velocity}");

        let left_wheel_velocity: f64 =
            linear_velocity - (angular_velocity * wheels.wheel_base / 2.0);
        let right_wheel_velocity = linear_velocity + (angular_velocity * wheels.wheel_base / 2.0);

        let speed_limit = self.max_velocity * 0.6;
        let max_velocity = right_wheel_velocity.abs().max(left_wheel_velocity.abs());

        self.predict(linear_velocity, angular_velocity, delta_time);

        if max_velocity > 0.0 {
            let scaling_factor = 1.0 / max_velocity;
            (
                left_wheel_velocity * scaling_factor * speed_limit,
                right_wheel_velocity * scaling_factor * speed_limit,
            )
        } else {
            (0.0, 0.0)
        }
    }

    pub fn reset(&mut self) {
        self.odometry.reset();
        self.linear_pid.reset();
        self.angular_pid.reset();
        println!("=================== reset ===================");
    }

    pub fn check(&self, x: Meters, y: Meters) -> bool {
        let (pos, orient) = self.get_error();
        pos.abs() < 0.002
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_odometry_update() {
        let mut rc = RobotController::new(0.0205, 0.0565, 0.5, 0.01, 0.1, 6.28); // Assuming wheel radius is 0.032 meters
        rc.set_target(Position { x: 1.0, y: 1.0 });
        let (l, r) = rc.update(0.0, 0.0, 0.032);
        //assert!(l == 0.0);
    }
}
