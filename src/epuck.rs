use crate::actions::{RobotRef, Robotics};
use crate::control::*;
use crate::metrics::{Odometry, Radians, Seconds};
use bindings::wb_inertial_unit_enable;
use forester_rs::runtime::{RtResult, RuntimeError};
use std::process::exit;
use std::sync::{Arc, Mutex};

use forester_webots::bindings::WbDeviceTag;
use forester_webots::bindings::{
    wb_gps_enable, wb_inertial_unit_get_roll_pitch_yaw, wb_supervisor_simulation_reset,
};
use forester_webots::*;
use rand::random;

pub fn new_epuck_ref() -> RobotRef {
    let instance = EPuck::default();
    Arc::new(Mutex::new(Box::new(instance)))
}

#[derive(Default, Debug)]
pub struct EPuck {
    max_speed: f64,
    basic_step: Option<f64>,
    left_sensors: Vec<WbDeviceTag>,
    right_sensors: Vec<WbDeviceTag>,
    line_sensors: Vec<WbDeviceTag>,
    left_motor: WbDeviceTag,
    right_motor: WbDeviceTag,
    left_position_sensor: WbDeviceTag,
    right_position_sensor: WbDeviceTag,
    controller: Option<RobotController>,
    previous_time: Seconds,
    drive_left_velocity: f64,
    drive_right_velocity: f64,

    imu: WbDeviceTag,
    gps: WbDeviceTag,
}

impl EPuck {
    fn gps_enable(&self, tag: WbDeviceTag, sampling_period: i32) {
        unsafe { wb_gps_enable(tag, sampling_period) }
    }

    fn imu_enable(&self, tag: WbDeviceTag, sampling_period: i32) {
        unsafe { wb_inertial_unit_enable(tag, sampling_period) }
    }

    fn imu_data(&self) -> Vec<f64> {
        unsafe {
            std::slice::from_raw_parts(wb_inertial_unit_get_roll_pitch_yaw(self.imu), 3).to_vec()
        }
    }
}

impl Robotics for EPuck {
    fn dt(&mut self) -> f64 {
        let delta = wb_robot_get_time() - self.previous_time;
        self.previous_time = wb_robot_get_time();
        return delta;
    }

    fn get_time_step(&mut self) -> i32 {
        if let Some(b_step) = self.basic_step {
            b_step as i32
        } else {
            let step = wb_robot_get_basic_time_step();
            self.basic_step = Some(step);
            step as i32
        }
    }

    fn step(&mut self) {
        let step = self.get_time_step();
        if wb_robot_step(step) == -1 {
            wb_robot_cleanup();
            exit(0);
        }
        let delta = self.dt();
        if let Some(ctrl) = &mut self.controller {
            let new_left_angle = wb_position_sensor_get_value(self.left_position_sensor) as Radians;
            let new_right_angle =
                wb_position_sensor_get_value(self.right_position_sensor) as Radians;
            (self.drive_left_velocity, self.drive_right_velocity) =
                ctrl.update(new_left_angle, new_right_angle, delta);
            println!("Controller : {}", ctrl)
        }
    }

    fn init_devices(&mut self) -> RtResult<()> {
        self.max_speed = 6.28;
        self.previous_time = wb_robot_get_time();
        self.controller = Some(RobotController::new(
            0.0205,
            0.0565,
            0.5,
            0.1,
            0.01,
            self.max_speed,
        ));

        self.left_sensors = [5, 6, 7]
            .iter()
            .map(|i| wb_robot_get_device(&format!("ps{}", i)))
            .collect();

        self.right_sensors = [0, 1, 2]
            .iter()
            .map(|i| wb_robot_get_device(&format!("ps{}", i)))
            .collect();

        self.line_sensors = [0, 1, 2]
            .iter()
            .map(|i| wb_robot_get_device(&format!("gs{}", i)))
            .collect();

        let step = self.get_time_step();
        for sensor in [
            self.left_sensors.as_slice(),
            self.right_sensors.as_slice(),
            self.line_sensors.as_slice(),
        ]
        .concat()
        {
            wb_distance_sensor_enable(sensor, step);
        }
        self.left_motor = wb_robot_get_device("left wheel motor");
        self.right_motor = wb_robot_get_device("right wheel motor");

        self.imu = wb_robot_get_device("inertial unit");
        self.gps = wb_robot_get_device("gps");

        self.gps_enable(self.gps, step);
        self.imu_enable(self.imu, step);

        wb_motor_set_position(self.left_motor, f64::INFINITY);
        wb_motor_set_position(self.right_motor, f64::INFINITY);

        wb_motor_set_velocity(self.left_motor, 0.0);
        wb_motor_set_velocity(self.right_motor, 0.0);

        self.left_position_sensor = wb_robot_get_device("left wheel sensor");
        self.right_position_sensor = wb_robot_get_device("right wheel sensor");

        wb_position_sensor_enable(self.left_position_sensor, step);
        wb_position_sensor_enable(self.right_position_sensor, step);

        Ok(())
    }

    fn stop(&self) {
        wb_motor_set_velocity(self.left_motor, -0f64);
        wb_motor_set_velocity(self.right_motor, -0f64);
    }

    fn wait(&mut self, sec: f64) {
        let start_time = wb_robot_get_time();
        while start_time + sec > wb_robot_get_time() {
            self.step()
        }
    }

    fn control_drive(&mut self) {
        let imu_data = self.imu_data();

        if let Some(ctrl) = &mut self.controller {
            wb_motor_set_velocity(self.left_motor, self.drive_left_velocity);
            wb_motor_set_velocity(self.right_motor, self.drive_right_velocity);
        }
    }

    fn turn(&mut self, angle: f64) {
        if angle < 0.0 {
            wb_motor_set_velocity(self.left_motor, self.max_speed * 0.8);
            wb_motor_set_velocity(self.right_motor, self.max_speed);
        } else {
            wb_motor_set_velocity(self.left_motor, self.max_speed);
            wb_motor_set_velocity(self.right_motor, self.max_speed * 0.8);
        }
    }

    fn is_there_a_collision_at_left(&self) -> bool {
        let r = wb_distance_sensor_get_value(self.line_sensors[2]);
        let l = wb_distance_sensor_get_value(self.line_sensors[0]);
        return l > 600.00 && r < 600.0;
    }

    fn is_there_a_collision_at_right(&self) -> bool {
        let r = wb_distance_sensor_get_value(self.line_sensors[2]);
        let l = wb_distance_sensor_get_value(self.line_sensors[0]);
        return r > 600.00 && l < 600.00;
    }
    /*
    fn flush_ir_receiver(&self) {
        while wb_receiver_get_queue_length(self.receiver) > 0 {
            wb_receiver_next_packet(self.receiver);
        }
    }
    fn is_there_a_virtual_wall(&self) -> bool {
        wb_receiver_get_queue_length(self.receiver) > 0
    }
    fn is_there_a_cliff_at_left(&self) -> bool {
        wb_distance_sensor_get_value(self.cliff_front_left) < 100.0
            || wb_distance_sensor_get_value(self.cliff_left) < 100.0
    }
    fn is_there_a_cliff_at_right(&self) -> bool {
        wb_distance_sensor_get_value(self.cliff_front_right) < 100.0
            || wb_distance_sensor_get_value(self.cliff_right) < 100.0
    }
    fn is_there_a_cliff_at_front(&self) -> bool {
        wb_distance_sensor_get_value(self.cliff_front_left) < 100.0
            || wb_distance_sensor_get_value(self.cliff_front_right) < 100.0
    }
    */
    fn go_forward(&self) {
        wb_motor_set_velocity(self.left_motor, self.max_speed);
        wb_motor_set_velocity(self.right_motor, self.max_speed);
    }
    fn go_backward(&self) {
        wb_motor_set_velocity(self.left_motor, -0.5 * self.max_speed);
        wb_motor_set_velocity(self.right_motor, -0.5 * self.max_speed);
    }

    fn led_on(&mut self) {}

    fn flush_ir_receiver(&self) {}

    fn is_there_a_virtual_wall(&self) -> bool {
        false
    }

    fn is_there_a_cliff_at_left(&self) -> bool {
        false
    }

    fn is_there_a_cliff_at_right(&self) -> bool {
        false
    }

    fn is_there_a_cliff_at_front(&self) -> bool {
        false
    }

    fn controller(&mut self) -> Option<&mut RobotController> {
        self.controller.as_mut()
    }

    fn reset_simulation(&self) {
        unsafe { wb_supervisor_simulation_reset() }
    }
}
