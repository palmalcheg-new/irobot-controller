use forester_rs::runtime::builder::ForesterBuilder;
use std::{
    path::PathBuf,
    sync::{Arc, Mutex},
};

use crate::actions::{RobotRef, Robotics};

pub fn test_folder(path_in: &str) -> PathBuf {
    let mut root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    root.push("tree");
    root.push(path_in);
    root
}

pub fn fb(folder: &str, treeFile: &str) -> ForesterBuilder {
    let root = test_folder(folder);

    let mut fb = ForesterBuilder::from_fs();
    fb.main_file(treeFile.to_string());
    fb.root(root);

    fb
}

#[derive(Default)]
pub struct TestRobot;

pub fn new_test_ref() -> RobotRef {
    let instance = TestRobot::default();
    Arc::new(Mutex::new(Box::new(instance)))
}

impl Robotics for TestRobot {
    fn led_on(&mut self) {
        todo!()
    }

    fn get_time_step(&mut self) -> i32 {
        todo!()
    }

    fn step(&mut self) {
        todo!()
    }

    fn init_devices(&mut self) -> forester_rs::runtime::RtResult<()> {
        todo!()
    }

    fn is_there_a_collision_at_left(&self) -> bool {
        todo!()
    }

    fn is_there_a_collision_at_right(&self) -> bool {
        todo!()
    }

    fn flush_ir_receiver(&self) {
        todo!()
    }

    fn is_there_a_virtual_wall(&self) -> bool {
        todo!()
    }

    fn is_there_a_cliff_at_left(&self) -> bool {
        todo!()
    }

    fn is_there_a_cliff_at_right(&self) -> bool {
        todo!()
    }

    fn is_there_a_cliff_at_front(&self) -> bool {
        todo!()
    }

    fn go_forward(&self) {
        todo!()
    }

    fn go_backward(&self) {
        todo!()
    }

    fn stop(&self) {
        todo!()
    }

    fn control_drive(&mut self) {
        todo!()
    }

    fn wait(&mut self, sec: f64) {
        todo!()
    }

    fn turn(&mut self, angle: f64) {
        todo!()
    }

    fn dt(&mut self) -> f64 {
        todo!()
    }

    fn controller(&mut self) -> Option<&mut crate::control::RobotController> {
        None
    }

    fn reset_simulation(&self) {
        todo!()
    }
}
