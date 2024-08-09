use forester_rs::runtime::action::Impl;
use forester_rs::runtime::builder::ForesterBuilder;
use forester_rs::runtime::RtResult;
use forester_rs::tracer::{Tracer, TracerConfig};
use irobot_controller::robot::new_robot_ref;
use std::f64::consts::PI;
use std::path::PathBuf;

use irobot_controller::actions::RobotRef;
use irobot_controller::actions::{
    CollisionChecker, Init, Moving, Simulation, Step, Turning, Waiting,
};
use irobot_controller::epuck::new_epuck_ref;

fn main() -> RtResult<()> {
    let mut forester = init_fb().build()?;
    println!("The Forester is ready");
    let res = forester.run()?;
    println!("{:?}", res);
    Ok(())
}

fn init_fb() -> ForesterBuilder {
    let mut root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .to_path_buf();
    root.push("irobot-controller");
    root.push("tree");

    let robot = new_epuck_ref();
    let mut fb = go_to_goal(root, robot);

    fb.tracer(tracer());
    fb
}

fn line_follow(root: PathBuf, ro: RobotRef) -> ForesterBuilder {
    let mut fb = ForesterBuilder::from_fs();
    fb.root(root.clone());
    fb.main_file("epuck_line_follow.tree".to_string());
    fb.register_sync_action("init_robot", Init(ro.clone()));
    fb.register_sync_action("wait", Waiting(ro.clone()));
    fb.register_sync_action("collision", CollisionChecker(ro.clone()));
    fb.register_sync_action("turn", Turning(ro.clone()));
    fb.register_sync_action("move", Moving(ro.clone()));
    fb.register_sync_action("step", Step(ro.clone()));
    fb
}

fn go_to_goal(root: PathBuf, ro: RobotRef) -> ForesterBuilder {
    let mut fb = ForesterBuilder::from_fs();
    fb.root(root.clone());
    fb.main_file("epuck_go_to_goal.tree".to_string());
    fb.register_sync_action("init_robot", Init(ro.clone()));
    fb.register_sync_action("wait", Waiting(ro.clone()));
    fb.register_sync_action("collision", CollisionChecker(ro.clone()));
    fb.register_sync_action("turn", Turning(ro.clone()));
    fb.register_sync_action("step", Step(ro.clone()));
    fb.register_sync_action("move", Moving(ro.clone()));
    fb.register_sync_action("simulation", Simulation(ro.clone()));
    irobot_controller::routing::apply(&mut fb, ro);
    fb
}

fn tracer() -> Tracer {
    let mut root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    root.push("");
    root.push("tracer.log");
    Tracer::create(TracerConfig::in_file(root, None)).unwrap()
}
