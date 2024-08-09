use forester_rs::runtime::{
    action::{Impl, Tick},
    args::{RtArgs, RtArgument, RtValue},
    builder::ForesterBuilder,
    context::TreeContextRef,
    rtree::rnode::DecoratorType,
    rtree::RuntimeTree,
    RuntimeError, TickResult,
};
use serde::Deserialize;
use std::error::Error;
use std::fs;

use crate::{
    actions::RobotRef,
    metrics::{Position, Radians},
};

pub fn apply(fb: &mut ForesterBuilder, ro: RobotRef) {
    let loader = Config::default();
    if let Ok(cfg) = loader.load_route_from_file("tree/route.yaml") {
        let cc = cfg.route.clone();
        fb.register_sync_action("is_arrived", ArrivedCheck(ro.clone(), cc.clone()));
        fb.register_sync_action("define_direction", DefineDir(ro.clone(), cc.clone()));
        fb.register_sync_action("init_pid", InitPID(ro.clone(), cfg));
    };
}

#[derive(Debug, Deserialize, Clone, Copy)]
struct RoutePosition {
    position: Position,
    // orientation: Radians
}
#[derive(Debug, Deserialize, Clone, Copy, Default)]
struct PIDConfig {
    line: PID,
    angle: PID,
}

#[derive(Debug, Deserialize, Clone, Copy, Default)]
struct PID {
    p: f64,
    i: f64,
    d: f64,
}

#[derive(Debug, Deserialize, Clone, Default)]
struct Config {
    pid: PIDConfig,
    route: Vec<RoutePosition>,
}

impl Config {
    fn load_route_from_file(&self, filename: &str) -> Result<Config, Box<dyn Error>> {
        let file_content = fs::read_to_string(filename)?;
        let config = serde_yaml::from_str(&file_content)?;
        Ok(config)
    }
}

struct ArrivedCheck(pub RobotRef, pub Vec<RoutePosition>);

impl Impl for ArrivedCheck {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        let robot = &mut self.0.lock()?;
        let target = target(args, &ctx)?;
        let rr = &self.1[target as usize];
        let b = ctx.bb();
        let mut bb = b.lock()?;
        if let Some(rbc) = robot.controller() {
            let tick_result = if rbc.check(rr.position.x, rr.position.y) {
                bb.put("target".to_owned(), RtValue::int(target + 1))?;
                ctx.trace(format!("robot arrived -> {target}"))?;
                Ok(TickResult::success())
            } else {
                ctx.trace(format!("not arrived yet -> {target}"))?;
                Ok(TickResult::failure(
                    "target and current are not eq".to_owned(),
                ))
            };
            tick_result
        } else {
            Ok(TickResult::success())
        }
    }
}

struct DefineDir(pub RobotRef, pub Vec<RoutePosition>);

impl Impl for DefineDir {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        let target = target(args, &ctx)?;
        let robot = &mut self.0.lock()?;
        match (self.1.get(target as usize), robot.controller()) {
            (Some(rr), Some(rbc)) => {
                rbc.reset();
                rbc.set_target(rr.position);
                ctx.trace(format!("move on -> {target}"))?;
                Ok(TickResult::success())
            }
            (_, _) => {
                robot.stop();
                ctx.trace("Unable to define direction!".to_string())?;
                Ok(TickResult::failure(
                    "Unable to define direction".to_string(),
                ))
            }
        }
    }
}

struct InitPID(pub RobotRef, pub Config);

impl Impl for InitPID {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        let robot = &mut self.0.lock()?;
        let cfg = &self.1;
        if let Some(rbc) = robot.controller() {
            let a_pid = cfg.pid.angle;
            let l_pid = cfg.pid.line;
            rbc.set_angle(a_pid.p, a_pid.i, a_pid.d);
            rbc.set_line(l_pid.p, l_pid.i, l_pid.d);
        }
        Ok(TickResult::success())
    }
}

fn target(args: RtArgs, ctx: &TreeContextRef) -> Result<i64, RuntimeError> {
    args.first()
        .and_then(|v| v.cast(ctx.clone()).int().ok())
        .flatten()
        .ok_or(RuntimeError::fail("target is absent".to_owned()))
}

fn current(ctx: &TreeContextRef) -> Result<i64, RuntimeError> {
    ctx.bb()
        .lock()?
        .get("curr_coord".to_owned())?
        .and_then(|v| v.clone().as_int())
        .ok_or(RuntimeError::fail("current is absent".to_owned()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tests::{fb, new_test_ref};

    #[test]
    fn load_config() {
        let loader = Config::default();
        match loader.load_route_from_file("tree/route.yaml") {
            Err(error) => {
                panic!("{}", error)
            }
            Ok(_) => {}
        };
    }

    #[test]
    fn inter_args_func() {
        struct T;
        impl Impl for T {
            fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
                let v = args
                    .first()
                    .and_then(|v| v.as_string())
                    .ok_or(RuntimeError::fail("expected a string".to_string()))?;

                let b = ctx.bb();
                let mut bb = b.lock()?;

                bb.put("t".to_string(), RtValue::str(v))?;

                let route = bb
                    .get("route".to_owned())?
                    .and_then(|v| v.clone().as_vec(|v| v.as_pointer()))
                    .ok_or(RuntimeError::fail("route is absent".to_owned()));

                Ok(TickResult::success())
            }
        }
        let mock = tests::new_test_ref();
        let mut fb = fb("", "tests/test_route.tree");
        fb.register_sync_action("t", T);
        fb.bb_load("bb_load.json".to_string());

        let mut forester = fb.build().unwrap();
        let result = forester.run().unwrap();

        assert_eq!(result, TickResult::success());

        let bb = forester.bb.lock().unwrap();
        let r = bb.get("t".to_string());
        assert_eq!(r, Ok(Some(&RtValue::str("test".to_string()))));
    }
}
