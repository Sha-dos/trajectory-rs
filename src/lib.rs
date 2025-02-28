#![feature(btree_cursors)]

use std::{collections::BTreeMap, process::Output, ops::{Mul, Add, Sub, Deref}};
use ordered_float::NotNan;
use serde::{Serialize, Deserialize};
use uom::si::{f64::{AngularVelocity, Length, Angle, Velocity, Time},
              length::meter, angle::{radian, degree},
              velocity::meter_per_second, angular_velocity::radian_per_second, time::second};
use uom::si::length::millimeter;

#[derive(Serialize, Deserialize)]
pub struct ChoreoTrajectory {
    pub name: String,
    pub version: u32,
    pub snapshot: Snapshot,
    pub params: Params,
    pub trajectory: TrajectoryData,
    pub events: Vec<Event>,
}

#[derive(Serialize, Deserialize)]
pub struct Snapshot {
    pub waypoints: Vec<SnapshotWaypoint>,
    pub constraints: Vec<Constraint>,
    #[serde(rename = "targetDt")]
    pub target_dt: f64,
}

#[derive(Serialize, Deserialize)]
pub struct SnapshotWaypoint {
    pub x: f64,
    pub y: f64,
    pub heading: f64,
    pub intervals: u32,
    pub split: bool,
    #[serde(rename = "fixTranslation")]
    pub fix_translation: bool,
    #[serde(rename = "fixHeading")]
    pub fix_heading: bool,
    #[serde(rename = "overrideIntervals")]
    pub override_intervals: bool,
}

#[derive(Serialize, Deserialize)]
pub struct Constraint {
    // TODO: implement
}

#[derive(Serialize, Deserialize)]
pub struct Params {
    // TODO: implement
}

#[derive(Serialize, Deserialize)]
pub struct Event {
    // TODO: implement
}

#[derive(Serialize, Deserialize)]
pub struct TrajectoryData {
    pub samples: Vec<Sample>,
    pub waypoints: Vec<f64>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Sample {
    pub t: f64,
    pub x: f64,
    pub y: f64,
    pub heading: f64,
    #[serde(rename = "vx")]
    pub velocity_x: f64,
    #[serde(rename = "vy")]
    pub velocity_y: f64,
    #[serde(rename = "omega")]
    pub angular_velocity: f64,
}

pub struct Path {
    samples: BTreeMap<NotNan<f64>, Pose>,
    waypoints: Vec<f64>,
}

impl Path {
    pub fn from_trajectory(trajectory: &str) -> Result<Self, serde_json::Error> {
        let choreo = serde_json::from_str::<ChoreoTrajectory>(trajectory)?;

        let valid_waypoints = choreo.snapshot.waypoints
            .iter()
            .enumerate()
            .filter(|(_, wp)| wp.split)
            .map(|(i, _)| choreo.trajectory.waypoints[i])
            .collect();

        let trajectory_data = TrajectoryData {
            samples: choreo.trajectory.samples,
            waypoints: valid_waypoints,
        };

        Ok(Self::from_trajectory_data(trajectory_data))
    }

    fn from_trajectory_data(data: TrajectoryData) -> Self {
        let mut samples = BTreeMap::new();
        for sample in data.samples {
            samples.insert(NotNan::new(sample.t).unwrap(), sample.into());
        }
        Self {
            samples,
            waypoints: data.waypoints
        }
    }

    pub fn get(&self, elapsed: Time) -> Pose {
        let elapsed = elapsed.get::<second>();
        let below = self.samples.upper_bound(std::ops::Bound::Included(&NotNan::new(elapsed).unwrap()));

        if let Some(above) = below.peek_next() {
            let below_time = **below.key().unwrap();
            let above_time = **above.0;
            let progress = (elapsed - below_time) / (above_time - below_time);

            below.value().unwrap().lerp(above.1, progress)
        } else {
            below.value().unwrap().to_owned()
        }
    }

    pub fn length(&self) -> Time {
        Time::new::<second>(**self.samples.last_key_value().unwrap().0)
    }

    // Get waypoint timestamps
    pub fn waypoints(&self) -> &[f64] {
        &self.waypoints
    }
}

#[derive(Clone, Debug)]
pub struct Pose {
    pub x: Length,
    pub y: Length,
    pub heading: Angle,
    pub angular_velocity: AngularVelocity,
    pub velocity_x: Velocity,
    pub velocity_y: Velocity,
}

impl Pose {
    fn lerp(&self, other: &Pose, l: f64) -> Pose {
        Pose {
            x: lerp(self.x, other.x, l),
            y: lerp(self.y, other.y, l),
            heading: lerp(self.heading, other.heading, l),
            angular_velocity: lerp(self.angular_velocity, other.angular_velocity, l),
            velocity_x: lerp(self.velocity_x, other.velocity_x, l),
            velocity_y: lerp(self.velocity_y, other.velocity_y, l),
        }
    }

    /// X and Y are half of the field length and width
    /// velocity might be wrong
    fn mirror(&self, x: Length, y: Length) -> Pose {
        Pose {
            x: x - self.x + x,
            y: y - self.y + y,
            heading: self.heading + Angle::new::<radian>(std::f64::consts::PI),
            angular_velocity: -self.angular_velocity,
            velocity_x: self.velocity_x,
            velocity_y: self.velocity_y,
        }
    }
}

fn lerp<A>(a: A, b: A, l: f64) -> A
where A: Sub<A, Output=A> + Add<A, Output=A> + Mul<f64, Output = A> + Clone {
    a.clone() + (b - a) * l
}

impl From<Sample> for Pose {
    fn from(value: Sample) -> Self {
        Self {
            x: Length::new::<meter>(value.x),
            y: Length::new::<meter>(value.y),
            heading: Angle::new::<radian>(value.heading),
            angular_velocity: AngularVelocity::new::<radian_per_second>(value.angular_velocity),
            velocity_x: Velocity::new::<meter_per_second>(value.velocity_x),
            velocity_y: Velocity::new::<meter_per_second>(value.velocity_y),
        }
    }
}

#[test]
fn parse() {
    let data = include_str!("../../RobotCode2025/auto/Blue2.traj");
    let path = Path::from_trajectory(data).unwrap();
    for i in path.waypoints() {
        println!("{}", i);
    }
}

#[test]
fn mirror_test() {
    let path = Path::from_trajectory(include_str!("../../RobotCode2025/auto/Blue2.traj")).unwrap();
    let setpoint = path.get(Time::new::<second>(0.0));

    println!("{:?}", setpoint);

    let setpoint = setpoint.mirror(Length::new::<meter>(17.55 / 2.), Length::new::<meter>(8.05 / 2.));

    assert_eq!(setpoint.x.get::<meter>(), 9.53808);
    assert_eq!(setpoint.y.get::<meter>(), 0.4438400000000007);
    assert_eq!(setpoint.heading.get::<degree>(), 180.);
}