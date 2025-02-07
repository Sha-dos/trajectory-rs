#![feature(btree_cursors, diagnostic_namespace)]

use std::{collections::BTreeMap, process::Output, ops::{Mul, Add, Sub, Deref}};

use ordered_float::NotNan;
use serde::{Serialize, Deserialize};
use uom::si::{f64::{AngularVelocity, Length, Angle, Velocity, Time}, length::meter, angle::{radian, degree}, velocity::meter_per_second, angular_velocity::radian_per_second, time::second};

// New root structure to match JSON
#[derive(Serialize, Deserialize)]
pub struct TrajectoryFile {
    trajectory: Trajectory
}

#[derive(Serialize, Deserialize)]
pub struct Trajectory {
    samples: Vec<Sample>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Sample {
    t: f64,  // Changed from timestamp
    x: f64,
    y: f64,
    heading: f64,
    #[serde(rename = "omega")]  // Changed from angular_velocity
    angular_velocity: f64,
    #[serde(rename = "vx")]  // Changed from velocity_x
    velocity_x: f64,
    #[serde(rename = "vy")]  // Changed from velocity_y
    velocity_y: f64,
}

pub struct Path {
    samples: BTreeMap<NotNan<f64>, Pose>,
}

impl Path {
    pub fn from_trajectory(trajectory: &str) -> Result<Self, serde_json::Error> {
        // Updated to handle the new root structure
        Ok(serde_json::from_str::<TrajectoryFile>(trajectory)?.trajectory.into())
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
}

impl From<Trajectory> for Path {
    fn from(value: Trajectory) -> Self {
        let mut map: BTreeMap<NotNan<f64>, Pose> = BTreeMap::new();
        for sample in value.samples {
            map.insert(NotNan::new(sample.t).unwrap(), sample.into());  // Changed from timestamp to t
        }
        Self { samples: map }
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

fn lerp<A>(a: A, b: A, l: f64) -> A
where A: Sub<A, Output=A> + Add<A, Output=A> + Mul<f64, Output = A> + Clone {
    a.clone() + (b - a) * l
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

    pub fn mirror(&mut self) {
        self.x = Length::new::<meter>(8.) - self.x;
        self.heading = Angle::new::<degree>(90.) - self.heading;
    }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let trajectory_file = include_str!("../../RobotCode2025/auto/BlueTriangle.traj");
        let path = Path::from_trajectory(trajectory_file).unwrap();
        println!("sec 1.5 {:?}", path.get(Time::new::<second>(1.5)));
    }
}