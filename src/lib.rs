#![feature(btree_cursors)]

use std::{collections::BTreeMap, process::Output, ops::{Mul, Add, Sub, Deref}};
use ordered_float::NotNan;
use serde::{Serialize, Deserialize};
use uom::si::{f64::{AngularVelocity, Length, Angle, Velocity, Time},
              length::meter, angle::{radian, degree},
              velocity::meter_per_second, angular_velocity::radian_per_second, time::second};

// Root structure to match Choreo JSON
#[derive(Serialize, Deserialize)]
pub struct ChoreoTrajectory {
    pub name: String,
    pub trajectory: TrajectoryData,
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
        Ok(Self::from_trajectory_data(choreo.trajectory))
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
