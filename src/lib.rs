#![feature(btree_cursors)]

use std::{collections::BTreeMap, process::Output, ops::{Mul, Add, Sub}};

use ordered_float::NotNan;
use serde::{Serialize, Deserialize};
use uom::si::{f64::{AngularVelocity, Length, Angle, Velocity, Time}, length::meter, angle::radian, velocity::meter_per_second, angular_velocity::radian_per_second, time::second};

#[derive(Serialize, Deserialize)]
pub struct Trajectory {
    samples: Vec<Sample>,
}


#[derive(Serialize, Deserialize, Debug)]
pub struct Sample {
    x: f64,
    y: f64,
    heading: f64,
    #[serde(rename = "angularVelocity")]
    angular_velocity: f64,
    #[serde(rename = "velocityX")]
    velocity_x: f64,
    #[serde(rename = "velocityY")]
    velocity_y: f64,
    timestamp: f64,
}

pub struct Path {
    samples: BTreeMap<NotNan<f64>, Pose>,
}

impl Path {
    fn get(&self, elapsed: Time) -> Pose {
        let elapsed = elapsed.get::<second>();
        let below = self.samples.upper_bound(std::ops::Bound::Included(&NotNan::new(elapsed).unwrap()));

        if let Some(above) = below.peek_next() {
            let progress = elapsed / *(above.0 - below.key().unwrap());

            below.value().unwrap().lerp(above.1, progress)

        } else {
            below.value().unwrap().to_owned()
        }
    }
}

impl From<Trajectory> for Path {
    fn from(value: Trajectory) -> Self {
        let mut map : BTreeMap<NotNan<f64>, Pose>= BTreeMap::new();
        for sample in value.samples {
            map.insert(NotNan::new(sample.timestamp).unwrap(),sample.into());
        }

        Self{ samples: map}
    }
}

#[derive(Clone, Debug)]
pub struct Pose {
    x: Length,
    y: Length,
    heading: Angle,
    angular_velocity: AngularVelocity,
    velocity_x: Velocity,
    velocity_y: Velocity,
}


fn lerp<A>(a: A, b: A, l: f64) -> A
where A: Sub<A, Output=A> +Add<A, Output=A> + Mul<f64, Output = A> + Clone {
    a.clone() + (b.clone()-a.clone())*l
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
        let deser: Trajectory = serde_json::from_str(include_str!("../../deploy/choreo/center-long.traj")).unwrap();
        println!("samp 1 {:?}", deser.samples[0]);
        println!("sec 1.5 {:?}", Path::from(deser).get(Time::new::<second>(1.5)));
    }
}
