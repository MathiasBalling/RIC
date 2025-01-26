use crate::{map::Map, transform::Transform};

const LIDAR_RANGE_MAX: i32 = 50;
const LIDAR_RANGE_MIN: i32 = 5;
const LIDAR_ANGLE_STEP: i32 = 10;

pub struct Robot {
    transform: Transform,
}

pub struct LidarReading {
    /// Angle of the lidar reading in radians
    pub angle: f64,
    /// Distance of the lidar reading
    pub distance: f64,
}

impl Robot {
    pub fn new(x: f64, y: f64, theta: f64) -> Self {
        Self {
            transform: Transform { x, y, theta },
        }
    }
    pub fn tranform(&mut self, delta_transform: &Transform) {
        self.transform += delta_transform;
    }
    pub fn get_transform(&self) -> &Transform {
        &self.transform
    }
    pub fn get_lidar_reading(map: &Map, tranform: &Transform) -> Vec<LidarReading> {
        let mut lidar_readings = Vec::new();

        let mut angle = 0;
        while angle < 360 {
            let lidar_angle = (angle as f64).to_radians() + tranform.theta;
            let mut lidar_reading = LidarReading {
                angle: lidar_angle,
                distance: LIDAR_RANGE_MAX as f64,
            };
            for rng in LIDAR_RANGE_MIN..LIDAR_RANGE_MAX {
                let lidar_x = tranform.x + rng as f64 * lidar_angle.cos();
                let lidar_y = tranform.y + rng as f64 * lidar_angle.sin();
                // Check for obstacles and map boundaries
                if map.is_occupied(lidar_x.round() as i32, lidar_y.round() as i32) {
                    // Overwrite the reading with last if there is an obstacle
                    let lidar_x = tranform.x + (rng - 1) as f64 * lidar_angle.cos();
                    let lidar_y = tranform.y + (rng - 1) as f64 * lidar_angle.sin();
                    let distance =
                        ((tranform.x - lidar_x).powi(2) + (tranform.y - lidar_y).powi(2)).sqrt();
                    lidar_reading = LidarReading {
                        angle: lidar_angle,
                        distance,
                    };
                    break;
                }
            }
            lidar_readings.push(lidar_reading);
            angle += LIDAR_ANGLE_STEP;
        }
        lidar_readings
    }
}
