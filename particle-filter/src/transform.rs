use std::ops::{AddAssign, Div};

#[derive(Debug, Clone, Copy)]
pub struct Transform {
    /// X-axis
    pub x: f64,
    /// Y-axis is inverted
    pub y: f64,
    /// Rotation in radians
    pub theta: f64,
}

impl Default for Transform {
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        }
    }
}

impl AddAssign<&Transform> for Transform {
    fn add_assign(&mut self, rhs: &Self) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.theta += rhs.theta;
    }
}

impl Div<f64> for Transform {
    type Output = Self;

    fn div(self, rhs: f64) -> Self::Output {
        Self {
            x: self.x / rhs,
            y: self.y / rhs,
            theta: self.theta / rhs,
        }
    }
}
