use opencv::{
    core::{self, Mat, MatTraitConst, Point, Scalar},
    imgproc::{self, rectangle_points},
};

pub struct Map {
    pub data: Mat,
}

impl Map {
    pub fn new(data: Mat) -> Self {
        Self { data }
    }
    pub fn new_obstacle_map() -> Self {
        let mut img = Mat::new_rows_cols_with_default(480, 640, core::CV_8UC3, Scalar::all(255.0))
            .expect("failed to create image");
        // Make a rectangle
        rectangle_points(
            &mut img,
            Point::new(200, 100),
            Point::new(550, 200),
            Scalar::all(0.0),
            imgproc::FILLED,
            imgproc::LINE_8,
            0,
        )
        .expect("failed to draw rectangle");

        rectangle_points(
            &mut img,
            Point::new(300, 50),
            Point::new(450, 150),
            Scalar::all(0.0),
            imgproc::FILLED,
            imgproc::LINE_8,
            0,
        )
        .expect("failed to draw rectangle");

        rectangle_points(
            &mut img,
            Point::new(300, 280),
            Point::new(500, 450),
            Scalar::all(0.0),
            imgproc::FILLED,
            imgproc::LINE_8,
            0,
        )
        .expect("failed to draw rectangle");

        rectangle_points(
            &mut img,
            Point::new(100, 300),
            Point::new(200, 400),
            Scalar::all(0.0),
            imgproc::FILLED,
            imgproc::LINE_8,
            0,
        )
        .expect("failed to draw rectangle");

        Self { data: img }
    }

    pub fn get_data(&self) -> &Mat {
        &self.data
    }

    pub fn get_data_mut(&mut self) -> &mut Mat {
        &mut self.data
    }

    pub fn is_valid(&self, x: i32, y: i32) -> bool {
        x >= 0 && x < self.data.cols() && y >= 0 && y < self.data.rows()
    }

    pub fn is_occupied(&self, x: i32, y: i32) -> bool {
        self.is_valid(x, y) && {
            let pixel = self
                .data
                .at_2d::<core::Vec3b>(y, x)
                .expect("failed to get pixel");
            pixel[0] == 0 && pixel[1] == 0 && pixel[2] == 0
        }
    }
}
