use opencv::{
    core::{self, Mat, MatTraitConst, Point, Point2i, Scalar, Vec3b},
    imgproc::{rectangle_points, FILLED, LINE_8},
};

pub fn is_valid_position(img: &Mat, pos: Point2i) -> bool {
    pos.x >= 0 && pos.x < img.cols() && pos.y >= 0 && pos.y < img.rows()
}

pub fn is_obstacle(img: &Mat, pos: Point2i) -> bool {
    let pixel = img.at_2d::<Vec3b>(pos.y, pos.x).expect("Not valid pos");
    *pixel != Vec3b::all(255)
}

pub fn generate_random_pos(row_max: i32, col_max: i32) -> Point2i {
    let col = rand::random::<i32>() % col_max;
    let row = rand::random::<i32>() % row_max;
    Point2i::new(col.abs(), row.abs())
}

pub fn generate_random_color(n: usize) -> Vec<Vec3b> {
    let mut colors = Vec::<Vec3b>::with_capacity(n);
    for _ in 0..n {
        let color = Vec3b::from_array([
            rand::random::<u8>(),
            rand::random::<u8>(),
            rand::random::<u8>(),
        ]);
        colors.push(color);
    }
    colors
}

pub fn make_map() -> Mat {
    let mut img = Mat::new_rows_cols_with_default(480, 640, core::CV_8UC3, Scalar::all(255.0))
        .expect("failed to create image");
    // Make a rectangle
    rectangle_points(
        &mut img,
        Point::new(200, 100),
        Point::new(550, 200),
        Scalar::all(0.0),
        FILLED,
        LINE_8,
        0,
    )
    .expect("failed to draw rectangle");

    rectangle_points(
        &mut img,
        Point::new(300, 50),
        Point::new(450, 150),
        Scalar::all(0.0),
        FILLED,
        LINE_8,
        0,
    )
    .expect("failed to draw rectangle");

    rectangle_points(
        &mut img,
        Point::new(300, 280),
        Point::new(500, 450),
        Scalar::all(0.0),
        FILLED,
        LINE_8,
        0,
    )
    .expect("failed to draw rectangle");

    rectangle_points(
        &mut img,
        Point::new(100, 300),
        Point::new(200, 400),
        Scalar::all(0.0),
        FILLED,
        LINE_8,
        0,
    )
    .expect("failed to draw rectangle");

    img
}
