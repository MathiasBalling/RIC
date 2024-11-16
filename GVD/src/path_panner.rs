use std::sync::{Arc, Mutex};

use opencv::{
    core::{Mat, MatTrait, Point2i, Point_, Scalar, Vec3b},
    highgui::{
        imshow, move_window, named_window, set_mouse_callback, wait_key, EVENT_LBUTTONDOWN,
        EVENT_LBUTTONUP, EVENT_MOUSEMOVE,
    },
    imgproc::{line, LINE_8},
};

const WINDOW_NAME: &str = "Path Panner";

pub fn path_panner(img: &Mat, edges: Vec<Point2i>) {
    println!("Performing path panner algorithm\n");
    let img: Arc<Mutex<Mat>> = Arc::new(Mutex::new(draw_edges(img, edges)));
    named_window(WINDOW_NAME, 0).unwrap();
    move_window(WINDOW_NAME, 650, 515).unwrap();

    let img_clone: Arc<Mutex<Mat>> = Arc::clone(&img);
    let pt_old = Arc::new(Mutex::new(Point_::default()));

    set_mouse_callback(
        WINDOW_NAME,
        Some(Box::new(move |event, x, y, flag| {
            let mut img_guard = img_clone.lock().unwrap();
            let mut pt_old_guard = pt_old.lock().unwrap();

            match event {
                EVENT_LBUTTONDOWN => {
                    *pt_old_guard = Point_::from((x, y));
                    println!("EVENT_LBUTTONDOWN: {},{}", x, y);
                }
                EVENT_LBUTTONUP => {
                    println!("EVENT_LBUTTONUP: {},{}", x, y);
                }
                EVENT_MOUSEMOVE => {
                    if flag == EVENT_LBUTTONDOWN {
                        let color = Scalar::from((0, 255, 0)); // BGR
                        let thickness = 2;
                        line(
                            &mut *img_guard,
                            *pt_old_guard,
                            Point_::from((x, y)),
                            color,
                            thickness,
                            LINE_8,
                            0,
                        )
                        .unwrap();
                        imshow(WINDOW_NAME, &*img_guard).unwrap();
                        *pt_old_guard = Point_::from((x, y));
                    }
                }
                _ => {}
            }
        })),
    )
    .unwrap();
    imshow(WINDOW_NAME, &*img.lock().unwrap()).unwrap();
    wait_key(1).unwrap();
}

fn draw_edges(img: &Mat, edges: Vec<Point2i>) -> Mat {
    let mut img = img.clone();
    for pos in edges {
        let pixel = img.at_2d_mut::<Vec3b>(pos.y, pos.x).unwrap();
        *pixel = Vec3b::from_array([0, 255, 0]);
    }
    img
}
