use std::env;

use anyhow::{Error, Result};

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;

    let node = rclrs::create_node(&context, "minimal_publisher")?;

    let publisher =
        node.create_publisher::<nav_msgs::msg::Odometry>("topic", rclrs::QOS_PROFILE_DEFAULT)?;

    let message = nav_msgs::msg::Odometry::default();

    while context.ok() {
        println!("Publishing: [{:?}]", message);
        publisher.publish(&message)?;
        std::thread::sleep(std::time::Duration::from_millis(1000));
    }
    Ok(())
}
