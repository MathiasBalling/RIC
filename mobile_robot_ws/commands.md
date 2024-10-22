# Good commands for ROS2

## General

Make package under `ros2_ws/src`

```bash
ros2 pkg create --build-type ament_cmake --license Apache-2.0 --node-name my_node my_package
```

Build workspace

- `symlink-install` allows the installed files to be changed by changing
  the files in the source space (e.g. Python files or other non-compiled resources)
  for faster iteration.

```bash
colcon build --symlink-install
```

Install dependencies from `ros2_ws`

```bash
rosdep install --from-paths src -y --ignore-src
```

## TF2

PDF to see published tf2 frames

```bash
ros2 run tf2_tools view_frames
```

Convert xacro file to URDF:

```bash
ros2 run xacro xacro --inorder your_robot.xacro > your_robot.urdf
```
