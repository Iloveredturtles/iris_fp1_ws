# IRIS Final Project — Autonomous Car

ROS 2 Jazzy packages for the IRIS Programming Division final project:
autonomous driving in the Gazebo car simulator.

## Packages

| package | what it does | objectives |
|---|---|---|
| `communication` | UDP bridge to the simulator, plus WASD teleop | 1, 2, 3 |
| `vision` | HLS thresholding, bird's eye view, turn target | 5, 6 |
| `master` | Ackermann steering, lane following, drive-to-point | 4, 7 |

## Topics

```
/cmd_drive            geometry_msgs/Twist         linear.x = PWM, angular.z = steer deg
/car_feedback         std_msgs/Int32MultiArray    [L ticks, R ticks, vx cm/s, vy cm/s]
/vision/turn_target   geometry_msgs/PointStamped  x = lateral error -1..+1, z = valid
/vision/debug_image   sensor_msgs/Image           bird's eye view with the decision drawn
```

## Build

```bash
cd ~/iris_fp1_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

Drive it by hand:

```bash
ros2 run communication communication_node     # terminal 1
ros2 run communication wasd_teleop_node        # terminal 2
```

Autonomous:

```bash
ros2 launch master autonomous.launch.py
```

Drive to a point:

```bash
ros2 param set /master_node mode goto_point
ros2 param set /master_node target_x 15.0
ros2 param set /master_node target_y 0.0
```

## Tests

```bash
./run_tests.sh
```

Checks the UDP wire format, the Ackermann geometry and the lane-centre
decision with plain g++ — no ROS, no simulator, no camera.

## Tuning

Every threshold is a live ROS parameter, so nothing needs a rebuild:

```bash
ros2 run rqt_image_view rqt_image_view        # watch /vision/debug_image
ros2 param set /vision_node l_min 150
ros2 param set /vision_node horizon_y 0.58
ros2 param set /master_node steer_gain -18.0
```

See `NOTES.md` for why the lane finder works the way it does, and for the
steering-sign gotcha.
