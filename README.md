# IRIS Final Project — Autonomous Car

ROS 2 Jazzy packages for the IRIS Programming Division final project:
autonomous driving in the FIRA car simulation.

## Packages

| package | what it does | objectives |
|---|---|---|
| `communication` | UDP bridge to the simulator, plus WASD teleop | 1, 2, 3 |
| `vision` | HLS thresholding, bird's eye view, turn target | 5, 6 |
| `master` | Ackermann steering, lane following, drive-to-point | 4, 7 |

## Topics

```
/cmd_drive            geometry_msgs/Twist         linear.x = pwm, angular.z = steering deg
/car_feedback         std_msgs/Int32MultiArray    [L ticks, R ticks, fwd cm/s, side cm/s]
/vision/turn_target   geometry_msgs/PointStamped  x = offset -1..+1, z = 1 if usable
/vision/debug_image   sensor_msgs/Image           birds eye view with the decision drawn on
```

## Build

```bash
cd ~/iris_fp1_ws
colcon build --packages-select communication vision master --symlink-install
source install/setup.bash
```

`src/simulation` is a ROS 1 workspace that runs inside the container, so it is
deliberately left out of the colcon build.

## Run

Start the simulator first:

```bash
./run_simulation.sh
```

Drive it by hand:

```bash
ros2 run communication communication_node     # terminal 1
ros2 run communication wasd_teleop_node       # terminal 2
```

`w`/`s` throttle, `a`/`d` steer, `space` stops the motor, `x` straightens the
wheels, `q` quits. Steering holds its last angle until you change it.

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

77 checks on the UDP wire format, the Ackermann geometry and the lane centre
decision, with plain g++ — no ROS, no simulator, no camera.

## Tuning

Everything is a live ROS parameter, so nothing needs a rebuild:

```bash
ros2 run rqt_image_view rqt_image_view        # watch /vision/debug_image

ros2 param set /vision_node min_lightness 150
ros2 param set /vision_node max_saturation 110
ros2 param set /vision_node horizon_row 0.58
ros2 param set /vision_node half_width_at_horizon 0.12

ros2 param set /master_node steer_gain -18.0
```

If `ros2 topic list` comes back nearly empty while the sim is clearly running,
the CLI's cached view is stale: `ros2 daemon stop`, then try again.

See `NOTES.md` for why the lane finder works the way it does, and for the
steering-sign gotcha.
