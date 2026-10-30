# Design notes

Why things are the way they are. Written for the demo session.

## One socket, one writer

Only `communication_node` touches UDP. Everything else publishes `/cmd_drive`.
Two nodes writing to the same port interleave their packets and the car
twitches, which looks like a control bug and isn't.

## The lane centre decision

`LaneFinder` thresholds in HLS, warps to a bird's eye view, then counts white
pixels per column inside a lookahead band and groups neighbouring columns into
*runs* — one run per marking.

The obvious alternative is to average every white pixel in the band. That is
wrong, and wrong in a way that passes a casual eye test. When one marking
leaves the frame on a bend, the average slides onto the marking that's left,
so the car is told it's centred at the exact moment it's about to leave the
road. Measured on synthetic frames:

| case | clustered | naive average |
|---|---|---|
| both markings, centred | −0.005 | −0.031 |
| both markings, lane 64 px right | +0.194 | +0.194 |
| left marking only, car centred | **−0.005** | **−0.503** |
| left marking only, car drifted right | −0.192 | −0.691 |
| right marking only, car centred | **−0.005** | **+0.494** |

The two bold rows are full-lock steering commands issued while the car is
perfectly centred.

Clustering fixes it by remembering the lane width from the last frame where
both markings were visible: one marking plus a known width still gives a
centre.

## Refusing to guess

One marking and *no* memory is genuinely ambiguous — nothing in the image says
which side of the car it's on. `LaneFinder` returns `CANT_TELL` with
`usable = false`, and `master_node` coasts on the previous steering angle
rather than steering on a coin flip. `vision_node` also calls `reset()` after
30 unusable frames, so a stale width from a different part of the track can't
mislead a later one-marking frame.

## Dead reckoning, and its limit

The car has no IMU. `Odometry` takes speed from the simulator's feedback and
integrates heading from the *commanded* steering angle via
`yaw_rate = speed · tan(steering) / wheelbase`. Position is integrated at the
midpoint heading, not the starting heading, because using the starting heading
drifts the path outward on every curve and the error grows with the step size.

Heading error accumulates. Fine over the tens of metres `goto_point` needs,
not fine over a lap.

## Throttle taper

`throttle_for_steering` drops PWM as the wheels turn. Entering a tight bend at
full throttle makes the car plough straight on, and no amount of steering gain
fixes that — it's a grip problem, not a control problem.

## Watchdogs

- `communication_node` zeroes the motor if `/cmd_drive` goes quiet for 0.5 s,
  but **holds the steering angle**. Centring the wheels at the same moment the
  motor cuts makes the car lurch sideways.
- `master_node` coasts when vision goes stale or returns `usable = false`.

## Steering sign

Positive steering turns left (ROS REP-103). A positive `offset_from_centre`
means the lane centre is to the *right*, so `steer_gain` is negative by
default. If the car swerves off immediately and consistently, the sign is
inverted for this setup — negate the gain, don't add to it:

```bash
ros2 param set /master_node steer_gain 25.0
```

## Tested without ROS

`udp_protocol.hpp`, `ackermann.hpp` and `lane_finder.hpp` are header-only and
ROS-free, so `./run_tests.sh` checks them with plain g++ — 77 assertions, no
simulator, no camera. The node files are thin wrappers around them.

## Confirmed against the live simulator

- camera topic is `/catvehicle/camera_front/image_raw_front`
- the packet layout works: `IRIS` header, little-endian int32s, feedback as
  left ticks / right ticks / forward cm/s / sideways cm/s
- `/cmd_drive` and `/car_feedback` are ours; the sim doesn't claim those names
- `/catvehicle/lidar_points` and `/catvehicle/front_laser_points` exist, for
  the optional obstacle-avoidance objective
