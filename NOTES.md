# Design notes

Short record of why things are the way they are, for the demo session.

## One socket, one writer

Only `communication_node` touches UDP. Every other node publishes
`/cmd_drive`. Two nodes writing to the same port interleave packets and the
car twitches, which looks like a control bug and is not one.

## The lane-centre decision

`LaneFinder` thresholds in HLS, warps to a bird's eye view, then counts lit
pixels per column inside a lookahead band and groups adjacent columns into
*runs* — one run per marking.

The obvious alternative is to average every lit pixel in the band. That is
wrong, and wrong in a way that passes a casual eye test. When one marking
leaves the frame on a bend, the average collapses onto the marking that is
left, so the car is told it is centred at the exact moment it is about to
leave the road. Measured on synthetic frames:

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

One marking and *no* history is genuinely ambiguous — nothing in the image
says which side of the car it is on. `LaneFinder` returns `kAmbiguous` with
`found = false`, and `master_node` coasts on the previous steering angle
rather than steering on a coin flip. `vision_node` also calls `reset()` after
30 unusable frames so a stale width from a different part of the track cannot
mislead a later one-line frame.

## Dead reckoning, and its limit

The car has no IMU. `Odometry` takes speed from the simulator's feedback and
integrates heading from the *commanded* steering angle via
`yaw_rate = v·tan(δ)/L`. Position is integrated at the midpoint heading, not
the start heading, because using the start heading drifts the path outward on
a curve and the error grows with `dt`.

Heading error accumulates. Fine over the tens of metres `goto_point` needs,
not fine over a lap.

## Throttle taper

`speedForSteer` drops PWM as the wheels turn. Entering a tight bend at full
throttle makes the car understeer off the road, and no amount of steering gain
fixes that — it is a grip problem, not a control problem.

## Watchdogs

- `communication_node` zeroes the motor if `/cmd_drive` goes quiet for
  0.5 s, but **holds the steering angle**. Centring the wheels at the same
  moment the motor cuts makes the car lurch sideways.
- `master_node` coasts when vision goes stale or returns `found = false`.

## Steering sign

Positive steering angle turns left (ROS REP-103). A positive `lateral_error`
means the lane centre is to the *right*, so `steer_gain` is negative by
default. If the car swerves off the road immediately and consistently, the
sign is inverted for your setup — negate the gain, do not add to it:

```bash
ros2 param set /master_node steer_gain 25.0
```

## Tested without ROS

`udp_protocol.hpp`, `ackermann.hpp` and `lane_finder.hpp` are header-only and
ROS-free, so `./run_tests.sh` checks them with plain g++ — 75 assertions, no
simulator, no camera. The node files are thin wrappers around them.
