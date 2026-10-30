// ackermann.hpp — objective 4: steering geometry and "drive to a point".
//
// Header-only and ROS-free so it can be unit tested with plain g++.
//
// Frame convention, which everything here depends on (ROS REP-103):
//   x forward, y LEFT, yaw counter-clockwise, steering angle positive = left.
//
// A car is not a differential-drive robot: it cannot turn on the spot, and
// the tightest circle it can drive is set by its wheelbase and steering lock.
// Ackermann geometry is the relation between the two:
//
//     tan(steer) = wheelbase / turn_radius = wheelbase * curvature
//
// so a desired curvature becomes a steering angle with one atan.

#ifndef MASTER_ACKERMANN_HPP
#define MASTER_ACKERMANN_HPP

#include <algorithm>
#include <cmath>

namespace master {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kDefaultWheelbaseM = 2.6;

inline double toDeg(double r) { return r * 180.0 / kPi; }
inline double toRad(double d) { return d * kPi / 180.0; }

// Pure pursuit. Given a target at (x, y) in the car's frame, the curvature of
// the circular arc that starts at the car, points along +x, and passes through
// the target is 2y / Ld^2, where Ld is the straight-line distance.
//
// Why an arc rather than "point the wheels at the target": the car travels
// along a curve, so aiming straight at the target overshoots and then has to
// come back, which reads as weaving.
inline double pursuitCurvature(double x, double y) {
  const double ld2 = x * x + y * y;
  if (ld2 < 1e-9) {
    return 0.0;
  }
  return 2.0 * y / ld2;
}

inline double curvatureToSteerDeg(double curvature, double wheelbase) {
  return toDeg(std::atan(wheelbase * curvature));
}

// How fast the car rotates for a given speed and steering angle. Used to dead
// reckon heading when there is no IMU.
inline double yawRate(double speed_mps, double steer_deg, double wheelbase) {
  if (wheelbase < 1e-9) {
    return 0.0;
  }
  return speed_mps * std::tan(toRad(steer_deg)) / wheelbase;
}

// Throttle tapers as the wheels turn. Entering a tight bend at full PWM makes
// the car understeer straight off the road, and no amount of steering gain
// fixes that — it is a tyre problem, not a control problem.
inline double speedForSteer(double steer_deg, double steer_limit_deg, double max_pwm,
                            double min_pwm) {
  if (steer_limit_deg < 1e-9) {
    return max_pwm;
  }
  const double f = std::min(1.0, std::fabs(steer_deg) / steer_limit_deg);
  return max_pwm - (max_pwm - min_pwm) * f;
}

struct MotionLimits {
  double wheelbase = kDefaultWheelbaseM;
  double steer_limit_deg = 35.0;
  double max_pwm = 120.0;
  double min_pwm = 60.0;
  double arrive_tol_m = 0.5;
};

struct Motion {
  double pwm = 0.0;
  double steer_deg = 0.0;
  bool arrived = false;
};

// Target given in the CAR's frame. Use Odometry::toCarFrame to convert a
// world-frame goal first.
inline Motion motionToPoint(double x, double y, const MotionLimits& lim) {
  Motion m;

  const double dist = std::hypot(x, y);
  if (dist <= lim.arrive_tol_m) {
    m.arrived = true;
    return m;  // pwm and steer stay at zero
  }

  if (x <= 0.0) {
    // The target is behind the car. Pure pursuit is undefined here (the arc
    // would need infinite curvature), and a car cannot pivot, so crawl
    // forward on full lock and let the target swing round to the front.
    m.steer_deg = (y >= 0.0) ? lim.steer_limit_deg : -lim.steer_limit_deg;
    m.pwm = lim.min_pwm;
    return m;
  }

  const double curvature = pursuitCurvature(x, y);
  m.steer_deg = std::clamp(curvatureToSteerDeg(curvature, lim.wheelbase),
                           -lim.steer_limit_deg, lim.steer_limit_deg);
  m.pwm = speedForSteer(m.steer_deg, lim.steer_limit_deg, lim.max_pwm, lim.min_pwm);
  return m;
}

// Dead-reckoned pose. Speed comes from the simulator's feedback; heading is
// integrated from the commanded steering angle, because there is no IMU on
// this car. That means heading error accumulates — fine over the tens of
// metres this project asks for, not fine over a lap.
class Odometry {
 public:
  void reset() {
    x_ = 0.0;
    y_ = 0.0;
    yaw_ = 0.0;
  }

  void update(double speed_mps, double steer_deg, double wheelbase, double dt) {
    if (dt <= 0.0) {
      return;
    }
    const double w = yawRate(speed_mps, steer_deg, wheelbase);
    // Integrate position at the midpoint heading rather than the start
    // heading: on a curve, using the start heading consistently drifts the
    // path outward, and the error grows with dt.
    const double yaw_mid = yaw_ + 0.5 * w * dt;
    x_ += speed_mps * std::cos(yaw_mid) * dt;
    y_ += speed_mps * std::sin(yaw_mid) * dt;
    yaw_ += w * dt;
  }

  double x() const { return x_; }
  double y() const { return y_; }
  double yaw() const { return yaw_; }

  // World-frame point -> car-frame point.
  void toCarFrame(double gx, double gy, double& cx, double& cy) const {
    const double dx = gx - x_;
    const double dy = gy - y_;
    const double c = std::cos(yaw_);
    const double s = std::sin(yaw_);
    cx = dx * c + dy * s;
    cy = -dx * s + dy * c;
  }

 private:
  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
};

}  // namespace master

#endif  // MASTER_ACKERMANN_HPP
