// ackermann.hpp
// steering maths for a car, and working out where the car has got to

#ifndef ACKERMANN_HPP
#define ACKERMANN_HPP

#include <cmath>

namespace master {

const double PI = 3.14159265358979323846;
const double DEFAULT_WHEELBASE_M = 2.6;

// everywhere in here: x is forward, y is left, steering is positive going left

inline double to_degrees(double radians) { return radians * 180.0 / PI; }
inline double to_radians(double degrees) { return degrees * PI / 180.0; }

// pure pursuit. the curve that starts at the car, points straight forward,
// and passes through the target. we follow a curve rather than aiming the
// wheels straight at the target, or the car overshoots and weaves back
inline double pursuit_curvature(double target_x, double target_y) {
    double distance_squared = target_x * target_x + target_y * target_y;
    if (distance_squared < 1e-9) return 0.0;
    return 2.0 * target_y / distance_squared;
}

// ackermann geometry: tan(steering) = wheelbase * curvature
inline double curvature_to_steering_deg(double curvature, double wheelbase_m) {
    return to_degrees(atan(wheelbase_m * curvature));
}

// how fast the car rotates for a given speed and steering angle
inline double yaw_rate_rads(double speed_mps, double steering_deg, double wheelbase_m) {
    if (wheelbase_m < 1e-9) return 0.0;
    return speed_mps * tan(to_radians(steering_deg)) / wheelbase_m;
}

// ease off the throttle as the wheels turn. going into a tight bend at full
// speed just makes the car plough straight on, and more steering wont fix that
inline double throttle_for_steering(double steering_deg, double max_steering_deg,
                                    double max_pwm, double min_pwm) {
    if (max_steering_deg < 1e-9) return max_pwm;

    double how_hard_turning = fabs(steering_deg) / max_steering_deg;
    if (how_hard_turning > 1.0) how_hard_turning = 1.0;

    return max_pwm - (max_pwm - min_pwm) * how_hard_turning;
}

struct DrivingLimits {
    double wheelbase_m = DEFAULT_WHEELBASE_M;
    double max_steering_deg = 35.0;
    double max_pwm = 120.0;
    double min_pwm = 60.0;
    double arrived_distance_m = 0.5;
};

struct DriveDecision {
    double pwm = 0.0;
    double steering_deg = 0.0;
    bool arrived = false;
};

// target_x and target_y are measured from the car, not from the world origin
inline DriveDecision drive_towards(double target_x, double target_y,
                                   const DrivingLimits& limits) {
    DriveDecision decision;
    double distance_m = sqrt(target_x * target_x + target_y * target_y);

    if (distance_m <= limits.arrived_distance_m) {
        decision.arrived = true;
        return decision;
    }

    // target is behind us. a car cant spin on the spot, so crawl forward on
    // full lock and let the target come round to the front
    if (target_x <= 0.0) {
        decision.steering_deg = (target_y >= 0.0) ? limits.max_steering_deg
                                                  : -limits.max_steering_deg;
        decision.pwm = limits.min_pwm;
        return decision;
    }

    double curvature = pursuit_curvature(target_x, target_y);
    double steering_deg = curvature_to_steering_deg(curvature, limits.wheelbase_m);

    if (steering_deg >  limits.max_steering_deg) steering_deg =  limits.max_steering_deg;
    if (steering_deg < -limits.max_steering_deg) steering_deg = -limits.max_steering_deg;

    decision.steering_deg = steering_deg;
    decision.pwm = throttle_for_steering(steering_deg, limits.max_steering_deg,
                                         limits.max_pwm, limits.min_pwm);
    return decision;
}

// where the car has got to. there is no gps and no imu, so we add up speed and
// steering over time. the error creeps up slowly, which is fine for a short
// trip across the map but not for a whole lap
class Odometry {
public:
    void reset() {
        position_x_m = 0.0;
        position_y_m = 0.0;
        heading_rad = 0.0;
    }

    void update(double speed_mps, double steering_deg, double wheelbase_m, double seconds) {
        if (seconds <= 0.0) return;

        double turn_rate = yaw_rate_rads(speed_mps, steering_deg, wheelbase_m);

        // move along the heading halfway through this step, not the heading we
        // started with, or every curve drifts outward a little each time
        double middle_heading = heading_rad + 0.5 * turn_rate * seconds;

        position_x_m += speed_mps * cos(middle_heading) * seconds;
        position_y_m += speed_mps * sin(middle_heading) * seconds;
        heading_rad  += turn_rate * seconds;
    }

    // take a point in the world and say where it is relative to the car
    void to_car_frame(double world_x, double world_y,
                      double& car_x, double& car_y) const {
        double offset_x = world_x - position_x_m;
        double offset_y = world_y - position_y_m;

        car_x =  offset_x * cos(heading_rad) + offset_y * sin(heading_rad);
        car_y = -offset_x * sin(heading_rad) + offset_y * cos(heading_rad);
    }

    double position_x_m = 0.0;
    double position_y_m = 0.0;
    double heading_rad = 0.0;
};

}  // namespace master

#endif
