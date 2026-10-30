// test_logic.cpp
// checks the three header only bits with plain g++, no ros and no simulator
// run it with ./run_tests.sh

#include "communication/udp_protocol.hpp"
#include "master/ackermann.hpp"
#include "vision/lane_finder.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

int failures = 0;
int checks_done = 0;

void check(bool passed, const char* what, int line) {
    checks_done++;
    if (!passed) {
        printf("  FAIL  line %d : %s\n", line, what);
        failures++;
    }
}

void check_near(double got, double wanted, double tolerance, const char* what, int line) {
    checks_done++;
    if (fabs(got - wanted) > tolerance) {
        printf("  FAIL  line %d : %s = %.4f, wanted %.4f +/- %.4f\n",
               line, what, got, wanted, tolerance);
        failures++;
    }
}

#define CHECK(condition) check((condition), #condition, __LINE__)
#define CHECK_NEAR(got, wanted, tolerance) check_near((got), (wanted), (tolerance), #got, __LINE__)

void section(const char* name) { printf("\n%s\n", name); }

// ---------------------------------------------------------------------------
void test_udp_protocol() {
    section("udp protocol");
    using namespace communication;

    unsigned char packet[COMMAND_PACKET_BYTES];
    memset(packet, 0, sizeof(packet));

    DriveCommand command;
    command.motor_pwm = 100;
    command.steering_deg = -30;
    build_command(packet, command);

    CHECK(COMMAND_PACKET_BYTES == 12);
    CHECK(FEEDBACK_PACKET_BYTES == 20);
    CHECK(packet[0] == 'I' && packet[1] == 'R' && packet[2] == 'I' && packet[3] == 'S');
    CHECK(get_int(packet + 4) == 100);
    CHECK(get_int(packet + 8) == -30);

    // a runaway controller must not put nonsense on the wire
    command.motor_pwm = 9999;
    command.steering_deg = 400;
    build_command(packet, command);
    CHECK(get_int(packet + 4) == 255);
    CHECK(get_int(packet + 8) == 90);

    command.motor_pwm = -9999;
    command.steering_deg = -400;
    build_command(packet, command);
    CHECK(get_int(packet + 4) == -255);
    CHECK(get_int(packet + 8) == -90);

    // feedback round trip
    unsigned char feedback_packet[FEEDBACK_PACKET_BYTES];
    memset(feedback_packet, 0, sizeof(feedback_packet));
    memcpy(feedback_packet, PACKET_HEADER, HEADER_BYTES);
    put_int(feedback_packet + 4, 1234);
    put_int(feedback_packet + 8, -5678);
    put_int(feedback_packet + 12, 250);
    put_int(feedback_packet + 16, -3);

    Feedback feedback;
    CHECK(parse_feedback(feedback_packet, FEEDBACK_PACKET_BYTES, feedback));
    CHECK(feedback.left_wheel_ticks == 1234);
    CHECK(feedback.right_wheel_ticks == -5678);
    CHECK(feedback.forward_speed_cms == 250);
    CHECK(feedback.sideways_speed_cms == -3);

    // anything can arrive on a udp port, so both of these happen for real
    CHECK(!parse_feedback(feedback_packet, FEEDBACK_PACKET_BYTES - 1, feedback));
    feedback_packet[0] = 'X';
    CHECK(!parse_feedback(feedback_packet, FEEDBACK_PACKET_BYTES, feedback));
}

// ---------------------------------------------------------------------------
void test_ackermann() {
    section("ackermann");
    using namespace master;

    CHECK_NEAR(pursuit_curvature(5.0, 0.0), 0.0, 1e-9);
    CHECK_NEAR(pursuit_curvature(5.0, 1.0), 2.0 / 26.0, 1e-9);
    CHECK_NEAR(pursuit_curvature(0.0, 0.0), 0.0, 1e-9);   // must not divide by zero

    // atan(2.6 * 0.076923) = atan(0.2) = 11.3099 degrees
    CHECK_NEAR(curvature_to_steering_deg(2.0 / 26.0, 2.6), 11.3099, 1e-3);

    // a target to the left (y positive) has to give a left turn (positive)
    CHECK(curvature_to_steering_deg(pursuit_curvature(5.0, 1.0), 2.6) > 0.0);
    CHECK(curvature_to_steering_deg(pursuit_curvature(5.0, -1.0), 2.6) < 0.0);

    CHECK_NEAR(throttle_for_steering(0.0, 35.0, 120.0, 60.0), 120.0, 1e-9);
    CHECK_NEAR(throttle_for_steering(35.0, 35.0, 120.0, 60.0), 60.0, 1e-9);
    CHECK_NEAR(throttle_for_steering(17.5, 35.0, 120.0, 60.0), 90.0, 1e-9);
    CHECK_NEAR(throttle_for_steering(90.0, 35.0, 120.0, 60.0), 60.0, 1e-9);  // past the lock, still clamped

    DrivingLimits limits;   // wheelbase 2.6, lock 35, pwm 120/60, arrive 0.5

    DriveDecision close_enough = drive_towards(0.2, 0.0, limits);
    CHECK(close_enough.arrived);
    CHECK_NEAR(close_enough.pwm, 0.0, 1e-9);

    DriveDecision straight_ahead = drive_towards(10.0, 0.0, limits);
    CHECK(!straight_ahead.arrived);
    CHECK_NEAR(straight_ahead.steering_deg, 0.0, 1e-6);
    CHECK_NEAR(straight_ahead.pwm, 120.0, 1e-6);

    DriveDecision off_to_the_left = drive_towards(5.0, 2.0, limits);
    CHECK(off_to_the_left.steering_deg > 0.0);
    CHECK(off_to_the_left.pwm < 120.0);   // throttle eased because the wheels are turned

    // behind us: full lock and a crawl, never a straight line
    DriveDecision behind_left = drive_towards(-5.0, 2.0, limits);
    CHECK_NEAR(behind_left.steering_deg, limits.max_steering_deg, 1e-9);
    CHECK_NEAR(behind_left.pwm, limits.min_pwm, 1e-9);

    DriveDecision behind_right = drive_towards(-5.0, -2.0, limits);
    CHECK_NEAR(behind_right.steering_deg, -limits.max_steering_deg, 1e-9);

    // geometry can ask for more lock than the car has
    DriveDecision very_sharp = drive_towards(0.6, 0.6, limits);
    CHECK(fabs(very_sharp.steering_deg) <= limits.max_steering_deg + 1e-9);

    section("odometry");
    Odometry odometry;

    // straight line, 2 m/s for 1 second in ten steps
    for (int i = 0; i < 10; i++) {
        odometry.update(2.0, 0.0, 2.6, 0.1);
    }
    CHECK_NEAR(odometry.position_x_m, 2.0, 1e-6);
    CHECK_NEAR(odometry.position_y_m, 0.0, 1e-9);
    CHECK_NEAR(odometry.heading_rad, 0.0, 1e-9);

    // a point ahead in the world is still ahead of the car
    double ahead = 0.0;
    double to_the_left = 0.0;
    odometry.to_car_frame(5.0, 0.0, ahead, to_the_left);
    CHECK_NEAR(ahead, 3.0, 1e-6);
    CHECK_NEAR(to_the_left, 0.0, 1e-9);

    // turning: yaw rate = speed * tan(steering) / wheelbase
    odometry.reset();
    double expected_rate = 2.0 * tan(to_radians(35.0)) / 2.6;
    for (int i = 0; i < 100; i++) {
        odometry.update(2.0, 35.0, 2.6, 0.01);
    }
    CHECK_NEAR(odometry.heading_rad, expected_rate, 1e-6);
    CHECK(odometry.position_y_m > 0.0);   // turning left moves us left

    // a zero length step must change nothing and must not produce NaN
    odometry.reset();
    for (int i = 0; i < 100; i++) {
        odometry.update(5.0, 20.0, 2.6, 0.0);
    }
    CHECK_NEAR(odometry.position_x_m, 0.0, 1e-9);
    CHECK_NEAR(odometry.heading_rad, 0.0, 1e-9);

    CHECK_NEAR(yaw_rate_rads(1.0, 0.0, 2.6), 0.0, 1e-9);
}

// ---------------------------------------------------------------------------

// make the warp a plain resize so these tests are about the run grouping and
// the centre decision, not about trapezoid geometry, which you can only really
// judge against a real camera picture
vision::LaneSettings plain_warp_settings() {
    vision::LaneSettings settings;
    settings.horizon_row = 0.0;
    settings.bottom_row = 1.0;
    settings.half_width_at_horizon = 0.5;
    settings.half_width_at_bottom = 0.5;
    settings.warped_size_px = 320;
    return settings;
}

// vertical white stripes on a dark background, 640 x 480
cv::Mat white_stripes(const std::vector<int>& centres, int stripe_width = 20) {
    cv::Mat picture(480, 640, CV_8UC3, cv::Scalar(20, 20, 20));
    for (size_t i = 0; i < centres.size(); i++) {
        cv::rectangle(picture,
                      cv::Rect(centres[i] - stripe_width / 2, 0, stripe_width, 480),
                      cv::Scalar(255, 255, 255), cv::FILLED);
    }
    return picture;
}

void test_lane_finder() {
    section("lane finder, both markings");
    vision::LaneFinder finder(plain_warp_settings());

    // stripes at 160 and 480 sit either side of the frame centre at 320
    vision::LaneResult result = finder.process(white_stripes({160, 480}));
    CHECK(result.usable);
    CHECK(result.source == vision::LaneSource::BOTH_MARKINGS);
    CHECK_NEAR(result.offset_from_centre, 0.0, 0.03);
    CHECK(result.left_marking_x < result.right_marking_x);
    CHECK_NEAR(result.lane_width_px, 159.5, 4.0);

    // shift the lane 64 px right: the centre is now to our right, so positive
    result = finder.process(white_stripes({224, 544}));
    CHECK(result.usable);
    CHECK_NEAR(result.offset_from_centre, 0.196, 0.03);

    // and left, which has to flip the sign
    result = finder.process(white_stripes({96, 416}));
    CHECK(result.usable);
    CHECK_NEAR(result.offset_from_centre, -0.196, 0.03);

    section("lane finder, one marking plus a remembered width");
    // this is the case the naive version gets wrong. see both markings, then
    // drop the right hand one
    finder.process(white_stripes({160, 480}));
    result = finder.process(white_stripes({160}));
    CHECK(result.usable);
    CHECK(result.source == vision::LaneSource::ONE_MARKING);
    // the marking left in view is the LEFT one, so the centre is half a lane
    // to its right, not on top of it. averaging every white pixel would say
    // about -0.5 here and steer us straight onto the line
    CHECK_NEAR(result.offset_from_centre, 0.0, 0.05);

    // same again but the car has drifted right, so the left marking is further
    // left in the picture. the offset has to go negative: steer back left
    finder.process(white_stripes({160, 480}));
    result = finder.process(white_stripes({100}));
    CHECK(result.usable);
    CHECK(result.source == vision::LaneSource::ONE_MARKING);
    CHECK(result.offset_from_centre < -0.1);

    // and the mirror image: only the RIGHT marking, car centred
    finder.process(white_stripes({160, 480}));
    result = finder.process(white_stripes({480}));
    CHECK(result.usable);
    CHECK(result.source == vision::LaneSource::ONE_MARKING);
    CHECK_NEAR(result.offset_from_centre, 0.0, 0.05);

    section("lane finder, wont guess a side it cant tell");
    // the marking sits almost exactly where we last saw the lane centre, so
    // which side of the car it is on is a coin flip. getting it wrong puts
    // the centre a FULL LANE out and steers the car off the road, so it has
    // to refuse. this one cost a corner before it was caught
    finder.process(white_stripes({160, 480}));          // centre at BEV ~159
    result = finder.process(white_stripes({320}));      // marking at BEV ~159
    CHECK(!result.usable);
    CHECK(result.source == vision::LaneSource::CANT_TELL);

    // and just outside the margin it is willing again
    finder.process(white_stripes({160, 480}));
    result = finder.process(white_stripes({420}));      // BEV ~209, clear of centre
    CHECK(result.usable);
    CHECK(result.source == vision::LaneSource::ONE_MARKING);

    section("lane finder, refuses to guess");
    // one marking and no memory. nothing in the picture says which side of the
    // car it is on, so usable must be false and master_node coasts
    vision::LaneFinder fresh_finder(plain_warp_settings());
    result = fresh_finder.process(white_stripes({200}));
    CHECK(!result.usable);
    CHECK(result.source == vision::LaneSource::CANT_TELL);

    // nothing bright at all
    vision::LaneFinder dark_finder(plain_warp_settings());
    cv::Mat all_dark(480, 640, CV_8UC3, cv::Scalar(10, 10, 10));
    result = dark_finder.process(all_dark);
    CHECK(!result.usable);
    CHECK(result.source == vision::LaneSource::NOTHING_FOUND);

    // an empty frame must not crash
    result = dark_finder.process(cv::Mat());
    CHECK(!result.usable);

    section("lane finder, road surface");
    // one very wide bright slab is the road, not a painted line
    vision::LaneFinder surface_finder(plain_warp_settings());
    result = surface_finder.process(white_stripes({320}, 300));
    CHECK(result.usable);
    CHECK(result.source == vision::LaneSource::ROAD_SURFACE);
    CHECK_NEAR(result.offset_from_centre, 0.0, 0.05);

    section("lane finder, dashes join into one marking");
    // two dashes a few pixels apart must not look like two sides of a lane
    vision::LaneFinder dashed_finder(plain_warp_settings());
    result = dashed_finder.process(white_stripes({160, 166, 480}));
    CHECK(result.usable);
    CHECK(result.source == vision::LaneSource::BOTH_MARKINGS);
    CHECK_NEAR(result.offset_from_centre, 0.01, 0.06);

    section("lane finder, reset forgets the width");
    vision::LaneFinder resettable_finder(plain_warp_settings());
    resettable_finder.process(white_stripes({160, 480}));
    resettable_finder.reset();
    result = resettable_finder.process(white_stripes({160}));
    CHECK(!result.usable);   // width forgotten, so one marking is ambiguous again
}

int main() {
    printf("=== IRIS FP logic tests ===\n");
    test_udp_protocol();
    test_ackermann();
    test_lane_finder();

    printf("\n%d checks, %d failures\n", checks_done, failures);
    if (failures == 0) printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
