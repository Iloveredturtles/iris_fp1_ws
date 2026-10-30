// test_logic.cpp — exercises the three ROS-free headers with plain g++.
//
// Build and run with ./run_tests.sh. No ROS, no simulator, no camera: this is
// the part you can check before any of the rest of the system exists, and it
// is where the subtle bugs live.

#include "communication/udp_protocol.hpp"
#include "master/ackermann.hpp"
#include "vision/lane_finder.hpp"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace {

int g_failures = 0;
int g_checks = 0;

void check(bool ok, const char* expr, int line) {
  ++g_checks;
  if (!ok) {
    std::printf("  FAIL  line %d : %s\n", line, expr);
    ++g_failures;
  }
}

void checkNear(double got, double want, double tol, const char* expr, int line) {
  ++g_checks;
  if (std::fabs(got - want) > tol) {
    std::printf("  FAIL  line %d : %s = %.4f, expected %.4f +/- %.4f\n", line, expr, got,
                want, tol);
    ++g_failures;
  }
}

#define CHECK(cond) check((cond), #cond, __LINE__)
#define CHECK_NEAR(got, want, tol) checkNear((got), (want), (tol), #got, __LINE__)

void section(const char* name) { std::printf("\n%s\n", name); }

// ---------------------------------------------------------------------------
// UDP protocol
// ---------------------------------------------------------------------------
void testUdp() {
  section("udp_protocol");
  using namespace communication;

  unsigned char buf[kCommandPacketSize] = {0};
  DriveCommand cmd;
  cmd.motor_pwm = 100;
  cmd.steering_deg = -30;

  CHECK(buildCommandPacket(buf, cmd) == kCommandPacketSize);
  CHECK(kCommandPacketSize == 12);
  CHECK(kFeedbackPacketSize == 20);
  CHECK(buf[0] == 'I' && buf[1] == 'R' && buf[2] == 'I' && buf[3] == 'S');
  CHECK(getInt32(buf + 4) == 100);
  CHECK(getInt32(buf + 8) == -30);

  // Clamping: a runaway controller must not put nonsense on the wire.
  cmd.motor_pwm = 9999;
  cmd.steering_deg = 400;
  buildCommandPacket(buf, cmd);
  CHECK(getInt32(buf + 4) == 255);
  CHECK(getInt32(buf + 8) == 90);

  cmd.motor_pwm = -9999;
  cmd.steering_deg = -400;
  buildCommandPacket(buf, cmd);
  CHECK(getInt32(buf + 4) == -255);
  CHECK(getInt32(buf + 8) == -90);

  // Feedback round trip.
  unsigned char fbuf[kFeedbackPacketSize] = {0};
  std::memcpy(fbuf, kHeader, kHeaderSize);
  putInt32(fbuf + 4, 1234);
  putInt32(fbuf + 8, -5678);
  putInt32(fbuf + 12, 250);
  putInt32(fbuf + 16, -3);

  Feedback fb;
  CHECK(parseFeedbackPacket(fbuf, kFeedbackPacketSize, fb));
  CHECK(fb.left_ticks == 1234);
  CHECK(fb.right_ticks == -5678);
  CHECK(fb.vel_x_cms == 250);
  CHECK(fb.vel_y_cms == -3);

  // Rejections. UDP lets anything arrive on the port, so both of these
  // happen in practice and must not be parsed as data.
  CHECK(!parseFeedbackPacket(fbuf, kFeedbackPacketSize - 1, fb));
  fbuf[0] = 'X';
  CHECK(!parseFeedbackPacket(fbuf, kFeedbackPacketSize, fb));
}

// ---------------------------------------------------------------------------
// Ackermann / pure pursuit
// ---------------------------------------------------------------------------
void testAckermann() {
  section("ackermann");
  using namespace master;

  CHECK_NEAR(pursuitCurvature(5.0, 0.0), 0.0, 1e-9);
  CHECK_NEAR(pursuitCurvature(5.0, 1.0), 2.0 / 26.0, 1e-9);
  CHECK_NEAR(pursuitCurvature(0.0, 0.0), 0.0, 1e-9);  // degenerate, must not divide by zero

  // atan(2.6 * 0.076923) = atan(0.2) = 11.3099 deg
  CHECK_NEAR(curvatureToSteerDeg(2.0 / 26.0, 2.6), 11.3099, 1e-3);

  // A target to the left (y > 0) must produce a left turn (positive).
  CHECK(curvatureToSteerDeg(pursuitCurvature(5.0, 1.0), 2.6) > 0.0);
  CHECK(curvatureToSteerDeg(pursuitCurvature(5.0, -1.0), 2.6) < 0.0);

  CHECK_NEAR(speedForSteer(0.0, 35.0, 120.0, 60.0), 120.0, 1e-9);
  CHECK_NEAR(speedForSteer(35.0, 35.0, 120.0, 60.0), 60.0, 1e-9);
  CHECK_NEAR(speedForSteer(17.5, 35.0, 120.0, 60.0), 90.0, 1e-9);
  CHECK_NEAR(speedForSteer(90.0, 35.0, 120.0, 60.0), 60.0, 1e-9);  // past the limit, still clamped

  MotionLimits lim;  // wheelbase 2.6, limit 35, pwm 120/60, tol 0.5

  Motion near = motionToPoint(0.2, 0.0, lim);
  CHECK(near.arrived);
  CHECK_NEAR(near.pwm, 0.0, 1e-9);

  Motion ahead = motionToPoint(10.0, 0.0, lim);
  CHECK(!ahead.arrived);
  CHECK_NEAR(ahead.steer_deg, 0.0, 1e-6);
  CHECK_NEAR(ahead.pwm, 120.0, 1e-6);

  Motion left = motionToPoint(5.0, 2.0, lim);
  CHECK(left.steer_deg > 0.0);
  CHECK(left.pwm < 120.0);  // throttle tapered because the wheels are turned

  // Behind the car: full lock, crawl, never a zero-curvature straight line.
  Motion behind = motionToPoint(-5.0, 2.0, lim);
  CHECK_NEAR(behind.steer_deg, lim.steer_limit_deg, 1e-9);
  CHECK_NEAR(behind.pwm, lim.min_pwm, 1e-9);
  Motion behind_right = motionToPoint(-5.0, -2.0, lim);
  CHECK_NEAR(behind_right.steer_deg, -lim.steer_limit_deg, 1e-9);

  // Steering is clamped to the lock even when geometry asks for more.
  Motion sharp = motionToPoint(0.6, 0.6, lim);
  CHECK(std::fabs(sharp.steer_deg) <= lim.steer_limit_deg + 1e-9);

  section("odometry");
  Odometry odo;

  // Straight: 2 m/s for 1 s in 0.1 s steps.
  for (int i = 0; i < 10; ++i) {
    odo.update(2.0, 0.0, 2.6, 0.1);
  }
  CHECK_NEAR(odo.x(), 2.0, 1e-6);
  CHECK_NEAR(odo.y(), 0.0, 1e-9);
  CHECK_NEAR(odo.yaw(), 0.0, 1e-9);

  // A point straight ahead in the world is straight ahead in the car frame.
  double cx = 0.0;
  double cy = 0.0;
  odo.toCarFrame(5.0, 0.0, cx, cy);
  CHECK_NEAR(cx, 3.0, 1e-6);
  CHECK_NEAR(cy, 0.0, 1e-9);

  // Turning: yaw rate = v tan(d) / L = 2 * tan(35deg) / 2.6
  odo.reset();
  const double expected_rate = 2.0 * std::tan(toRad(35.0)) / 2.6;
  for (int i = 0; i < 100; ++i) {
    odo.update(2.0, 35.0, 2.6, 0.01);
  }
  CHECK_NEAR(odo.yaw(), expected_rate, 1e-6);
  CHECK(odo.y() > 0.0);  // left turn moves the car left

  // Rotation into the car frame. Facing +y (yaw 90 deg), a world point at
  // (0, 5) is dead ahead.
  odo.reset();
  const double rate = yawRate(1.0, 0.0, 2.6);
  CHECK_NEAR(rate, 0.0, 1e-9);
  for (int i = 0; i < 1000; ++i) {
    odo.update(0.0, 0.0, 2.6, 0.0);  // dt 0 must be a no-op, not a NaN
  }
  CHECK_NEAR(odo.x(), 0.0, 1e-9);
}

// ---------------------------------------------------------------------------
// Lane finder
// ---------------------------------------------------------------------------

// Make the perspective warp a plain resize so the test is about the run
// clustering and the centre decision, not about trapezoid geometry (which
// can only really be judged against a real camera image).
vision::LaneParams identityBev() {
  vision::LaneParams p;
  p.horizon_y = 0.0;
  p.bottom_y = 1.0;
  p.top_half_width = 0.5;
  p.bottom_half_width = 0.5;
  p.bev_size = 320;
  return p;
}

// Vertical white stripes on a dark background, 640x480.
cv::Mat stripes(const std::vector<int>& centres, int width_px = 20) {
  cv::Mat img(480, 640, CV_8UC3, cv::Scalar(20, 20, 20));
  for (int x : centres) {
    cv::rectangle(img, cv::Rect(x - width_px / 2, 0, width_px, 480),
                  cv::Scalar(255, 255, 255), cv::FILLED);
  }
  return img;
}

void testLaneFinder() {
  section("lane_finder: two lines");
  vision::LaneFinder finder(identityBev());

  // Centred lane: stripes at 160 and 480 straddle the frame centre of 320.
  vision::LaneResult r = finder.process(stripes({160, 480}));
  CHECK(r.found);
  CHECK(r.source == vision::LaneSource::kTwoLines);
  CHECK_NEAR(r.lateral_error, 0.0, 0.03);
  CHECK(r.left_x < r.right_x);
  CHECK_NEAR(r.lane_width, 159.5, 4.0);

  // Shift the whole lane right by 64 px: the car is now left of centre, so
  // the lane centre is to its right and the error must be positive.
  r = finder.process(stripes({224, 544}));
  CHECK(r.found);
  CHECK(r.source == vision::LaneSource::kTwoLines);
  CHECK_NEAR(r.lateral_error, 0.196, 0.03);

  // ...and left by 64 px, which must flip the sign.
  r = finder.process(stripes({96, 416}));
  CHECK(r.found);
  CHECK_NEAR(r.lateral_error, -0.196, 0.03);

  section("lane_finder: one line, with remembered width");
  // This is the case the naive implementation gets wrong. Re-establish the
  // lane, then drop the right-hand marking.
  finder.process(stripes({160, 480}));
  r = finder.process(stripes({160}));
  CHECK(r.found);
  CHECK(r.source == vision::LaneSource::kOneLine);
  // The remaining marking is the LEFT one, so the centre is half a lane to
  // its right — not on top of it. Averaging every lit pixel would report
  // roughly -0.5 here and drive the car onto the line.
  CHECK_NEAR(r.lateral_error, 0.0, 0.05);

  // Same, but the car has drifted right, so the left marking sits further
  // left in the image. The error must go negative: steer back to the left.
  finder.process(stripes({160, 480}));
  r = finder.process(stripes({100}));
  CHECK(r.found);
  CHECK(r.source == vision::LaneSource::kOneLine);
  CHECK(r.lateral_error < -0.1);

  section("lane_finder: refuses to guess");
  // One marking and no history: there is genuinely no way to know which side
  // of the car it is on. found must be false so master_node coasts instead
  // of steering on a coin flip.
  vision::LaneFinder fresh(identityBev());
  r = fresh.process(stripes({200}));
  CHECK(!r.found);
  CHECK(r.source == vision::LaneSource::kAmbiguous);

  // Nothing bright at all.
  vision::LaneFinder dark(identityBev());
  cv::Mat black(480, 640, CV_8UC3, cv::Scalar(10, 10, 10));
  r = dark.process(black);
  CHECK(!r.found);
  CHECK(r.source == vision::LaneSource::kNone);

  // Empty input must not crash.
  r = dark.process(cv::Mat());
  CHECK(!r.found);

  section("lane_finder: road surface");
  // A single wide bright slab is the road itself, not a marking.
  vision::LaneFinder surf(identityBev());
  r = surf.process(stripes({320}, 300));
  CHECK(r.found);
  CHECK(r.source == vision::LaneSource::kSurface);
  CHECK_NEAR(r.lateral_error, 0.0, 0.05);

  section("lane_finder: dashed markings merge into one run");
  // A dashed line is lit in some rows and not others, but the column profile
  // still sees it; two dashes a few px apart must not become two "lanes".
  vision::LaneFinder dash(identityBev());
  r = dash.process(stripes({160, 166, 480}));
  CHECK(r.found);
  CHECK(r.source == vision::LaneSource::kTwoLines);
  CHECK_NEAR(r.lateral_error, 0.01, 0.06);

  section("lane_finder: reset clears memory");
  vision::LaneFinder rst(identityBev());
  rst.process(stripes({160, 480}));
  rst.reset();
  r = rst.process(stripes({160}));
  CHECK(!r.found);  // width forgotten, so one line is ambiguous again
}

}  // namespace

int main() {
  std::printf("=== IRIS FP logic tests ===\n");
  testUdp();
  testAckermann();
  testLaneFinder();

  std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
  if (g_failures == 0) {
    std::printf("all passed\n");
  }
  return g_failures == 0 ? 0 : 1;
}
