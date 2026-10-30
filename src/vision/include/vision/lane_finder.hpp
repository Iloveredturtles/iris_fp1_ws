// lane_finder.hpp — objectives 5 and 6: find the road and say which way to turn.
//
// Header-only and ROS-free so it can be tested against synthetic images with
// plain g++ and OpenCV. The node is a thin wrapper around this.
//
// Pipeline:
//   1. threshold in HLS            -> binary mask of the lane markings
//   2. perspective warp            -> bird's eye view, where a lane is two
//                                     roughly vertical stripes a fixed
//                                     distance apart
//   3. column profile in a band    -> where the markings are, left to right
//   4. group columns into runs     -> one run per marking
//   5. decide the lane centre      -> lateral error in -1 .. +1
//
// Step 4 is the part worth caring about. The obvious implementation — average
// every lit pixel in the band — is wrong, and wrong in a way that passes a
// casual eye test. When one of the two markings leaves the frame on a bend,
// the average collapses onto the marking that is left, so the car is told it
// is perfectly centred at the exact moment it is about to leave the road.
// Clustering into runs and remembering the lane width fixes it: one visible
// marking plus a known width still gives a centre.

#ifndef VISION_LANE_FINDER_HPP
#define VISION_LANE_FINDER_HPP

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace vision {

enum class LaneSource {
  kNone,       // nothing bright enough in the band
  kTwoLines,   // both markings visible — the good case
  kOneLine,    // one marking plus a remembered lane width
  kSurface,    // one very wide blob, treated as the road surface
  kAmbiguous,  // one marking and no history, so no way to know which side
};

inline const char* toString(LaneSource s) {
  switch (s) {
    case LaneSource::kNone: return "none";
    case LaneSource::kTwoLines: return "two-lines";
    case LaneSource::kOneLine: return "one-line";
    case LaneSource::kSurface: return "surface";
    case LaneSource::kAmbiguous: return "ambiguous";
  }
  return "?";
}

struct LaneParams {
  // --- marking threshold, HLS ---------------------------------------------
  // White paint on asphalt is bright and nearly colourless, so a high L with
  // a low S ceiling picks it out and ignores coloured scenery.
  int l_min = 170;
  int s_max = 90;
  int morph_open = 2;  // kills single-pixel speckle before clustering

  // --- bird's eye view ----------------------------------------------------
  // Source trapezoid as fractions of the input frame. The defaults assume a
  // forward-facing camera mounted near the middle of the car; expect to tune
  // horizon_y and top_half_width once you see the warped image.
  double horizon_y = 0.62;         // top edge of the trapezoid
  double bottom_y = 0.98;          // bottom edge
  double top_half_width = 0.10;    // half-width at the horizon
  double bottom_half_width = 0.52; // half-width at the bottom
  int bev_size = 320;              // output is bev_size x bev_size

  // --- lookahead band inside the BEV --------------------------------------
  // 0.0 is the far edge, 1.0 is right at the bumper. Looking slightly ahead
  // rather than at the bumper stops the steering oscillating.
  double band_far = 0.20;
  double band_near = 0.55;

  // --- run clustering -----------------------------------------------------
  int min_column_hits = 2;    // a column needs this many lit pixels to count
  int min_run_px = 3;         // narrower than this is noise
  int merge_gap_px = 4;       // runs closer than this are one dashed marking
  int max_line_run_px = 70;   // wider than this is surface, not a line
  double min_lane_frac = 0.20;// two runs closer than this are the same marking
};

struct LaneResult {
  bool found = false;          // false means do not steer on this frame
  double lateral_error = 0.0;  // -1 fully left of centre, +1 fully right
  LaneSource source = LaneSource::kNone;
  double centre_x = -1.0;      // in BEV pixels
  double left_x = -1.0;
  double right_x = -1.0;
  double lane_width = -1.0;
};

namespace detail {

struct Run {
  int begin = 0;
  int end = 0;   // exclusive
  long mass = 0;

  int width() const { return end - begin; }
  double centre() const { return 0.5 * (begin + end - 1); }
};

}  // namespace detail

class LaneFinder {
 public:
  LaneFinder() = default;
  explicit LaneFinder(const LaneParams& params) : params_(params) {}

  LaneParams& params() { return params_; }
  const LaneParams& params() const { return params_; }

  // Forget the remembered lane geometry. Call this when the car is replaced
  // in the world, or after a long stretch of lost frames, so a stale width
  // from a different part of the track cannot mislead the one-line case.
  void reset() {
    lane_width_ = -1.0;
    last_left_x_ = -1.0;
    last_right_x_ = -1.0;
    last_centre_x_ = -1.0;
  }

  const cv::Mat& mask() const { return mask_; }
  const cv::Mat& bev() const { return bev_; }

  LaneResult process(const cv::Mat& bgr) {
    LaneResult result;
    if (bgr.empty()) {
      return result;
    }

    buildMask(bgr);
    warpToBev();
    const std::vector<detail::Run> runs = findRuns();
    result = decide(runs);
    remember(result);
    return result;
  }

  // BEV with the decision drawn on it, for /vision/debug_image.
  cv::Mat debugImage(const LaneResult& r) const {
    if (bev_.empty()) {
      return cv::Mat();
    }
    cv::Mat out;
    cv::cvtColor(bev_, out, cv::COLOR_GRAY2BGR);

    const int h = out.rows;
    const int y0 = static_cast<int>(params_.band_far * h);
    const int y1 = static_cast<int>(params_.band_near * h);
    cv::rectangle(out, cv::Point(0, y0), cv::Point(out.cols - 1, y1),
                  cv::Scalar(60, 60, 60), 1);

    const int mid = out.cols / 2;
    cv::line(out, cv::Point(mid, 0), cv::Point(mid, h - 1), cv::Scalar(120, 120, 120), 1);

    if (r.left_x >= 0) {
      cv::line(out, cv::Point(static_cast<int>(r.left_x), y0),
               cv::Point(static_cast<int>(r.left_x), y1), cv::Scalar(0, 180, 255), 2);
    }
    if (r.right_x >= 0) {
      cv::line(out, cv::Point(static_cast<int>(r.right_x), y0),
               cv::Point(static_cast<int>(r.right_x), y1), cv::Scalar(0, 180, 255), 2);
    }
    if (r.found && r.centre_x >= 0) {
      cv::line(out, cv::Point(static_cast<int>(r.centre_x), y0),
               cv::Point(static_cast<int>(r.centre_x), y1), cv::Scalar(0, 255, 0), 2);
    }

    cv::putText(out, cv::format("%s  err %+.2f", toString(r.source), r.lateral_error),
                cv::Point(6, 18), cv::FONT_HERSHEY_SIMPLEX, 0.45,
                r.found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 140, 255), 1, cv::LINE_AA);
    return out;
  }

 private:
  void buildMask(const cv::Mat& bgr) {
    cv::Mat hls;
    cv::cvtColor(bgr, hls, cv::COLOR_BGR2HLS);
    // HLS channel order is H, L, S. Hue is left wide open on purpose: the
    // markings are defined by being bright and colourless, not by a hue.
    cv::inRange(hls,
                cv::Scalar(0, params_.l_min, 0),
                cv::Scalar(179, 255, params_.s_max),
                mask_);

    if (params_.morph_open > 0) {
      const cv::Mat k = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
      cv::morphologyEx(mask_, mask_, cv::MORPH_OPEN, k, cv::Point(-1, -1),
                       params_.morph_open);
    }
  }

  void warpToBev() {
    const double w = mask_.cols;
    const double h = mask_.rows;
    const double cx = 0.5 * w;

    const cv::Point2f src[4] = {
        cv::Point2f(static_cast<float>(cx - params_.top_half_width * w),
                    static_cast<float>(params_.horizon_y * h)),
        cv::Point2f(static_cast<float>(cx + params_.top_half_width * w),
                    static_cast<float>(params_.horizon_y * h)),
        cv::Point2f(static_cast<float>(cx + params_.bottom_half_width * w),
                    static_cast<float>(params_.bottom_y * h)),
        cv::Point2f(static_cast<float>(cx - params_.bottom_half_width * w),
                    static_cast<float>(params_.bottom_y * h)),
    };

    const float s = static_cast<float>(params_.bev_size);
    const cv::Point2f dst[4] = {
        cv::Point2f(0.0f, 0.0f),
        cv::Point2f(s - 1.0f, 0.0f),
        cv::Point2f(s - 1.0f, s - 1.0f),
        cv::Point2f(0.0f, s - 1.0f),
    };

    const cv::Mat m = cv::getPerspectiveTransform(src, dst);
    cv::warpPerspective(mask_, bev_, m, cv::Size(params_.bev_size, params_.bev_size),
                        cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
  }

  // Column-wise hit counts inside the lookahead band, grouped into runs of
  // adjacent lit columns.
  std::vector<detail::Run> findRuns() const {
    std::vector<detail::Run> runs;
    if (bev_.empty()) {
      return runs;
    }

    const int h = bev_.rows;
    const int w = bev_.cols;
    const int y0 = std::clamp(static_cast<int>(params_.band_far * h), 0, h - 1);
    const int y1 = std::clamp(static_cast<int>(params_.band_near * h), y0 + 1, h);

    std::vector<int> profile(static_cast<std::size_t>(w), 0);
    for (int y = y0; y < y1; ++y) {
      const unsigned char* row = bev_.ptr<unsigned char>(y);
      for (int x = 0; x < w; ++x) {
        if (row[x] != 0) {
          ++profile[static_cast<std::size_t>(x)];
        }
      }
    }

    // Walk the profile, opening a run on the first lit column and closing it
    // after merge_gap_px dark columns, so a dashed line stays one run.
    int begin = -1;
    int gap = 0;
    long mass = 0;

    for (int x = 0; x < w; ++x) {
      const bool lit = profile[static_cast<std::size_t>(x)] >= params_.min_column_hits;
      if (lit) {
        if (begin < 0) {
          begin = x;
          mass = 0;
        }
        gap = 0;
        mass += profile[static_cast<std::size_t>(x)];
      } else if (begin >= 0) {
        ++gap;
        if (gap > params_.merge_gap_px) {
          const int end = x - gap + 1;
          if (end - begin >= params_.min_run_px) {
            runs.push_back({begin, end, mass});
          }
          begin = -1;
          gap = 0;
          mass = 0;
        }
      }
    }
    if (begin >= 0) {
      const int end = w - gap;
      if (end - begin >= params_.min_run_px) {
        runs.push_back({begin, end, mass});
      }
    }
    return runs;
  }

  LaneResult decide(const std::vector<detail::Run>& runs) {
    LaneResult r;
    const double w = static_cast<double>(params_.bev_size);
    const double half = 0.5 * w;

    if (runs.empty()) {
      r.source = LaneSource::kNone;
      return r;
    }

    // A single very wide blob is the road surface rather than a marking —
    // some tracks are a light slab on dark ground. Its middle is the centre.
    if (runs.size() == 1 && runs.front().width() > params_.max_line_run_px) {
      r.source = LaneSource::kSurface;
      r.found = true;
      r.centre_x = runs.front().centre();
      r.lateral_error = clampErr((r.centre_x - half) / half);
      return r;
    }

    // Pick the two heaviest runs that are far enough apart to be the two
    // sides of a lane. Sorting by mass, not width, keeps a long solid line
    // ahead of a short bright speck.
    std::vector<detail::Run> sorted = runs;
    std::sort(sorted.begin(), sorted.end(),
              [](const detail::Run& a, const detail::Run& b) { return a.mass > b.mass; });

    const double min_sep = params_.min_lane_frac * w;
    for (std::size_t i = 0; i < sorted.size(); ++i) {
      for (std::size_t j = i + 1; j < sorted.size(); ++j) {
        const double a = sorted[i].centre();
        const double b = sorted[j].centre();
        if (std::fabs(a - b) >= min_sep) {
          r.left_x = std::min(a, b);
          r.right_x = std::max(a, b);
          r.centre_x = 0.5 * (r.left_x + r.right_x);
          r.lane_width = r.right_x - r.left_x;
          r.source = LaneSource::kTwoLines;
          r.found = true;
          r.lateral_error = clampErr((r.centre_x - half) / half);
          return r;
        }
      }
    }

    // Only one marking is usable. Which side of the car is it on? With a
    // remembered width and a remembered position we can tell; without them
    // we genuinely cannot, and guessing would steer the car off the road.
    const double x = sorted.front().centre();

    if (lane_width_ > 0.0 && last_centre_x_ >= 0.0) {
      const bool is_left = x < last_centre_x_;
      if (is_left) {
        r.left_x = x;
        r.centre_x = x + 0.5 * lane_width_;
      } else {
        r.right_x = x;
        r.centre_x = x - 0.5 * lane_width_;
      }
      r.lane_width = lane_width_;
      r.source = LaneSource::kOneLine;
      r.found = true;
      r.lateral_error = clampErr((r.centre_x - half) / half);
      return r;
    }

    r.source = LaneSource::kAmbiguous;
    r.centre_x = x;
    return r;
  }

  void remember(const LaneResult& r) {
    if (r.source == LaneSource::kTwoLines) {
      lane_width_ = r.lane_width;
      last_left_x_ = r.left_x;
      last_right_x_ = r.right_x;
      last_centre_x_ = r.centre_x;
    } else if (r.found && r.centre_x >= 0.0) {
      // Let the centre drift with the one-line and surface cases, but never
      // overwrite the width from them — it was not measured on this frame.
      last_centre_x_ = r.centre_x;
    }
  }

  static double clampErr(double v) { return std::clamp(v, -1.0, 1.0); }

  LaneParams params_{};
  cv::Mat mask_;
  cv::Mat bev_;

  double lane_width_ = -1.0;
  double last_left_x_ = -1.0;
  double last_right_x_ = -1.0;
  double last_centre_x_ = -1.0;
};

}  // namespace vision

#endif  // VISION_LANE_FINDER_HPP
