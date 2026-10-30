// lane_finder.hpp
// finds the road markings in a camera frame and says how far off centre we are
//
// the steps are:
//   1. threshold in HLS            -> white pixels where the markings are
//   2. warp to a birds eye view    -> markings become roughly vertical stripes
//   3. count white pixels per column in a band ahead of the car
//   4. group neighbouring columns into runs, one run per marking
//   5. work out the lane centre from those runs
//
// step 4 is the one that matters. the obvious version is to average every
// white pixel in the band, but that breaks on a bend: when one marking leaves
// the frame the average slides onto the marking that is left, so the car is
// told it is centred right when it is about to drive off the road

#ifndef LANE_FINDER_HPP
#define LANE_FINDER_HPP

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace vision {

enum class LaneSource {
    NOTHING_FOUND,   // no bright pixels at all
    BOTH_MARKINGS,   // found both sides, the good case
    ONE_MARKING,     // one side plus the lane width we remembered
    ROAD_SURFACE,    // one wide blob, so the road itself not a marking
    CANT_TELL        // one side and no memory, so we refuse to guess
};

inline const char* lane_source_name(LaneSource source) {
    switch (source) {
        case LaneSource::NOTHING_FOUND: return "nothing";
        case LaneSource::BOTH_MARKINGS: return "both markings";
        case LaneSource::ONE_MARKING:   return "one marking";
        case LaneSource::ROAD_SURFACE:  return "road surface";
        case LaneSource::CANT_TELL:     return "cant tell";
    }
    return "?";
}

struct LaneSettings {
    // --- picking out the markings ---
    // road paint is bright and has almost no colour, so a high lightness floor
    // with a low saturation ceiling catches it and ignores coloured scenery
    int min_lightness = 170;
    int max_saturation = 90;
    int despeckle_passes = 2;

    // --- birds eye warp ---
    // the four corners of the road patch we care about, as fractions of the
    // frame. expect to tune horizon_row once you see the warped picture
    double horizon_row = 0.62;
    double bottom_row = 0.98;
    double half_width_at_horizon = 0.10;
    double half_width_at_bottom = 0.52;
    int warped_size_px = 320;

    // --- the band we look at ---
    // 0.0 is far away, 1.0 is at the bumper. looking a little ahead instead of
    // right at the bumper stops the steering wobbling
    double band_far_edge = 0.20;
    double band_near_edge = 0.55;

    // --- grouping columns into markings ---
    int min_white_per_column = 2;
    int min_marking_width_px = 3;
    int join_gap_px = 4;           // dashes closer than this count as one marking
    int max_marking_width_px = 70; // wider than this is road surface
    double min_lane_width_fraction = 0.20;
};

struct LaneResult {
    bool usable = false;          // false means dont steer on this frame
    double offset_from_centre = 0.0;  // -1 lane is left of us, +1 lane is right
    LaneSource source = LaneSource::NOTHING_FOUND;

    double lane_centre_x = -1.0;  // all in warped pixels
    double left_marking_x = -1.0;
    double right_marking_x = -1.0;
    double lane_width_px = -1.0;
};

// one stretch of neighbouring columns that all had white in them
struct MarkingRun {
    int start_column = 0;
    int end_column = 0;    // one past the last
    long white_pixels = 0;

    int width() const { return end_column - start_column; }
    double centre() const { return 0.5 * (start_column + end_column - 1); }
};

class LaneFinder {
public:
    LaneFinder() {}
    explicit LaneFinder(const LaneSettings& settings) : settings_(settings) {}

    LaneSettings& settings() { return settings_; }

    // forget the lane width we learned. call this when the car gets moved, or
    // after a long run of bad frames, so an old width cant mislead us
    void reset() {
        remembered_lane_width_ = -1.0;
        remembered_centre_x_ = -1.0;
    }

    const cv::Mat& warped_mask() const { return warped_mask_; }

    LaneResult process(const cv::Mat& camera_frame) {
        LaneResult result;
        if (camera_frame.empty()) return result;

        build_mask(camera_frame);
        warp_to_birds_eye();

        std::vector<MarkingRun> runs = find_marking_runs();
        result = work_out_centre(runs);
        remember(result);
        return result;
    }

    // the warped view with our decision drawn on it, for the debug topic
    cv::Mat debug_picture(const LaneResult& result) const {
        if (warped_mask_.empty()) return cv::Mat();

        cv::Mat picture;
        cv::cvtColor(warped_mask_, picture, cv::COLOR_GRAY2BGR);

        int height = picture.rows;
        int band_top = (int)(settings_.band_far_edge * height);
        int band_bottom = (int)(settings_.band_near_edge * height);

        cv::rectangle(picture, cv::Point(0, band_top),
                      cv::Point(picture.cols - 1, band_bottom),
                      cv::Scalar(60, 60, 60), 1);

        int middle = picture.cols / 2;
        cv::line(picture, cv::Point(middle, 0), cv::Point(middle, height - 1),
                 cv::Scalar(120, 120, 120), 1);

        if (result.left_marking_x >= 0)
            draw_vertical(picture, result.left_marking_x, band_top, band_bottom,
                          cv::Scalar(0, 180, 255));
        if (result.right_marking_x >= 0)
            draw_vertical(picture, result.right_marking_x, band_top, band_bottom,
                          cv::Scalar(0, 180, 255));
        if (result.usable && result.lane_centre_x >= 0)
            draw_vertical(picture, result.lane_centre_x, band_top, band_bottom,
                          cv::Scalar(0, 255, 0));

        cv::putText(picture,
                    cv::format("%s  offset %+.2f", lane_source_name(result.source),
                               result.offset_from_centre),
                    cv::Point(6, 18), cv::FONT_HERSHEY_SIMPLEX, 0.45,
                    result.usable ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 140, 255),
                    1, cv::LINE_AA);
        return picture;
    }

private:
    static void draw_vertical(cv::Mat& picture, double x, int top, int bottom,
                              cv::Scalar colour) {
        cv::line(picture, cv::Point((int)x, top), cv::Point((int)x, bottom), colour, 2);
    }

    void build_mask(const cv::Mat& camera_frame) {
        cv::Mat hls;
        cv::cvtColor(camera_frame, hls, cv::COLOR_BGR2HLS);

        // channels are hue, lightness, saturation in that order. hue is left
        // wide open because the markings are defined by being bright and
        // colourless, not by any particular colour
        cv::inRange(hls,
                    cv::Scalar(0, settings_.min_lightness, 0),
                    cv::Scalar(179, 255, settings_.max_saturation),
                    mask_);

        if (settings_.despeckle_passes > 0) {
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
            cv::morphologyEx(mask_, mask_, cv::MORPH_OPEN, kernel,
                             cv::Point(-1, -1), settings_.despeckle_passes);
        }
    }

    void warp_to_birds_eye() {
        double frame_width = mask_.cols;
        double frame_height = mask_.rows;
        double middle_x = 0.5 * frame_width;

        cv::Point2f road_corners[4] = {
            cv::Point2f((float)(middle_x - settings_.half_width_at_horizon * frame_width),
                        (float)(settings_.horizon_row * frame_height)),
            cv::Point2f((float)(middle_x + settings_.half_width_at_horizon * frame_width),
                        (float)(settings_.horizon_row * frame_height)),
            cv::Point2f((float)(middle_x + settings_.half_width_at_bottom * frame_width),
                        (float)(settings_.bottom_row * frame_height)),
            cv::Point2f((float)(middle_x - settings_.half_width_at_bottom * frame_width),
                        (float)(settings_.bottom_row * frame_height))
        };

        float side = (float)settings_.warped_size_px;
        cv::Point2f square_corners[4] = {
            cv::Point2f(0.0f, 0.0f),
            cv::Point2f(side - 1.0f, 0.0f),
            cv::Point2f(side - 1.0f, side - 1.0f),
            cv::Point2f(0.0f, side - 1.0f)
        };

        cv::Mat warp_matrix = cv::getPerspectiveTransform(road_corners, square_corners);
        cv::warpPerspective(mask_, warped_mask_, warp_matrix,
                            cv::Size(settings_.warped_size_px, settings_.warped_size_px),
                            cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
    }

    std::vector<MarkingRun> find_marking_runs() const {
        std::vector<MarkingRun> runs;
        if (warped_mask_.empty()) return runs;

        int height = warped_mask_.rows;
        int width = warped_mask_.cols;
        int band_top = (int)(settings_.band_far_edge * height);
        int band_bottom = (int)(settings_.band_near_edge * height);
        if (band_top < 0) band_top = 0;
        if (band_bottom > height) band_bottom = height;

        // how many white pixels each column has inside the band
        std::vector<int> white_per_column(width, 0);
        for (int row = band_top; row < band_bottom; row++) {
            const unsigned char* pixels = warped_mask_.ptr<unsigned char>(row);
            for (int column = 0; column < width; column++) {
                if (pixels[column] != 0) white_per_column[column]++;
            }
        }

        // walk left to right, open a run on the first white column and close it
        // after enough dark ones, so a dashed line stays a single run
        int run_start = -1;
        int dark_columns = 0;
        long run_pixels = 0;

        for (int column = 0; column < width; column++) {
            bool is_marking = white_per_column[column] >= settings_.min_white_per_column;

            if (is_marking) {
                if (run_start < 0) {
                    run_start = column;
                    run_pixels = 0;
                }
                dark_columns = 0;
                run_pixels += white_per_column[column];
            } else if (run_start >= 0) {
                dark_columns++;
                if (dark_columns > settings_.join_gap_px) {
                    int run_end = column - dark_columns + 1;
                    if (run_end - run_start >= settings_.min_marking_width_px) {
                        MarkingRun run;
                        run.start_column = run_start;
                        run.end_column = run_end;
                        run.white_pixels = run_pixels;
                        runs.push_back(run);
                    }
                    run_start = -1;
                    dark_columns = 0;
                    run_pixels = 0;
                }
            }
        }

        if (run_start >= 0) {
            int run_end = width - dark_columns;
            if (run_end - run_start >= settings_.min_marking_width_px) {
                MarkingRun run;
                run.start_column = run_start;
                run.end_column = run_end;
                run.white_pixels = run_pixels;
                runs.push_back(run);
            }
        }
        return runs;
    }

    LaneResult work_out_centre(const std::vector<MarkingRun>& runs) {
        LaneResult result;
        double warped_width = (double)settings_.warped_size_px;
        double half_warped = 0.5 * warped_width;

        if (runs.empty()) {
            result.source = LaneSource::NOTHING_FOUND;
            return result;
        }

        // one very wide blob is the road slab itself, not a painted line.
        // its middle is the centre
        if (runs.size() == 1 && runs[0].width() > settings_.max_marking_width_px) {
            result.source = LaneSource::ROAD_SURFACE;
            result.usable = true;
            result.lane_centre_x = runs[0].centre();
            result.offset_from_centre = clamp_offset((result.lane_centre_x - half_warped) / half_warped);
            return result;
        }

        // look for two runs far enough apart to be the two sides of a lane.
        // sort by pixel count so a long solid line beats a bright speck
        std::vector<MarkingRun> biggest_first = runs;
        std::sort(biggest_first.begin(), biggest_first.end(),
                  [](const MarkingRun& a, const MarkingRun& b) {
                      return a.white_pixels > b.white_pixels;
                  });

        double min_separation = settings_.min_lane_width_fraction * warped_width;

        for (size_t i = 0; i < biggest_first.size(); i++) {
            for (size_t j = i + 1; j < biggest_first.size(); j++) {
                double one_centre = biggest_first[i].centre();
                double other_centre = biggest_first[j].centre();

                if (fabs(one_centre - other_centre) >= min_separation) {
                    result.left_marking_x = std::min(one_centre, other_centre);
                    result.right_marking_x = std::max(one_centre, other_centre);
                    result.lane_centre_x = 0.5 * (result.left_marking_x + result.right_marking_x);
                    result.lane_width_px = result.right_marking_x - result.left_marking_x;
                    result.source = LaneSource::BOTH_MARKINGS;
                    result.usable = true;
                    result.offset_from_centre =
                        clamp_offset((result.lane_centre_x - half_warped) / half_warped);
                    return result;
                }
            }
        }

        // only one marking is usable. which side of the car is it on? if we
        // remember the lane width and roughly where the centre was, we can
        // work it out. if we dont, there is genuinely no way to tell, and
        // guessing would steer the car off the road
        double only_marking_x = biggest_first[0].centre();

        if (remembered_lane_width_ > 0.0 && remembered_centre_x_ >= 0.0) {
            bool marking_is_on_our_left = only_marking_x < remembered_centre_x_;

            if (marking_is_on_our_left) {
                result.left_marking_x = only_marking_x;
                result.lane_centre_x = only_marking_x + 0.5 * remembered_lane_width_;
            } else {
                result.right_marking_x = only_marking_x;
                result.lane_centre_x = only_marking_x - 0.5 * remembered_lane_width_;
            }

            result.lane_width_px = remembered_lane_width_;
            result.source = LaneSource::ONE_MARKING;
            result.usable = true;
            result.offset_from_centre =
                clamp_offset((result.lane_centre_x - half_warped) / half_warped);
            return result;
        }

        result.source = LaneSource::CANT_TELL;
        result.lane_centre_x = only_marking_x;
        return result;
    }

    void remember(const LaneResult& result) {
        if (result.source == LaneSource::BOTH_MARKINGS) {
            remembered_lane_width_ = result.lane_width_px;
            remembered_centre_x_ = result.lane_centre_x;
        } else if (result.usable && result.lane_centre_x >= 0.0) {
            // let the centre drift along, but never overwrite the width from
            // these cases since we didnt actually measure it this frame
            remembered_centre_x_ = result.lane_centre_x;
        }
    }

    static double clamp_offset(double offset) {
        if (offset < -1.0) return -1.0;
        if (offset >  1.0) return  1.0;
        return offset;
    }

    LaneSettings settings_;
    cv::Mat mask_;
    cv::Mat warped_mask_;

    double remembered_lane_width_ = -1.0;
    double remembered_centre_x_ = -1.0;
};

}  // namespace vision

#endif
