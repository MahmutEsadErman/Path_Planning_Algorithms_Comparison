/**
 * @file flight_report.cpp
 * @brief Comprehensive flight report generation with all performance metrics
 * 
 * Metrics computed:
 *   1. Cross-Track Error (CTE): perpendicular distance to nearest path segment
 *   2. Endpoint Error: distance to final target
 *   3. Along-Track Completion: percentage of path completed
 *   4. Heading Error: difference between drone yaw and ideal path yaw
 *   5. Path Length: reference path length and actual traversed length
 *   6. Feature Matching Quality: matches/frame, success rate, path loss count
 *   7. Computational Performance: timing per pipeline stage
 */

#include "gps_denied_nav/follow_path_node.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>

namespace {

/**
 * @brief Compute perpendicular distance from point P to line segment AB (2D)
 */
double pointToSegmentDistance(double px, double py,
                              double ax, double ay,
                              double bx, double by)
{
    double abx = bx - ax;
    double aby = by - ay;
    double apx = px - ax;
    double apy = py - ay;

    double ab_len_sq = abx * abx + aby * aby;
    if (ab_len_sq < 1e-12) {
        // A and B are the same point
        return std::sqrt(apx * apx + apy * apy);
    }

    // Project P onto AB, clamped to [0,1]
    double t = (apx * abx + apy * aby) / ab_len_sq;
    t = std::clamp(t, 0.0, 1.0);

    // Closest point on segment
    double cx = ax + t * abx;
    double cy = ay + t * aby;

    double dx = px - cx;
    double dy = py - cy;
    return std::sqrt(dx * dx + dy * dy);
}

double normalizeAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

namespace gps_denied_nav {

void FollowPathNode::generateFlightReport()
{
    if (metrics_data_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No metrics data to generate report");
        return;
    }

    // ============================================================
    //  Build reference path waypoints (forward or return)
    // ============================================================
    // Forward: path_data_[i].target_pose.position
    // Return:  path_history_[i].second.position (reversed)

    struct Point2D { double x, y; };
    std::vector<Point2D> ref_path;

    // Always start from starting_position_
    ref_path.push_back({starting_position_.x, starting_position_.y});

    for (size_t i = 0; i < path_data_.size(); ++i) {
        ref_path.push_back({path_data_[i].target_pose.position.x,
                            path_data_[i].target_pose.position.y});
    }

    // If returning, also add the return path (path_history reversed)
    std::vector<Point2D> return_ref_path;
    if (!path_history_.empty()) {
        for (int i = static_cast<int>(path_history_.size()) - 1; i >= 0; --i) {
            return_ref_path.push_back({path_history_[i].second.position.x,
                                       path_history_[i].second.position.y});
        }
    }

    // ============================================================
    //  1. Reference Path Length
    // ============================================================
    double ref_path_length = 0.0;
    for (size_t i = 1; i < ref_path.size(); ++i) {
        double dx = ref_path[i].x - ref_path[i-1].x;
        double dy = ref_path[i].y - ref_path[i-1].y;
        ref_path_length += std::sqrt(dx*dx + dy*dy);
    }

    double return_ref_path_length = 0.0;
    for (size_t i = 1; i < return_ref_path.size(); ++i) {
        double dx = return_ref_path[i].x - return_ref_path[i-1].x;
        double dy = return_ref_path[i].y - return_ref_path[i-1].y;
        return_ref_path_length += std::sqrt(dx*dx + dy*dy);
    }

    // ============================================================
    //  2. Actual Traversed Path Length (from gt_pose samples)
    // ============================================================
    double actual_path_length = 0.0;
    for (size_t i = 1; i < metrics_data_.size(); ++i) {
        double dx = metrics_data_[i].gt_x - metrics_data_[i-1].gt_x;
        double dy = metrics_data_[i].gt_y - metrics_data_[i-1].gt_y;
        actual_path_length += std::sqrt(dx*dx + dy*dy);
    }

    // ============================================================
    //  3. Per-frame CTE and Heading Error
    // ============================================================
    std::vector<double> cte_values;
    std::vector<double> heading_errors;
    int valid_match_frames = 0;
    double total_matches = 0.0;

    for (const auto& m : metrics_data_) {
        // --- CTE ---
        const auto& active_ref = m.is_returning ? return_ref_path : ref_path;

        if (active_ref.size() >= 2) {
            double min_dist = std::numeric_limits<double>::max();
            for (size_t i = 0; i + 1 < active_ref.size(); ++i) {
                double d = pointToSegmentDistance(
                    m.gt_x, m.gt_y,
                    active_ref[i].x, active_ref[i].y,
                    active_ref[i+1].x, active_ref[i+1].y);
                min_dist = std::min(min_dist, d);
            }
            cte_values.push_back(min_dist);
        }

        // --- Heading Error ---
        // Ideal yaw = direction from current path point to next
        const auto& href = m.is_returning ? return_ref_path : ref_path;
        int idx = m.path_index;
        if (m.is_returning) {
            // During return, path_index maps to history in reverse
            idx = std::min(idx, static_cast<int>(href.size()) - 2);
        } else {
            idx = std::min(idx, static_cast<int>(href.size()) - 2);
        }
        if (idx >= 0 && idx + 1 < static_cast<int>(href.size())) {
            double ideal_yaw = std::atan2(
                href[idx+1].y - href[idx].y,
                href[idx+1].x - href[idx].x);
            double h_err = std::abs(normalizeAngle(m.drone_yaw - ideal_yaw));
            heading_errors.push_back(h_err);
        }

        // --- Match quality ---
        total_matches += m.good_match_count;
        if (m.good_match_count >= min_feature_count_) {
            valid_match_frames++;
        }
    }

    // ============================================================
    //  4. Aggregate CTE statistics
    // ============================================================
    double cte_mean = 0, cte_max = 0, cte_rmse = 0, cte_stddev = 0;
    if (!cte_values.empty()) {
        size_t n = cte_values.size();
        cte_mean = std::accumulate(cte_values.begin(), cte_values.end(), 0.0) / n;
        cte_max = *std::max_element(cte_values.begin(), cte_values.end());

        double sq_sum = 0;
        for (double v : cte_values) sq_sum += v * v;
        cte_rmse = std::sqrt(sq_sum / n);

        double var_sum = 0;
        for (double v : cte_values) var_sum += (v - cte_mean) * (v - cte_mean);
        cte_stddev = std::sqrt(var_sum / n);
    }

    // ============================================================
    //  5. Aggregate Heading Error statistics
    // ============================================================
    double he_mean = 0, he_max = 0, he_rmse = 0;
    if (!heading_errors.empty()) {
        size_t n = heading_errors.size();
        he_mean = std::accumulate(heading_errors.begin(), heading_errors.end(), 0.0) / n;
        he_max = *std::max_element(heading_errors.begin(), heading_errors.end());

        double sq_sum = 0;
        for (double v : heading_errors) sq_sum += v * v;
        he_rmse = std::sqrt(sq_sum / n);
    }

    // ============================================================
    //  6. Along-Track Completion
    // ============================================================
    int total_points = returning_ 
        ? static_cast<int>(path_history_.size()) 
        : static_cast<int>(path_data_.size());
    double completion_pct = (total_points > 0) 
        ? (static_cast<double>(path_index_) / total_points * 100.0) 
        : 0.0;

    // ============================================================
    //  7. Endpoint Error (2D)
    // ============================================================
    if (gt_pose_ && distance_to_endpoint_ < 0) {
        geometry_msgs::msg::Point target_point;
        if (returning_) {
            target_point = starting_position_;
        } else if (!path_data_.empty()) {
            target_point = path_data_.back().target_pose.position;
        }
        double dx = gt_pose_->position.x - target_point.x;
        double dy = gt_pose_->position.y - target_point.y;
        distance_to_endpoint_ = std::sqrt(dx*dx + dy*dy);
    }

    // ============================================================
    //  8. Timing statistics
    // ============================================================
    double avg_fe = 0, avg_fm = 0, avg_ye = 0, avg_yf = 0, avg_total = 0;
    for (const auto& m : metrics_data_) {
        avg_fe += m.timing.feature_extraction_ms;
        avg_fm += m.timing.feature_matching_ms;
        avg_ye += m.timing.yaw_estimation_ms;
        avg_yf += m.timing.yaw_filtering_ms;
        avg_total += m.timing.total_frame_ms;
    }
    size_t n_frames = metrics_data_.size();
    avg_fe /= n_frames;
    avg_fm /= n_frames;
    avg_ye /= n_frames;
    avg_yf /= n_frames;
    avg_total /= n_frames;

    // ============================================================
    //  9. Feature matching stats
    // ============================================================
    double avg_matches = total_matches / n_frames;
    double match_success_rate = (n_frames > 0)
        ? (static_cast<double>(valid_match_frames) / n_frames * 100.0)
        : 0.0;

    // ============================================================
    //  10. Total time
    // ============================================================
    auto total_time = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::high_resolution_clock::now() - path_following_time_).count();

    // ============================================================
    //  Write CSV file
    // ============================================================
    std::ofstream file(report_output_file_);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open report file: %s",
                     report_output_file_.c_str());
        return;
    }

    // Per-frame data header
    file << "frame,gt_x,gt_y,drone_yaw_rad,path_index,is_returning,"
         << "good_matches,cte_m,heading_error_rad,"
         << "feature_extraction_ms,feature_matching_ms,yaw_estimation_ms,"
         << "yaw_filtering_ms,total_frame_ms\n";

    // Per-frame data rows
    for (size_t i = 0; i < metrics_data_.size(); ++i) {
        const auto& m = metrics_data_[i];
        double cte = (i < cte_values.size()) ? cte_values[i] : 0.0;
        double he = (i < heading_errors.size()) ? heading_errors[i] : 0.0;

        file << (i + 1) << ","
             << std::fixed << std::setprecision(6)
             << m.gt_x << "," << m.gt_y << ","
             << std::setprecision(4)
             << m.drone_yaw << ","
             << m.path_index << ","
             << (m.is_returning ? 1 : 0) << ","
             << m.good_match_count << ","
             << std::setprecision(6)
             << cte << "," << he << ","
             << std::setprecision(4)
             << m.timing.feature_extraction_ms << ","
             << m.timing.feature_matching_ms << ","
             << m.timing.yaw_estimation_ms << ","
             << m.timing.yaw_filtering_ms << ","
             << m.timing.total_frame_ms << "\n";
    }

    // Summary section
    file << "\n# ============================================================\n";
    file << "# FLIGHT REPORT SUMMARY\n";
    file << "# ============================================================\n";
    file << "#\n";
    file << "# [PARAMETERS]\n";
    file << "# feature_detector," << feature_detector_ << "\n";
    file << "# path_file," << path_file_ << "\n";
    file << "# velocity," << vel_ << "\n";
    file << "# similarity_threshold," << similarity_threshold_ << "\n";
    file << "# min_feature_count," << min_feature_count_ << "\n";
    file << "# camera_pitch_angle," << camera_pitch_angle_ << "\n";
    file << "#\n";
    file << "# [PATH]\n";
    file << "# status," << (path_completed_ ? "COMPLETED" : (returning_ ? "RETURNING" : (lost_path_ ? "PATH_LOST" : "FORWARD"))) << "\n";
    file << "# path_index," << path_index_ << "/" << total_points << "\n";
    file << "# completion_pct," << std::fixed << std::setprecision(1) << completion_pct << "\n";
    file << "# reference_path_length_m," << std::setprecision(4) << ref_path_length << "\n";
    file << "# return_ref_path_length_m," << return_ref_path_length << "\n";
    file << "# actual_traversed_length_m," << actual_path_length << "\n";
    file << "# path_history_size," << path_history_.size() << "\n";
    file << "#\n";
    file << "# [CROSS-TRACK ERROR]\n";
    file << "# cte_mean_m," << std::setprecision(6) << cte_mean << "\n";
    file << "# cte_max_m," << cte_max << "\n";
    file << "# cte_rmse_m," << cte_rmse << "\n";
    file << "# cte_stddev_m," << cte_stddev << "\n";
    file << "#\n";
    file << "# [HEADING ERROR]\n";
    file << "# heading_error_mean_rad," << std::setprecision(4) << he_mean << "\n";
    file << "# heading_error_mean_deg," << (he_mean * 180.0 / M_PI) << "\n";
    file << "# heading_error_max_rad," << he_max << "\n";
    file << "# heading_error_max_deg," << (he_max * 180.0 / M_PI) << "\n";
    file << "# heading_error_rmse_rad," << he_rmse << "\n";
    file << "#\n";
    file << "# [ENDPOINT]\n";
    file << "# endpoint_error_m," << std::setprecision(4) << distance_to_endpoint_ << "\n";
    file << "#\n";
    file << "# [FEATURE MATCHING]\n";
    file << "# avg_matches_per_frame," << std::setprecision(1) << avg_matches << "\n";
    file << "# match_success_rate_pct," << match_success_rate << "\n";
    file << "# path_loss_count," << path_loss_count_ << "\n";
    file << "#\n";
    file << "# [PERFORMANCE]\n";
    file << "# total_time_s," << total_time << "\n";
    file << "# total_frames," << n_frames << "\n";
    file << "# avg_fps," << std::setprecision(2) << avg_fps_ << "\n";
    file << "# avg_feature_extraction_ms," << std::setprecision(4) << avg_fe << "\n";
    file << "# avg_feature_matching_ms," << avg_fm << "\n";
    file << "# avg_yaw_estimation_ms," << avg_ye << "\n";
    file << "# avg_yaw_filtering_ms," << avg_yf << "\n";
    file << "# avg_total_frame_ms," << avg_total << "\n";
    file << "# ============================================================\n";

    file.close();

    // ============================================================
    //  Console output
    // ============================================================
    RCLCPP_INFO(this->get_logger(), "\n"
        "============================================================\n"
        "                      FLIGHT REPORT                         \n"
        "============================================================");

    RCLCPP_INFO(this->get_logger(), "[PARAMETERS]");
    RCLCPP_INFO(this->get_logger(), "  Feature Detector:    %s", feature_detector_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Path File:           %s", path_file_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Velocity:            %.2f m/s", vel_);
    RCLCPP_INFO(this->get_logger(), "  Similarity Thresh:   %d", similarity_threshold_);
    RCLCPP_INFO(this->get_logger(), "  Min Feature Count:   %d", min_feature_count_);

    RCLCPP_INFO(this->get_logger(), "------------------------------------------------------------");
    RCLCPP_INFO(this->get_logger(), "[PATH]");
    RCLCPP_INFO(this->get_logger(), "  Status:              %s",
        path_completed_ ? "COMPLETED" : (returning_ ? "RETURNING" : (lost_path_ ? "PATH LOST" : "FORWARD")));
    RCLCPP_INFO(this->get_logger(), "  Progress:            %d / %d (%.1f%%)",
        path_index_, total_points, completion_pct);
    RCLCPP_INFO(this->get_logger(), "  Ref Path Length:     %.4f m", ref_path_length);
    if (return_ref_path_length > 0) {
        RCLCPP_INFO(this->get_logger(), "  Return Path Length:  %.4f m", return_ref_path_length);
    }
    RCLCPP_INFO(this->get_logger(), "  Actual Traversed:    %.4f m", actual_path_length);

    RCLCPP_INFO(this->get_logger(), "------------------------------------------------------------");
    RCLCPP_INFO(this->get_logger(), "[CROSS-TRACK ERROR]");
    RCLCPP_INFO(this->get_logger(), "  Mean CTE:            %.6f m", cte_mean);
    RCLCPP_INFO(this->get_logger(), "  Max CTE:             %.6f m", cte_max);
    RCLCPP_INFO(this->get_logger(), "  RMSE CTE:            %.6f m", cte_rmse);
    RCLCPP_INFO(this->get_logger(), "  StdDev CTE:          %.6f m", cte_stddev);

    RCLCPP_INFO(this->get_logger(), "------------------------------------------------------------");
    RCLCPP_INFO(this->get_logger(), "[HEADING ERROR]");
    RCLCPP_INFO(this->get_logger(), "  Mean:                %.4f rad (%.2f deg)", he_mean, he_mean * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "  Max:                 %.4f rad (%.2f deg)", he_max, he_max * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "  RMSE:                %.4f rad (%.2f deg)", he_rmse, he_rmse * 180.0 / M_PI);

    RCLCPP_INFO(this->get_logger(), "------------------------------------------------------------");
    RCLCPP_INFO(this->get_logger(), "[ENDPOINT]");
    RCLCPP_INFO(this->get_logger(), "  Endpoint Error:      %.4f m", distance_to_endpoint_);

    RCLCPP_INFO(this->get_logger(), "------------------------------------------------------------");
    RCLCPP_INFO(this->get_logger(), "[FEATURE MATCHING]");
    RCLCPP_INFO(this->get_logger(), "  Avg Matches/Frame:   %.1f", avg_matches);
    RCLCPP_INFO(this->get_logger(), "  Match Success Rate:  %.1f%%", match_success_rate);
    RCLCPP_INFO(this->get_logger(), "  Path Loss Count:     %d", path_loss_count_);

    RCLCPP_INFO(this->get_logger(), "------------------------------------------------------------");
    RCLCPP_INFO(this->get_logger(), "[COMPUTATIONAL PERFORMANCE]");
    RCLCPP_INFO(this->get_logger(), "  Total Time:          %ld s", total_time);
    RCLCPP_INFO(this->get_logger(), "  Frames Processed:    %zu", n_frames);
    RCLCPP_INFO(this->get_logger(), "  Average FPS:         %.2f", avg_fps_);
    RCLCPP_INFO(this->get_logger(), "  %-24s %10s", "Operation", "Avg (ms)");
    RCLCPP_INFO(this->get_logger(), "  %-24s %10.4f", "Feature Extraction", avg_fe);
    RCLCPP_INFO(this->get_logger(), "  %-24s %10.4f", "Feature Matching", avg_fm);
    RCLCPP_INFO(this->get_logger(), "  %-24s %10.4f", "Yaw Estimation", avg_ye);
    RCLCPP_INFO(this->get_logger(), "  %-24s %10.4f", "Yaw Filtering", avg_yf);
    RCLCPP_INFO(this->get_logger(), "  %-24s %10.4f", "TOTAL", avg_total);

    RCLCPP_INFO(this->get_logger(),
        "============================================================\n"
        "  Report saved to: %s\n"
        "============================================================",
        report_output_file_.c_str());
}

}  // namespace gps_denied_nav
