#ifndef MEDICAL_CLEARANCE_PLANNER__EXECUTION_PATH_SELECTION_HPP_
#define MEDICAL_CLEARANCE_PLANNER__EXECUTION_PATH_SELECTION_HPP_

#include <cmath>
#include <optional>
#include <vector>

#include "medical_clearance_planner/blind_zone_geometry.hpp"

namespace medical_clearance_planner
{

using ExecutionPathPoints = std::vector<std::pair<double, double>>;

struct ExecutionPathBlindParams
{
  bool enabled{true};
  std::vector<double> angles_rad{blindZoneAnglesRadians({45.0, 135.0, -135.0, -45.0})};
  double half_width_rad{10.0 * kBlindZonePi / 180.0};
  double min_overlap_m{0.60};
  double lookahead_m{2.0};
};

enum class ExecutionPathSource {None, Raw, Smoothed};

struct ExecutionPathSelection
{
  ExecutionPathSource source{ExecutionPathSource::None};
  const char * reason{"invalid_raw_path"};
  double raw_overlap_m{-1.0};
  double smoothed_overlap_m{-1.0};
};

inline bool validExecutionPath(const ExecutionPathPoints & points)
{
  if (points.empty()) {
    return false;
  }
  for (const auto & point : points) {
    if (!std::isfinite(point.first) || !std::isfinite(point.second)) {
      return false;
    }
  }
  return true;
}

inline bool sameExecutionPathEndpoints(
  const ExecutionPathPoints & first, const ExecutionPathPoints & second)
{
  const auto close = [](const auto & a, const auto & b) {
      return std::hypot(a.first - b.first, a.second - b.second) <= 1.0e-3;
    };
  return validExecutionPath(first) && validExecutionPath(second) &&
         close(first.front(), second.front()) && close(first.back(), second.back());
}

// The result selects the path object, not just the geometry used for a limit.
inline ExecutionPathSelection selectExecutionPath(
  const ExecutionPathPoints & raw,
  const ExecutionPathPoints * smoothed,
  const std::optional<double> & current_yaw,
  const ExecutionPathBlindParams & params)
{
  ExecutionPathSelection result;
  if (!validExecutionPath(raw)) {
    return result;
  }
  result.source = ExecutionPathSource::Raw;
  const bool candidate_valid = smoothed && sameExecutionPathEndpoints(raw, *smoothed);
  if (params.enabled && (!current_yaw || !std::isfinite(*current_yaw))) {
    result.reason = "raw_no_fresh_yaw";
    return result;
  }
  const auto overlap = [&](const ExecutionPathPoints & points) {
      return params.enabled ? longestBlindZoneOverlap(
        points, *current_yaw, params.angles_rad, params.half_width_rad,
        params.lookahead_m) : 0.0;
    };
  result.raw_overlap_m = overlap(raw);
  if (!candidate_valid) {
    result.reason = smoothed ? "raw_invalid_smoothed_path" : "raw_no_smoothing";
    return result;
  }
  result.smoothed_overlap_m = overlap(*smoothed);
  if (params.enabled && result.smoothed_overlap_m >= params.min_overlap_m) {
    result.reason = result.raw_overlap_m >= params.min_overlap_m ?
      "both_blind_limited" : "smoothed_blind_overlap";
    return result;
  }
  result.source = ExecutionPathSource::Smoothed;
  result.reason = "smoothed_validated";
  return result;
}

}  // namespace medical_clearance_planner

#endif  // MEDICAL_CLEARANCE_PLANNER__EXECUTION_PATH_SELECTION_HPP_
