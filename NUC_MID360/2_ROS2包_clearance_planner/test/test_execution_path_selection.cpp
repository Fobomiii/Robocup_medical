#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "medical_clearance_planner/execution_path_selection.hpp"

using namespace medical_clearance_planner;

void check(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

int main()
{
  const ExecutionPathBlindParams params;
  const ExecutionPathPoints raw{{0.0, 0.0}, {1.2, 0.0}, {1.2, 1.2}};
  const ExecutionPathPoints diagonal{{0.0, 0.0}, {1.2, 1.2}};
  const ExecutionPathPoints smooth_safe{{0.0, 0.0}, {1.2, 0.05}, {1.2, 1.2}};

  auto result = selectExecutionPath(raw, &diagonal, 0.0, params);
  check(result.source == ExecutionPathSource::Raw, "unsafe smoothing must select raw object");
  check(result.raw_overlap_m == 0.0, "raw dogleg should avoid X shadows");
  check(result.smoothed_overlap_m >= 0.60, "unsafe smoothed overlap must be detected");

  result = selectExecutionPath(raw, &smooth_safe, 0.0, params);
  check(result.source == ExecutionPathSource::Smoothed, "safe smoothing must remain enabled");

  result = selectExecutionPath(diagonal, &diagonal, 0.0, params);
  check(result.source == ExecutionPathSource::Raw, "both unsafe must retain raw fallback");
  check(result.raw_overlap_m >= params.min_overlap_m, "raw fallback still needs speed protection");

  result = selectExecutionPath(diagonal, &raw, 0.0, params);
  check(result.source == ExecutionPathSource::Smoothed, "safe candidate may improve unsafe raw path");

  result = selectExecutionPath(raw, nullptr, 0.0, params);
  check(result.source == ExecutionPathSource::Raw, "smoother failure must retain raw path");

  const ExecutionPathPoints horizontal{{0.0, 0.0}, {3.0, 0.0}};
  result = selectExecutionPath(horizontal, &horizontal, kBlindZonePi / 4.0, params);
  check(result.source == ExecutionPathSource::Raw, "validation must rotate with current yaw");
  check(result.smoothed_overlap_m == 2.0, "whole-route overlap uses runtime lookahead cap");

  const ExecutionPathPoints short_crossing{{0.0, 0.0}, {0.2, 0.2}};
  result = selectExecutionPath(short_crossing, &short_crossing, 0.0, params);
  check(result.source == ExecutionPathSource::Smoothed, "short crossing must not be over-rejected");

  ExecutionPathPoints small_segments;
  for (int index = 0; index <= 12; ++index) {
    small_segments.emplace_back(0.1 * index, 0.1 * index);
  }
  result = selectExecutionPath(raw, &small_segments, 0.0, params);
  check(result.source == ExecutionPathSource::Raw, "short segments must accumulate blind overlap");
  check(result.smoothed_overlap_m >= 0.60, "subdivision must not hide blind overlap");

  result = selectExecutionPath(raw, &smooth_safe, std::nullopt, params);
  check(result.source == ExecutionPathSource::Raw, "missing fresh yaw must not endorse smoothing");

  result = selectExecutionPath(raw, &smooth_safe, std::numeric_limits<double>::quiet_NaN(), params);
  check(result.source == ExecutionPathSource::Raw, "invalid yaw must not endorse smoothing");

  auto disabled = params;
  disabled.enabled = false;
  result = selectExecutionPath(raw, &diagonal, std::nullopt, disabled);
  check(result.source == ExecutionPathSource::Smoothed, "explicit disable should preserve smoothing");

  const ExecutionPathPoints wrong_goal{{0.0, 0.0}, {9.0, 9.0}};
  result = selectExecutionPath(raw, &wrong_goal, 0.0, params);
  check(result.source == ExecutionPathSource::Raw, "other goal's candidate must not be selected");

  const ExecutionPathPoints invalid{{0.0, 0.0}, {std::numeric_limits<double>::quiet_NaN(), 1.0}};
  result = selectExecutionPath(raw, &invalid, 0.0, params);
  check(result.source == ExecutionPathSource::Raw, "invalid intermediate pose must reject candidate");
  result = selectExecutionPath(invalid, &smooth_safe, 0.0, params);
  check(result.source == ExecutionPathSource::None, "invalid raw route must fail selection");
  result = selectExecutionPath({}, &smooth_safe, 0.0, params);
  check(result.source == ExecutionPathSource::None, "empty raw route must fail selection");

  // Bed1/Bed3 and forward/reverse geometry use exactly the same acceptance.
  for (const double x_sign : {-1.0, 1.0}) {
    for (const double y_sign : {-1.0, 1.0}) {
      ExecutionPathPoints reflected_raw;
      ExecutionPathPoints reflected_diagonal;
      for (const auto & point : raw) {
        reflected_raw.emplace_back(x_sign * point.first, y_sign * point.second);
      }
      for (const auto & point : diagonal) {
        reflected_diagonal.emplace_back(x_sign * point.first, y_sign * point.second);
      }
      result = selectExecutionPath(reflected_raw, &reflected_diagonal, 0.0, params);
      check(result.source == ExecutionPathSource::Raw, "all four X-drive directions must be symmetric");
    }
  }
  std::cout << "Execution-path selection regression checks passed\n";
}
