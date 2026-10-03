#ifndef MEDICAL_CLEARANCE_PLANNER__BLIND_ZONE_GEOMETRY_HPP_
#define MEDICAL_CLEARANCE_PLANNER__BLIND_ZONE_GEOMETRY_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace medical_clearance_planner
{

constexpr double kBlindZonePi = 3.14159265358979323846;

inline double wrapBlindZoneAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline std::vector<double> blindZoneAnglesRadians(
  const std::vector<double> & angles_deg)
{
  std::vector<double> angles_rad;
  angles_rad.reserve(angles_deg.size());
  for (const double angle_deg : angles_deg) {
    angles_rad.push_back(angle_deg * kBlindZonePi / 180.0);
  }
  return angles_rad;
}

inline double blindZoneAngleError(
  double world_dx, double world_dy, double travel_yaw,
  const std::vector<double> & blind_angles_rad)
{
  if (blind_angles_rad.empty() || std::hypot(world_dx, world_dy) <= 1.0e-9) {
    return std::numeric_limits<double>::infinity();
  }

  const double body_direction = wrapBlindZoneAngle(
    std::atan2(world_dy, world_dx) - travel_yaw);
  double minimum_error = std::numeric_limits<double>::infinity();
  for (const double blind_angle : blind_angles_rad) {
    minimum_error = std::min(
      minimum_error,
      std::abs(wrapBlindZoneAngle(body_direction - blind_angle)));
  }
  return minimum_error;
}

// This is the whole-route equivalent of the runtime limiter's 2 m lookahead:
// it finds the worst continuous blind-aligned run that any lookahead window
// could encounter. Capping at lookahead keeps both decisions equivalent.
inline double longestBlindZoneOverlap(
  const std::vector<std::pair<double, double>> & points,
  double travel_yaw,
  const std::vector<double> & blind_angles_rad,
  double half_width_rad,
  double lookahead_distance_m)
{
  if (points.size() < 2 || blind_angles_rad.empty() ||
    half_width_rad <= 0.0 || lookahead_distance_m <= 0.0)
  {
    return 0.0;
  }

  double longest_overlap = 0.0;
  double current_overlap = 0.0;
  for (std::size_t index = 1; index < points.size(); ++index) {
    const double dx = points[index].first - points[index - 1].first;
    const double dy = points[index].second - points[index - 1].second;
    const double length = std::hypot(dx, dy);
    if (length <= 1.0e-9) {
      continue;
    }
    const double error = blindZoneAngleError(
      dx, dy, travel_yaw, blind_angles_rad);
    if (error <= half_width_rad) {
      current_overlap = std::min(
        lookahead_distance_m, current_overlap + length);
      longest_overlap = std::max(longest_overlap, current_overlap);
    } else {
      current_overlap = 0.0;
    }
  }
  return longest_overlap;
}

}  // namespace medical_clearance_planner

#endif  // MEDICAL_CLEARANCE_PLANNER__BLIND_ZONE_GEOMETRY_HPP_
