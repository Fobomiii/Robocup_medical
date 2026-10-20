#include "medical_clearance_planner/parking_exemption_geometry.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_costmap_2d/cost_values.hpp"

namespace medical_clearance_planner
{

bool insideParkingExemption(
  double x, double y, const ParkingExemptionGeometry & geometry) noexcept
{
  const double bed_radius_squared = geometry.bed_radius * geometry.bed_radius;
  const double bed1_dx = x - geometry.bed1_x;
  const double bed1_dy = y - geometry.bed1_y;
  if (bed1_dx * bed1_dx + bed1_dy * bed1_dy <= bed_radius_squared) {
    return true;
  }

  const double bed3_dx = x - geometry.bed3_x;
  const double bed3_dy = y - geometry.bed3_y;
  if (bed3_dx * bed3_dx + bed3_dy * bed3_dy <= bed_radius_squared) {
    return true;
  }

  return std::abs(x - geometry.home_x) <= geometry.home_size_x * 0.5 &&
         std::abs(y - geometry.home_y) <= geometry.home_size_y * 0.5;
}

bool cellIntersectsParkingExemption(
  double x, double y, double resolution,
  const ParkingExemptionGeometry & geometry) noexcept
{
  const double half_cell = resolution * 0.5;
  const double bed_radius_squared = geometry.bed_radius * geometry.bed_radius;
  const double bed_centers[2][2] = {
    {geometry.bed1_x, geometry.bed1_y},
    {geometry.bed3_x, geometry.bed3_y}
  };
  for (const auto & bed : bed_centers) {
    const double dx = std::max(std::abs(x - bed[0]) - half_cell, 0.0);
    const double dy = std::max(std::abs(y - bed[1]) - half_cell, 0.0);
    if (dx * dx + dy * dy <= bed_radius_squared) {
      return true;
    }
  }

  return std::abs(x - geometry.home_x) <= geometry.home_size_x * 0.5 + half_cell &&
         std::abs(y - geometry.home_y) <= geometry.home_size_y * 0.5 + half_cell;
}

unsigned char clearParkingInflation(unsigned char cost) noexcept
{
  if (cost > nav2_costmap_2d::FREE_SPACE &&
    cost <= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
  {
    return nav2_costmap_2d::FREE_SPACE;
  }
  return cost;
}

}  // namespace medical_clearance_planner
