#ifndef MEDICAL_CLEARANCE_PLANNER__PARKING_EXEMPTION_GEOMETRY_HPP_
#define MEDICAL_CLEARANCE_PLANNER__PARKING_EXEMPTION_GEOMETRY_HPP_

namespace medical_clearance_planner
{

struct ParkingExemptionGeometry
{
  double bed1_x{5.4};
  double bed1_y{2.2};
  double bed3_x{5.4};
  double bed3_y{-2.2};
  double bed_radius{0.30};
  double home_x{0.0};
  double home_y{0.0};
  double home_size_x{1.0};
  double home_size_y{1.0};
};

bool insideParkingExemption(
  double x, double y, const ParkingExemptionGeometry & geometry) noexcept;

bool cellIntersectsParkingExemption(
  double x, double y, double resolution,
  const ParkingExemptionGeometry & geometry) noexcept;

unsigned char clearParkingInflation(unsigned char cost) noexcept;

}  // namespace medical_clearance_planner

#endif  // MEDICAL_CLEARANCE_PLANNER__PARKING_EXEMPTION_GEOMETRY_HPP_
