#ifndef MEDICAL_CLEARANCE_PLANNER__CORNER_GEOMETRY_HPP_
#define MEDICAL_CLEARANCE_PLANNER__CORNER_GEOMETRY_HPP_

#include <optional>
#include <vector>

namespace medical_clearance_planner
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct CircularFillet
{
  Point2D corner;
  Point2D entry;
  Point2D exit;
  Point2D center;
  double radius{0.0};
  double start_angle{0.0};
  double sweep_angle{0.0};
};

std::optional<CircularFillet> makeCircularFillet(
  const Point2D & previous,
  const Point2D & corner,
  const Point2D & next,
  double requested_radius,
  double minimum_radius,
  double maximum_tangent_fraction,
  double minimum_turn_angle,
  double maximum_turn_angle);

std::optional<CircularFillet> makeTransitionFillet(
  const Point2D & previous,
  const Point2D & incoming_end,
  const Point2D & outgoing_start,
  const Point2D & next,
  double maximum_bridge_length,
  double requested_radius,
  double minimum_radius,
  double maximum_tangent_fraction,
  double minimum_turn_angle,
  double maximum_turn_angle);

std::vector<Point2D> sampleCircularFillet(
  const CircularFillet & fillet, double maximum_spacing);

}  // namespace medical_clearance_planner

#endif  // MEDICAL_CLEARANCE_PLANNER__CORNER_GEOMETRY_HPP_
