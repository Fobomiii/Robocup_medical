#include "medical_clearance_planner/corner_geometry.hpp"

#include <algorithm>
#include <cmath>

namespace medical_clearance_planner
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1.0e-9;

double length(double x, double y)
{
  return std::hypot(x, y);
}

double crossProduct(double ax, double ay, double bx, double by)
{
  return ax * by - ay * bx;
}

}  // namespace

std::optional<CircularFillet> makeCircularFillet(
  const Point2D & previous,
  const Point2D & corner,
  const Point2D & next,
  double requested_radius,
  double minimum_radius,
  double maximum_tangent_fraction,
  double minimum_turn_angle,
  double maximum_turn_angle)
{
  const double incoming_x = corner.x - previous.x;
  const double incoming_y = corner.y - previous.y;
  const double outgoing_x = next.x - corner.x;
  const double outgoing_y = next.y - corner.y;
  const double incoming_length = length(incoming_x, incoming_y);
  const double outgoing_length = length(outgoing_x, outgoing_y);
  if (incoming_length <= kEpsilon || outgoing_length <= kEpsilon) {
    return std::nullopt;
  }

  const double in_x = incoming_x / incoming_length;
  const double in_y = incoming_y / incoming_length;
  const double out_x = outgoing_x / outgoing_length;
  const double out_y = outgoing_y / outgoing_length;
  const double dot = std::clamp(in_x * out_x + in_y * out_y, -1.0, 1.0);
  const double turn_angle = std::acos(dot);
  const double cross = in_x * out_y - in_y * out_x;
  if (turn_angle < minimum_turn_angle || turn_angle > maximum_turn_angle ||
    std::abs(cross) <= kEpsilon)
  {
    return std::nullopt;
  }

  const double tangent_scale = std::tan(0.5 * turn_angle);
  if (!std::isfinite(tangent_scale) || tangent_scale <= kEpsilon) {
    return std::nullopt;
  }
  const double tangent_limit = std::clamp(maximum_tangent_fraction, 0.0, 0.49) *
    std::min(incoming_length, outgoing_length);
  const double available_radius = tangent_limit / tangent_scale;
  const double radius = std::min(requested_radius, available_radius);
  if (!std::isfinite(radius) || radius <= kEpsilon ||
    radius + kEpsilon < minimum_radius)
  {
    return std::nullopt;
  }

  const double tangent_distance = radius * tangent_scale;
  CircularFillet fillet;
  fillet.corner = corner;
  fillet.entry = {
    corner.x - in_x * tangent_distance,
    corner.y - in_y * tangent_distance};
  fillet.exit = {
    corner.x + out_x * tangent_distance,
    corner.y + out_y * tangent_distance};
  const double turn_sign = cross > 0.0 ? 1.0 : -1.0;
  fillet.center = {
    fillet.entry.x - turn_sign * in_y * radius,
    fillet.entry.y + turn_sign * in_x * radius};
  fillet.radius = radius;
  fillet.start_angle = std::atan2(
    fillet.entry.y - fillet.center.y,
    fillet.entry.x - fillet.center.x);
  const double exit_angle = std::atan2(
    fillet.exit.y - fillet.center.y,
    fillet.exit.x - fillet.center.x);
  fillet.sweep_angle = std::atan2(
    std::sin(exit_angle - fillet.start_angle),
    std::cos(exit_angle - fillet.start_angle));
  if (turn_sign > 0.0 && fillet.sweep_angle < 0.0) {
    fillet.sweep_angle += 2.0 * kPi;
  } else if (turn_sign < 0.0 && fillet.sweep_angle > 0.0) {
    fillet.sweep_angle -= 2.0 * kPi;
  }
  if (std::abs(fillet.sweep_angle) > maximum_turn_angle + 1.0e-6) {
    return std::nullopt;
  }
  return fillet;
}

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
  double maximum_turn_angle)
{
  const double incoming_dx = incoming_end.x - previous.x;
  const double incoming_dy = incoming_end.y - previous.y;
  const double outgoing_dx = next.x - outgoing_start.x;
  const double outgoing_dy = next.y - outgoing_start.y;
  const double incoming_length = length(incoming_dx, incoming_dy);
  const double outgoing_length = length(outgoing_dx, outgoing_dy);
  if (incoming_length <= kEpsilon || outgoing_length <= kEpsilon) {
    return std::nullopt;
  }
  const double incoming_x = incoming_dx / incoming_length;
  const double incoming_y = incoming_dy / incoming_length;
  const double outgoing_x = outgoing_dx / outgoing_length;
  const double outgoing_y = outgoing_dy / outgoing_length;
  const double denominator = crossProduct(
    incoming_x, incoming_y, outgoing_x, outgoing_y);
  if (std::abs(denominator) <= 1.0e-6) {
    return std::nullopt;
  }

  const double offset_x = outgoing_start.x - incoming_end.x;
  const double offset_y = outgoing_start.y - incoming_end.y;
  const double incoming_extension = crossProduct(
    offset_x, offset_y, outgoing_x, outgoing_y) / denominator;
  const Point2D corner{
    incoming_end.x + incoming_x * incoming_extension,
    incoming_end.y + incoming_y * incoming_extension};
  const double outgoing_extension =
    (outgoing_start.x - corner.x) * outgoing_x +
    (outgoing_start.y - corner.y) * outgoing_y;
  if (incoming_extension < -kEpsilon || outgoing_extension < -kEpsilon ||
    incoming_extension + outgoing_extension > maximum_bridge_length + kEpsilon)
  {
    return std::nullopt;
  }
  return makeCircularFillet(
    previous, corner, next,
    requested_radius, minimum_radius, maximum_tangent_fraction,
    minimum_turn_angle, maximum_turn_angle);
}

std::vector<Point2D> sampleCircularFillet(
  const CircularFillet & fillet, double maximum_spacing)
{
  if (fillet.radius <= 0.0 || maximum_spacing <= 0.0 ||
    std::abs(fillet.sweep_angle) <= kEpsilon)
  {
    return {};
  }
  const double arc_length = fillet.radius * std::abs(fillet.sweep_angle);
  const int segments = std::max(
    1, static_cast<int>(std::ceil(arc_length / maximum_spacing)));
  std::vector<Point2D> points;
  points.reserve(static_cast<std::size_t>(segments) + 1U);
  for (int index = 0; index <= segments; ++index) {
    const double ratio = static_cast<double>(index) / segments;
    const double angle = fillet.start_angle + fillet.sweep_angle * ratio;
    points.push_back({
      fillet.center.x + fillet.radius * std::cos(angle),
      fillet.center.y + fillet.radius * std::sin(angle)});
  }
  points.front() = fillet.entry;
  points.back() = fillet.exit;
  return points;
}

}  // namespace medical_clearance_planner
