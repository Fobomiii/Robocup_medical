#include "medical_clearance_planner/parking_exemption_layer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"

namespace medical_clearance_planner
{

void ParkingExemptionLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("ParkingExemptionLayer lifecycle node expired");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("zone_frame", rclcpp::ParameterValue("map"));
  declareParameter("bed1_x", rclcpp::ParameterValue(5.4));
  declareParameter("bed1_y", rclcpp::ParameterValue(2.2));
  declareParameter("bed3_x", rclcpp::ParameterValue(5.4));
  declareParameter("bed3_y", rclcpp::ParameterValue(-2.2));
  declareParameter("bed_radius", rclcpp::ParameterValue(0.30));
  declareParameter("home_x", rclcpp::ParameterValue(0.0));
  declareParameter("home_y", rclcpp::ParameterValue(0.0));
  declareParameter("home_size_x", rclcpp::ParameterValue(1.0));
  declareParameter("home_size_y", rclcpp::ParameterValue(1.0));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".zone_frame", zone_frame_);
  node->get_parameter(name_ + ".bed1_x", geometry_.bed1_x);
  node->get_parameter(name_ + ".bed1_y", geometry_.bed1_y);
  node->get_parameter(name_ + ".bed3_x", geometry_.bed3_x);
  node->get_parameter(name_ + ".bed3_y", geometry_.bed3_y);
  node->get_parameter(name_ + ".bed_radius", geometry_.bed_radius);
  node->get_parameter(name_ + ".home_x", geometry_.home_x);
  node->get_parameter(name_ + ".home_y", geometry_.home_y);
  node->get_parameter(name_ + ".home_size_x", geometry_.home_size_x);
  node->get_parameter(name_ + ".home_size_y", geometry_.home_size_y);

  if (geometry_.bed_radius <= 0.0 || geometry_.home_size_x <= 0.0 ||
    geometry_.home_size_y <= 0.0)
  {
    throw std::invalid_argument("Parking exemption dimensions must be positive");
  }

  current_ = true;
  RCLCPP_INFO(
    node->get_logger(),
    "%s ready | frame=%s beds=(%.2f,%.2f),(%.2f,%.2f) radius=%.2f m "
    "home=(%.2f,%.2f) size=%.2fx%.2f m",
    name_.c_str(), zone_frame_.c_str(), geometry_.bed1_x, geometry_.bed1_y,
    geometry_.bed3_x, geometry_.bed3_y, geometry_.bed_radius,
    geometry_.home_x, geometry_.home_y, geometry_.home_size_x,
    geometry_.home_size_y);
}

bool ParkingExemptionLayer::updateFrameTransform()
{
  const std::string global_frame = layered_costmap_->getGlobalFrameID();
  if (global_frame == zone_frame_) {
    frame_transform_ = FrameTransform{};
    frame_transform_.valid = true;
    return true;
  }

  auto node = node_.lock();
  if (!node) {
    frame_transform_.valid = false;
    return false;
  }

  try {
    const auto transform = tf_->lookupTransform(
      global_frame, zone_frame_, tf2::TimePointZero);
    const auto & translation = transform.transform.translation;
    const auto & rotation = transform.transform.rotation;
    const double sin_yaw = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y);
    const double cos_yaw =
      1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z);
    const double yaw = std::atan2(sin_yaw, cos_yaw);
    frame_transform_.translation_x = translation.x;
    frame_transform_.translation_y = translation.y;
    frame_transform_.cosine = std::cos(yaw);
    frame_transform_.sine = std::sin(yaw);
    frame_transform_.valid = true;
    return true;
  } catch (const tf2::TransformException & error) {
    frame_transform_.valid = false;
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *node->get_clock(), 2000,
      "%s cannot transform parking zones from %s to %s: %s",
      name_.c_str(), zone_frame_.c_str(), global_frame.c_str(), error.what());
    return false;
  }
}

void ParkingExemptionLayer::zoneToGlobal(
  double zone_x, double zone_y, double & global_x, double & global_y) const
{
  global_x = frame_transform_.cosine * zone_x - frame_transform_.sine * zone_y +
    frame_transform_.translation_x;
  global_y = frame_transform_.sine * zone_x + frame_transform_.cosine * zone_y +
    frame_transform_.translation_y;
}

void ParkingExemptionLayer::globalToZone(
  double global_x, double global_y, double & zone_x, double & zone_y) const
{
  const double translated_x = global_x - frame_transform_.translation_x;
  const double translated_y = global_y - frame_transform_.translation_y;
  zone_x = frame_transform_.cosine * translated_x + frame_transform_.sine * translated_y;
  zone_y = -frame_transform_.sine * translated_x + frame_transform_.cosine * translated_y;
}

void ParkingExemptionLayer::updateBounds(
  double, double, double,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }
  if (!updateFrameTransform()) {
    current_ = false;
    return;
  }

  const double padding = layered_costmap_->getCostmap()->getResolution();
  const auto include_point =
    [min_x, min_y, max_x, max_y, padding](double x, double y) {
      *min_x = std::min(*min_x, x - padding);
      *min_y = std::min(*min_y, y - padding);
      *max_x = std::max(*max_x, x + padding);
      *max_y = std::max(*max_y, y + padding);
    };

  const double bed_centers[2][2] = {
    {geometry_.bed1_x, geometry_.bed1_y},
    {geometry_.bed3_x, geometry_.bed3_y}
  };
  for (const auto & bed : bed_centers) {
    double center_x = 0.0;
    double center_y = 0.0;
    zoneToGlobal(bed[0], bed[1], center_x, center_y);
    *min_x = std::min(*min_x, center_x - geometry_.bed_radius - padding);
    *min_y = std::min(*min_y, center_y - geometry_.bed_radius - padding);
    *max_x = std::max(*max_x, center_x + geometry_.bed_radius + padding);
    *max_y = std::max(*max_y, center_y + geometry_.bed_radius + padding);
  }

  const double half_x = geometry_.home_size_x * 0.5;
  const double half_y = geometry_.home_size_y * 0.5;
  const double home_corners[4][2] = {
    {geometry_.home_x - half_x, geometry_.home_y - half_y},
    {geometry_.home_x - half_x, geometry_.home_y + half_y},
    {geometry_.home_x + half_x, geometry_.home_y - half_y},
    {geometry_.home_x + half_x, geometry_.home_y + half_y}
  };
  for (const auto & corner : home_corners) {
    double corner_x = 0.0;
    double corner_y = 0.0;
    zoneToGlobal(corner[0], corner[1], corner_x, corner_y);
    include_point(corner_x, corner_y);
  }
  current_ = true;
}

void ParkingExemptionLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  if (!frame_transform_.valid && !updateFrameTransform()) {
    current_ = false;
    return;
  }

  const double resolution = master_grid.getResolution();

  for (int map_y = min_j; map_y < max_j; ++map_y) {
    for (int map_x = min_i; map_x < max_i; ++map_x) {
      double global_x = 0.0;
      double global_y = 0.0;
      master_grid.mapToWorld(map_x, map_y, global_x, global_y);
      double zone_x = 0.0;
      double zone_y = 0.0;
      globalToZone(global_x, global_y, zone_x, zone_y);
      if (!cellIntersectsParkingExemption(
          zone_x, zone_y, resolution, geometry_))
      {
        continue;
      }

      const unsigned char original_cost = master_grid.getCost(map_x, map_y);
      const unsigned char exempted_cost = clearParkingInflation(original_cost);
      if (exempted_cost != original_cost) {
        master_grid.setCost(map_x, map_y, exempted_cost);
      }
    }
  }
  current_ = true;
}

void ParkingExemptionLayer::reset()
{
  frame_transform_.valid = false;
  current_ = true;
}

bool ParkingExemptionLayer::isClearable()
{
  return false;
}

}  // namespace medical_clearance_planner

PLUGINLIB_EXPORT_CLASS(
  medical_clearance_planner::ParkingExemptionLayer,
  nav2_costmap_2d::Layer)
