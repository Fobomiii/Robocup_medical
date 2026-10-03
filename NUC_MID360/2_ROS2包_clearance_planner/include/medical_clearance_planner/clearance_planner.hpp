#ifndef MEDICAL_CLEARANCE_PLANNER__CLEARANCE_PLANNER_HPP_
#define MEDICAL_CLEARANCE_PLANNER__CLEARANCE_PLANNER_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace medical_clearance_planner
{

class ClearancePlanner : public nav2_core::GlobalPlanner
{
public:
  struct Parameters
  {
    bool allow_unknown{false};
    double tolerance{0.10};
    double max_planning_time{1.5};
    double forward_speed{3.00};
    double reverse_speed{3.00};
    double lateral_speed{3.00};
    double max_planar_speed{3.00};
    double max_wheel_speed{3.00};
    double max_time_ratio{1.10};
    double min_time_slack{0.25};
    double costmap_weight{3.0};
    double preferred_clearance{0.65};
    double clearance_weight{12.0};
    double clearance_power{2.0};
    double density_radius{0.75};
    double density_weight{8.0};
    double density_normalization{0.18};
    double goal_exemption_radius{0.75};
    double start_exemption_radius{0.45};
    double simplification_cost_tolerance{1.03};
    double simplification_time_tolerance{1.01};
    double route_switch_risk_improvement{0.55};
    double route_switch_time_improvement{0.50};
    double route_switch_max_slowdown{0.00};
    double route_reuse_max_distance{0.50};
    bool blind_zone_enabled{true};
    std::vector<double> blind_zone_angles_deg{45.0, 135.0, -135.0, -45.0};
    double blind_zone_half_width_deg{7.0};
    double blind_zone_margin_deg{3.0};
    double blind_zone_min_segment_length{1.0};
    double blind_zone_min_lateral_offset{0.18};
    double blind_zone_lateral_offset{0.32};
    double blind_zone_max_detour_time_ratio{1.25};
  };

  ClearancePlanner() = default;
  ~ClearancePlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  Parameters readParameters() const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  rclcpp::Logger logger_{rclcpp::get_logger("medical_clearance_planner")};
  bool has_previous_route_{false};
  geometry_msgs::msg::PoseStamped previous_goal_;
  std::vector<std::pair<double, double>> previous_route_;
};

}  // namespace medical_clearance_planner

#endif  // MEDICAL_CLEARANCE_PLANNER__CLEARANCE_PLANNER_HPP_
