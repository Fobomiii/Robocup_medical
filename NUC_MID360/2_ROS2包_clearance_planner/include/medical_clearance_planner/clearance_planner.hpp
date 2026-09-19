#ifndef MEDICAL_CLEARANCE_PLANNER__CLEARANCE_PLANNER_HPP_
#define MEDICAL_CLEARANCE_PLANNER__CLEARANCE_PLANNER_HPP_

#include <memory>
#include <string>
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
};

}  // namespace medical_clearance_planner

#endif  // MEDICAL_CLEARANCE_PLANNER__CLEARANCE_PLANNER_HPP_
