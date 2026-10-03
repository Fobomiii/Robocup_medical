#include <cmath>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "medical_clearance_planner/execution_path_selection.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

namespace medical_clearance_planner
{

class SelectExecutionPath : public BT::SyncActionNode
{
public:
  SelectExecutionPath(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_ = config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    node_->get_parameter("robot_base_frame", base_frame_);
    params_.enabled = parameter("blind_zone_enabled", true);
    params_.angles_rad = blindZoneAnglesRadians(parameter(
        "blind_zone_angles_deg", std::vector<double>{45.0, 135.0, -135.0, -45.0}));
    params_.half_width_rad = (
      parameter("blind_zone_half_width_deg", 7.0) +
      parameter("blind_zone_heading_tolerance_deg", 3.0)) * kBlindZonePi / 180.0;
    params_.min_overlap_m = parameter("blind_zone_min_overlap_m", 0.60);
    params_.lookahead_m = parameter("blind_zone_lookahead_m", 2.0);
    pose_timeout_s_ = parameter("pose_timeout_s", 0.50);
    const auto path_topic = parameter(
      "path_topic", std::string("/medical_nav/execution_path"));
    const auto status_topic = parameter(
      "status_topic", std::string("/medical_nav/execution_path_status"));
    path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(path_topic, rclcpp::QoS(1));
    status_pub_ = node_->create_publisher<std_msgs::msg::String>(status_topic, rclcpp::QoS(10));
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("raw_path"),
      BT::InputPort<nav_msgs::msg::Path>("smoothed_path"),
      BT::OutputPort<nav_msgs::msg::Path>("path")};
  }

  BT::NodeStatus tick() override
  {
    nav_msgs::msg::Path raw;
    if (!getInput("raw_path", raw) || raw.header.frame_id.empty()) {
      return BT::NodeStatus::FAILURE;
    }
    nav_msgs::msg::Path smoothed;
    const bool has_smoothed = getInput("smoothed_path", smoothed) &&
      smoothed.header.frame_id == raw.header.frame_id;
    const auto raw_points = points(raw);
    const auto smoothed_points = points(smoothed);
    const auto yaw = currentYaw(raw.header.frame_id);
    const auto selection = selectExecutionPath(
      raw_points, has_smoothed ? &smoothed_points : nullptr, yaw, params_);
    if (selection.source == ExecutionPathSource::None) {
      return BT::NodeStatus::FAILURE;
    }

    auto selected = selection.source == ExecutionPathSource::Smoothed ? smoothed : raw;
    // One version and one object feed both FollowPath and its speed limiter.
    selected.header.stamp = node_->now();
    setOutput("path", selected);
    path_pub_->publish(selected);
    publishStatus(selected, selection, yaw);
    return BT::NodeStatus::SUCCESS;
  }

private:
  template<typename T>
  T parameter(const std::string & suffix, const T & default_value)
  {
    const auto name = "execution_path_selector." + suffix;
    nav2_util::declare_parameter_if_not_declared(
      node_, name, rclcpp::ParameterValue(default_value));
    return node_->get_parameter(name).get_value<T>();
  }

  static ExecutionPathPoints points(const nav_msgs::msg::Path & path)
  {
    ExecutionPathPoints result;
    result.reserve(path.poses.size());
    for (const auto & pose : path.poses) {
      if (!pose.header.frame_id.empty() && pose.header.frame_id != path.header.frame_id) {
        return {};
      }
      result.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }
    return result;
  }

  std::optional<double> currentYaw(const std::string & frame)
  {
    try {
      const auto transform = tf_->lookupTransform(frame, base_frame_, tf2::TimePointZero);
      const double age = (node_->now() - rclcpp::Time(
          transform.header.stamp, node_->get_clock()->get_clock_type())).seconds();
      if (std::abs(age) > pose_timeout_s_) {
        return std::nullopt;
      }
      const double yaw = tf2::getYaw(transform.transform.rotation);
      return std::isfinite(yaw) ? std::optional<double>(yaw) : std::nullopt;
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  void publishStatus(
    const nav_msgs::msg::Path & selected,
    const ExecutionPathSelection & selection, const std::optional<double> & yaw)
  {
    const auto json_number = [](double value) {
        return value < 0.0 ? std::string("null") : std::to_string(value);
      };
    std::ostringstream data;
    data << "{\"source\":\"" <<
      (selection.source == ExecutionPathSource::Smoothed ? "smoothed" : "raw") <<
      "\",\"reason\":\"" << selection.reason << "\",\"path_version\":\"" <<
      selected.header.stamp.sec << '.' << std::setw(9) << std::setfill('0') <<
      selected.header.stamp.nanosec << "\",\"raw_overlap_m\":" <<
      json_number(selection.raw_overlap_m) << ",\"smoothed_overlap_m\":" <<
      json_number(selection.smoothed_overlap_m) << ",\"yaw_rad\":" <<
      (yaw ? std::to_string(*yaw) : "null") << "}";
    std_msgs::msg::String message;
    message.data = data.str();
    status_pub_->publish(message);
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  ExecutionPathBlindParams params_;
  std::string base_frame_{"base_link"};
  double pose_timeout_s_{0.50};
};

}  // namespace medical_clearance_planner

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<medical_clearance_planner::SelectExecutionPath>("SelectExecutionPath");
}
