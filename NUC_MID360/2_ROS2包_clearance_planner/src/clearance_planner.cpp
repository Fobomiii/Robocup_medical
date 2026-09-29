#include "medical_clearance_planner/clearance_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace medical_clearance_planner
{
namespace
{

constexpr double kSqrtTwo = 1.4142135623730951;
constexpr double kInfinity = std::numeric_limits<double>::infinity();

struct GridSnapshot
{
  unsigned int width{0};
  unsigned int height{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  std::vector<unsigned char> costs;

  std::size_t size() const
  {
    return static_cast<std::size_t>(width) * height;
  }

  bool contains(int x, int y) const
  {
    return x >= 0 && y >= 0 && x < static_cast<int>(width) &&
           y < static_cast<int>(height);
  }

  int index(int x, int y) const
  {
    return y * static_cast<int>(width) + x;
  }

  std::pair<int, int> coordinates(int index_value) const
  {
    return {index_value % static_cast<int>(width),
      index_value / static_cast<int>(width)};
  }

  bool worldToMap(double wx, double wy, int & mx, int & my) const
  {
    if (wx < origin_x || wy < origin_y || resolution <= 0.0) {
      return false;
    }
    mx = static_cast<int>(std::floor((wx - origin_x) / resolution));
    my = static_cast<int>(std::floor((wy - origin_y) / resolution));
    return contains(mx, my);
  }

  std::pair<double, double> mapToWorld(int mx, int my) const
  {
    return {
      origin_x + (static_cast<double>(mx) + 0.5) * resolution,
      origin_y + (static_cast<double>(my) + 0.5) * resolution};
  }
};

struct QueueEntry
{
  double priority;
  int index;

  bool operator>(const QueueEntry & other) const
  {
    return priority > other.priority;
  }
};

double clamp01(double value)
{
  return std::max(0.0, std::min(1.0, value));
}

double smoothStep(double value)
{
  const double x = clamp01(value);
  return x * x * (3.0 - 2.0 * x);
}

bool isObstacleSource(unsigned char cost, bool allow_unknown)
{
  return cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
         (!allow_unknown && cost == nav2_costmap_2d::NO_INFORMATION);
}

bool isBlocked(unsigned char cost, bool allow_unknown)
{
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return !allow_unknown;
  }
  return cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

std::vector<double> distanceField(const GridSnapshot & grid, bool allow_unknown)
{
  std::vector<double> distance(grid.size(), kInfinity);
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open;

  for (std::size_t i = 0; i < grid.size(); ++i) {
    if (isObstacleSource(grid.costs[i], allow_unknown)) {
      distance[i] = 0.0;
      open.push({0.0, static_cast<int>(i)});
    }
  }

  static constexpr int kDx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
  static constexpr int kDy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
  while (!open.empty()) {
    const QueueEntry current = open.top();
    open.pop();
    if (current.priority > distance[current.index]) {
      continue;
    }
    const auto [x, y] = grid.coordinates(current.index);
    for (int direction = 0; direction < 8; ++direction) {
      const int nx = x + kDx[direction];
      const int ny = y + kDy[direction];
      if (!grid.contains(nx, ny)) {
        continue;
      }
      const int neighbor = grid.index(nx, ny);
      const double step = (kDx[direction] != 0 && kDy[direction] != 0) ?
        kSqrtTwo * grid.resolution : grid.resolution;
      const double candidate = current.priority + step;
      if (candidate < distance[neighbor]) {
        distance[neighbor] = candidate;
        open.push({candidate, neighbor});
      }
    }
  }
  return distance;
}

std::vector<int> obstacleIntegralImage(const GridSnapshot & grid, bool allow_unknown)
{
  const unsigned int stride = grid.width + 1;
  std::vector<int> integral(
    static_cast<std::size_t>(grid.width + 1) * (grid.height + 1), 0);
  for (unsigned int y = 0; y < grid.height; ++y) {
    int row_sum = 0;
    for (unsigned int x = 0; x < grid.width; ++x) {
      row_sum += isObstacleSource(
        grid.costs[grid.index(static_cast<int>(x), static_cast<int>(y))], allow_unknown) ? 1 : 0;
      integral[static_cast<std::size_t>(y + 1) * stride + x + 1] =
        integral[static_cast<std::size_t>(y) * stride + x + 1] + row_sum;
    }
  }
  return integral;
}

double densityAt(
  const GridSnapshot & grid, const std::vector<int> & integral,
  int x, int y, int radius_cells)
{
  const int x0 = std::max(0, x - radius_cells);
  const int y0 = std::max(0, y - radius_cells);
  const int x1 = std::min(static_cast<int>(grid.width), x + radius_cells + 1);
  const int y1 = std::min(static_cast<int>(grid.height), y + radius_cells + 1);
  const int stride = static_cast<int>(grid.width) + 1;
  const int count =
    integral[y1 * stride + x1] - integral[y0 * stride + x1] -
    integral[y1 * stride + x0] + integral[y0 * stride + x0];
  const int area = std::max(1, (x1 - x0) * (y1 - y0));
  return static_cast<double>(count) / static_cast<double>(area);
}

double distanceBetween(double ax, double ay, double bx, double by)
{
  return std::hypot(ax - bx, ay - by);
}

std::vector<double> traversalRisks(
  const GridSnapshot & grid,
  const std::vector<double> & clearance,
  const std::vector<int> & density_integral,
  const ClearancePlanner::Parameters & params,
  double start_x, double start_y,
  double goal_x, double goal_y)
{
  std::vector<double> risk(grid.size(), 0.0);
  const int density_cells = std::max(
    1, static_cast<int>(std::ceil(params.density_radius / grid.resolution)));

  for (unsigned int y = 0; y < grid.height; ++y) {
    for (unsigned int x = 0; x < grid.width; ++x) {
      const int index_value = grid.index(static_cast<int>(x), static_cast<int>(y));
      if (isBlocked(grid.costs[index_value], params.allow_unknown)) {
        risk[index_value] = kInfinity;
        continue;
      }

      const auto [wx, wy] = grid.mapToWorld(static_cast<int>(x), static_cast<int>(y));
      double preference_scale = 1.0;
      if (params.goal_exemption_radius > 0.0) {
        preference_scale = std::min(
          preference_scale,
          smoothStep(distanceBetween(wx, wy, goal_x, goal_y) /
          params.goal_exemption_radius));
      }
      if (params.start_exemption_radius > 0.0) {
        preference_scale = std::min(
          preference_scale,
          smoothStep(distanceBetween(wx, wy, start_x, start_y) /
          params.start_exemption_radius));
      }

      const double clearance_ratio = params.preferred_clearance > 0.0 ?
        clamp01((params.preferred_clearance - clearance[index_value]) /
        params.preferred_clearance) : 0.0;
      const double clearance_penalty = params.clearance_weight *
        std::pow(clearance_ratio, params.clearance_power);

      const double raw_density = densityAt(
        grid, density_integral, static_cast<int>(x), static_cast<int>(y), density_cells);
      const double normalized_density = clamp01(
        raw_density / std::max(0.001, params.density_normalization));
      const double density_penalty = params.density_weight * normalized_density;

      const double costmap_ratio =
        std::min<double>(grid.costs[index_value], nav2_costmap_2d::MAX_NON_OBSTACLE) /
        static_cast<double>(nav2_costmap_2d::MAX_NON_OBSTACLE);
      const double costmap_penalty = params.costmap_weight * costmap_ratio;

      // Goal/start exemptions apply only to this plugin's extra preference
      // field. The original Nav2 costmap penalty stays active everywhere, so
      // a newly observed obstacle near a target is never made artificially
      // attractive by the precision-approach exemption.
      risk[index_value] = costmap_penalty + preference_scale *
        (clearance_penalty + density_penalty);
    }
  }
  return risk;
}

bool diagonalMoveIsClear(
  const GridSnapshot & grid, int x, int y, int nx, int ny,
  const std::vector<double> & risk)
{
  if (x == nx || y == ny) {
    return true;
  }
  return std::isfinite(risk[grid.index(nx, y)]) &&
         std::isfinite(risk[grid.index(x, ny)]);
}

int nearestTraversableGoal(
  const GridSnapshot & grid, int requested_x, int requested_y,
  const std::vector<double> & risk, double tolerance)
{
  const int requested = grid.index(requested_x, requested_y);
  if (std::isfinite(risk[requested])) {
    return requested;
  }
  const int radius = std::max(0, static_cast<int>(std::ceil(tolerance / grid.resolution)));
  int best = -1;
  double best_distance = kInfinity;
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      const int x = requested_x + dx;
      const int y = requested_y + dy;
      if (!grid.contains(x, y)) {
        continue;
      }
      const double distance = std::hypot(dx, dy) * grid.resolution;
      const int candidate = grid.index(x, y);
      if (distance <= tolerance && distance < best_distance &&
        std::isfinite(risk[candidate]))
      {
        best = candidate;
        best_distance = distance;
      }
    }
  }
  return best;
}

double moveTime(
  const GridSnapshot & grid, int x, int y, int nx, int ny,
  double travel_yaw,
  const ClearancePlanner::Parameters & params)
{
  const double world_x = static_cast<double>(nx - x) * grid.resolution;
  const double world_y = static_cast<double>(ny - y) * grid.resolution;
  const double cosine = std::cos(travel_yaw);
  const double sine = std::sin(travel_yaw);
  const double body_x = cosine * world_x + sine * world_y;
  const double body_y = -sine * world_x + cosine * world_y;
  const double x_speed = body_x >= 0.0 ? params.forward_speed : params.reverse_speed;
  const double x_time = std::abs(body_x) / std::max(0.01, x_speed);
  const double y_time = std::abs(body_y) / std::max(0.01, params.lateral_speed);
  const double wheel_time =
    (std::abs(body_x) + std::abs(body_y)) /
    std::max(0.01, params.max_wheel_speed);
  return std::max({x_time, y_time, wheel_time});
}

double poseYaw(const geometry_msgs::msg::Quaternion & orientation)
{
  return std::atan2(
    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
    1.0 - 2.0 * (
      orientation.y * orientation.y + orientation.z * orientation.z));
}

bool planningTimedOut(
  const std::chrono::steady_clock::time_point & started,
  double max_planning_time)
{
  return max_planning_time > 0.0 &&
         std::chrono::duration<double>(
    std::chrono::steady_clock::now() - started).count() > max_planning_time;
}

std::vector<double> fastestTimeField(
  const GridSnapshot & grid, int goal,
  const std::vector<double> & risk,
  double travel_yaw,
  const ClearancePlanner::Parameters & params,
  const std::chrono::steady_clock::time_point & started,
  std::size_t & expanded)
{
  std::vector<double> time_to_goal(grid.size(), kInfinity);
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open;
  time_to_goal[goal] = 0.0;
  open.push({0.0, goal});

  static constexpr int kDx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
  static constexpr int kDy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
  while (!open.empty()) {
    const QueueEntry current = open.top();
    open.pop();
    if (current.priority > time_to_goal[current.index]) {
      continue;
    }
    ++expanded;
    if ((expanded & 0xFFU) == 0U && planningTimedOut(started, params.max_planning_time)) {
      return {};
    }

    const auto [x, y] = grid.coordinates(current.index);
    for (int direction = 0; direction < 8; ++direction) {
      const int px = x + kDx[direction];
      const int py = y + kDy[direction];
      if (!grid.contains(px, py)) {
        continue;
      }
      const int predecessor = grid.index(px, py);
      if (!std::isfinite(risk[predecessor]) ||
        !diagonalMoveIsClear(grid, px, py, x, y, risk))
      {
        continue;
      }
      const double candidate = current.priority +
        moveTime(grid, px, py, x, y, travel_yaw, params);
      if (candidate < time_to_goal[predecessor]) {
        time_to_goal[predecessor] = candidate;
        open.push({candidate, predecessor});
      }
    }
  }
  return time_to_goal;
}

std::vector<int> reconstructFastestPath(
  const GridSnapshot & grid, int start, int goal,
  const std::vector<double> & risk,
  const std::vector<double> & time_to_goal,
  double travel_yaw,
  const ClearancePlanner::Parameters & params)
{
  if (!std::isfinite(time_to_goal[start])) {
    return {};
  }
  std::vector<int> path{start};
  int current = start;
  static constexpr int kDx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
  static constexpr int kDy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
  for (std::size_t step_count = 0; current != goal && step_count < grid.size(); ++step_count) {
    const auto [x, y] = grid.coordinates(current);
    int best = -1;
    double best_time = kInfinity;
    double best_risk = kInfinity;
    for (int direction = 0; direction < 8; ++direction) {
      const int nx = x + kDx[direction];
      const int ny = y + kDy[direction];
      if (!grid.contains(nx, ny)) {
        continue;
      }
      const int neighbor = grid.index(nx, ny);
      if (!std::isfinite(risk[neighbor]) || !std::isfinite(time_to_goal[neighbor]) ||
        !diagonalMoveIsClear(grid, x, y, nx, ny, risk))
      {
        continue;
      }
      const double candidate = moveTime(grid, x, y, nx, ny, travel_yaw, params) +
        time_to_goal[neighbor];
      if (candidate < best_time - 1.0e-9 ||
        (std::abs(candidate - best_time) <= 1.0e-9 && risk[neighbor] < best_risk))
      {
        best = neighbor;
        best_time = candidate;
        best_risk = risk[neighbor];
      }
    }
    if (best < 0 || best_time > time_to_goal[current] + 1.0e-6) {
      return {};
    }
    current = best;
    path.push_back(current);
  }
  return current == goal ? path : std::vector<int>{};
}

struct SafeLabel
{
  int cell;
  int parent;
  double elapsed;
  double risk;
  bool active;
};

struct SafeQueueEntry
{
  double risk;
  double elapsed;
  int label;

  bool operator>(const SafeQueueEntry & other) const
  {
    if (risk != other.risk) {
      return risk > other.risk;
    }
    return elapsed > other.elapsed;
  }
};

std::vector<int> constrainedSafePath(
  const GridSnapshot & grid, int start, int goal,
  const std::vector<double> & risk,
  const std::vector<double> & time_to_goal,
  double time_budget,
  double travel_yaw,
  const ClearancePlanner::Parameters & params,
  const std::chrono::steady_clock::time_point & started,
  std::size_t & expanded)
{
  std::vector<SafeLabel> labels;
  labels.reserve(grid.size() * 2);
  std::vector<std::vector<int>> pareto(grid.size());
  std::priority_queue<
    SafeQueueEntry, std::vector<SafeQueueEntry>, std::greater<SafeQueueEntry>> open;

  labels.push_back({start, -1, 0.0, 0.0, true});
  pareto[start].push_back(0);
  open.push({0.0, 0.0, 0});

  static constexpr int kDx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
  static constexpr int kDy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
  while (!open.empty()) {
    const int label_index = open.top().label;
    open.pop();
    if (!labels[label_index].active) {
      continue;
    }
    const SafeLabel current = labels[label_index];
    ++expanded;
    if (current.cell == goal) {
      std::vector<int> path;
      for (int index = label_index; index >= 0; index = labels[index].parent) {
        path.push_back(labels[index].cell);
      }
      std::reverse(path.begin(), path.end());
      return path;
    }
    if ((expanded & 0xFFU) == 0U && planningTimedOut(started, params.max_planning_time)) {
      return {};
    }

    const auto [x, y] = grid.coordinates(current.cell);
    for (int direction = 0; direction < 8; ++direction) {
      const int nx = x + kDx[direction];
      const int ny = y + kDy[direction];
      if (!grid.contains(nx, ny)) {
        continue;
      }
      const int neighbor = grid.index(nx, ny);
      if (!std::isfinite(risk[neighbor]) || !std::isfinite(time_to_goal[neighbor]) ||
        !diagonalMoveIsClear(grid, x, y, nx, ny, risk))
      {
        continue;
      }
      const double next_elapsed = current.elapsed +
        moveTime(grid, x, y, nx, ny, travel_yaw, params);
      if (next_elapsed + time_to_goal[neighbor] > time_budget + 1.0e-9) {
        continue;
      }
      const double step_distance = std::hypot(nx - x, ny - y) * grid.resolution;
      const double next_risk = current.risk + step_distance * 0.5 *
        (risk[current.cell] + risk[neighbor]);

      bool dominated = false;
      auto & frontier = pareto[neighbor];
      for (const int existing_index : frontier) {
        const SafeLabel & existing = labels[existing_index];
        if (existing.active &&
          existing.elapsed <= next_elapsed + 1.0e-9 &&
          existing.risk <= next_risk + 1.0e-9)
        {
          dominated = true;
          break;
        }
      }
      if (dominated) {
        continue;
      }
      for (const int existing_index : frontier) {
        SafeLabel & existing = labels[existing_index];
        if (existing.active &&
          next_elapsed <= existing.elapsed + 1.0e-9 &&
          next_risk <= existing.risk + 1.0e-9)
        {
          existing.active = false;
        }
      }
      frontier.erase(
        std::remove_if(
          frontier.begin(), frontier.end(),
          [&labels](int index) {return !labels[index].active;}),
        frontier.end());
      const int next_index = static_cast<int>(labels.size());
      labels.push_back({neighbor, label_index, next_elapsed, next_risk, true});
      frontier.push_back(next_index);
      open.push({next_risk, next_elapsed, next_index});
    }
  }
  return {};
}

double polylineCost(
  const GridSnapshot & grid, const std::vector<int> & cells,
  std::size_t first, std::size_t last,
  const std::vector<double> & multiplier, double & maximum)
{
  double cost = 0.0;
  maximum = 0.0;
  for (std::size_t i = first; i <= last; ++i) {
    maximum = std::max(maximum, multiplier[cells[i]]);
    if (i == first) {
      continue;
    }
    const auto [ax, ay] = grid.coordinates(cells[i - 1]);
    const auto [bx, by] = grid.coordinates(cells[i]);
    cost += std::hypot(ax - bx, ay - by) * 0.5 *
      (multiplier[cells[i - 1]] + multiplier[cells[i]]);
  }
  return cost;
}

double polylineTime(
  const GridSnapshot & grid, const std::vector<int> & cells,
  std::size_t first, std::size_t last, double travel_yaw,
  const ClearancePlanner::Parameters & params)
{
  double elapsed = 0.0;
  for (std::size_t index = first + 1; index <= last; ++index) {
    const auto [previous_x, previous_y] = grid.coordinates(cells[index - 1]);
    const auto [current_x, current_y] = grid.coordinates(cells[index]);
    elapsed += moveTime(
      grid, previous_x, previous_y, current_x, current_y, travel_yaw, params);
  }
  return elapsed;
}

bool directSegmentCost(
  const GridSnapshot & grid, int first, int last,
  const std::vector<double> & multiplier,
  double & cost, double & maximum)
{
  const auto [ax, ay] = grid.coordinates(first);
  const auto [bx, by] = grid.coordinates(last);
  const double length = std::hypot(ax - bx, ay - by);
  const int samples = std::max(1, static_cast<int>(std::ceil(length * 2.0)));
  cost = 0.0;
  maximum = 0.0;
  int previous_x = ax;
  int previous_y = ay;
  int previous = first;
  for (int sample = 1; sample <= samples; ++sample) {
    const double t = static_cast<double>(sample) / samples;
    const int x = static_cast<int>(std::lround(ax + (bx - ax) * t));
    const int y = static_cast<int>(std::lround(ay + (by - ay) * t));
    if (!grid.contains(x, y)) {
      return false;
    }
    const int current = grid.index(x, y);
    if (!std::isfinite(multiplier[current]) ||
      !diagonalMoveIsClear(grid, previous_x, previous_y, x, y, multiplier))
    {
      return false;
    }
    const double step = std::hypot(x - previous_x, y - previous_y);
    if (step > 0.0) {
      cost += step * 0.5 * (multiplier[previous] + multiplier[current]);
    }
    maximum = std::max(maximum, multiplier[current]);
    previous_x = x;
    previous_y = y;
    previous = current;
  }
  return true;
}

std::vector<int> simplifyPath(
  const GridSnapshot & grid, const std::vector<int> & cells,
  const std::vector<double> & multiplier, double cost_tolerance,
  double time_tolerance, double travel_yaw,
  const ClearancePlanner::Parameters & params)
{
  if (cells.size() < 3) {
    return cells;
  }
  std::vector<int> simplified;
  simplified.push_back(cells.front());
  std::size_t anchor = 0;
  while (anchor + 1 < cells.size()) {
    std::size_t accepted = anchor + 1;
    for (std::size_t candidate = cells.size() - 1; candidate > anchor + 1; --candidate) {
      double original_max = 0.0;
      const double original = polylineCost(
        grid, cells, anchor, candidate, multiplier, original_max);
      const double original_time = polylineTime(
        grid, cells, anchor, candidate, travel_yaw, params);
      double direct = 0.0;
      double direct_max = 0.0;
      const auto [anchor_x, anchor_y] = grid.coordinates(cells[anchor]);
      const auto [candidate_x, candidate_y] = grid.coordinates(cells[candidate]);
      const double direct_time = moveTime(
        grid, anchor_x, anchor_y, candidate_x, candidate_y, travel_yaw, params);
      if (directSegmentCost(
          grid, cells[anchor], cells[candidate], multiplier, direct, direct_max) &&
        direct <= original * cost_tolerance &&
        direct_max <= original_max * cost_tolerance + 0.05 &&
        direct_time <= original_time * time_tolerance + 1.0e-9)
      {
        accepted = candidate;
        break;
      }
    }
    simplified.push_back(cells[accepted]);
    anchor = accepted;
  }
  return simplified;
}

}  // namespace

void ClearancePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  costmap_ = costmap_ros_->getCostmap();

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("ClearancePlanner lifecycle node expired during configure");
  }
  logger_ = node->get_logger();
  const auto declare = [&node, this](const std::string & key, const auto & value) {
      nav2_util::declare_parameter_if_not_declared(
        node, name_ + "." + key, rclcpp::ParameterValue(value));
    };
  declare("allow_unknown", false);
  declare("tolerance", 0.10);
  declare("max_planning_time", 1.5);
  declare("forward_speed", 2.00);
  declare("reverse_speed", 1.20);
  declare("lateral_speed", 1.50);
  declare("max_wheel_speed", 2.00);
  declare("max_time_ratio", 1.10);
  declare("min_time_slack", 0.50);
  declare("costmap_weight", 3.0);
  declare("preferred_clearance", 0.65);
  declare("clearance_weight", 12.0);
  declare("clearance_power", 2.0);
  declare("density_radius", 0.75);
  declare("density_weight", 8.0);
  declare("density_normalization", 0.18);
  declare("goal_exemption_radius", 0.75);
  declare("start_exemption_radius", 0.45);
  declare("simplification_cost_tolerance", 1.03);
  declare("simplification_time_tolerance", 1.01);

  RCLCPP_INFO(
    logger_, "Configured %s: bounded-time clearance planning enabled",
    name_.c_str());
}

void ClearancePlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up %s", name_.c_str());
  costmap_ = nullptr;
  costmap_ros_.reset();
  tf_.reset();
}

void ClearancePlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating %s", name_.c_str());
}

void ClearancePlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating %s", name_.c_str());
}

ClearancePlanner::Parameters ClearancePlanner::readParameters() const
{
  Parameters params;
  auto node = node_.lock();
  if (!node) {
    return params;
  }
  const auto read = [&node, this](const std::string & key, auto & value) {
      node->get_parameter(name_ + "." + key, value);
    };
  read("allow_unknown", params.allow_unknown);
  read("tolerance", params.tolerance);
  read("max_planning_time", params.max_planning_time);
  read("forward_speed", params.forward_speed);
  read("reverse_speed", params.reverse_speed);
  read("lateral_speed", params.lateral_speed);
  read("max_wheel_speed", params.max_wheel_speed);
  read("max_time_ratio", params.max_time_ratio);
  read("min_time_slack", params.min_time_slack);
  read("costmap_weight", params.costmap_weight);
  read("preferred_clearance", params.preferred_clearance);
  read("clearance_weight", params.clearance_weight);
  read("clearance_power", params.clearance_power);
  read("density_radius", params.density_radius);
  read("density_weight", params.density_weight);
  read("density_normalization", params.density_normalization);
  read("goal_exemption_radius", params.goal_exemption_radius);
  read("start_exemption_radius", params.start_exemption_radius);
  read("simplification_cost_tolerance", params.simplification_cost_tolerance);
  read("simplification_time_tolerance", params.simplification_time_tolerance);

  params.tolerance = std::max(0.0, params.tolerance);
  params.max_planning_time = std::max(0.0, params.max_planning_time);
  params.forward_speed = std::max(0.05, params.forward_speed);
  params.reverse_speed = std::max(0.05, params.reverse_speed);
  params.lateral_speed = std::max(0.05, params.lateral_speed);
  params.max_wheel_speed = std::max(0.05, params.max_wheel_speed);
  params.max_time_ratio = std::max(1.0, params.max_time_ratio);
  params.min_time_slack = std::max(0.0, params.min_time_slack);
  params.costmap_weight = std::max(0.0, params.costmap_weight);
  params.preferred_clearance = std::max(0.0, params.preferred_clearance);
  params.clearance_weight = std::max(0.0, params.clearance_weight);
  params.clearance_power = std::max(0.1, params.clearance_power);
  params.density_radius = std::max(0.0, params.density_radius);
  params.density_weight = std::max(0.0, params.density_weight);
  params.density_normalization = std::max(0.001, params.density_normalization);
  params.goal_exemption_radius = std::max(0.0, params.goal_exemption_radius);
  params.start_exemption_radius = std::max(0.0, params.start_exemption_radius);
  params.simplification_cost_tolerance = std::max(
    1.0, params.simplification_cost_tolerance);
  params.simplification_time_tolerance = std::max(
    1.0, params.simplification_time_tolerance);
  return params;
}

nav_msgs::msg::Path ClearancePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  auto node = node_.lock();
  if (!node || costmap_ == nullptr || costmap_ros_ == nullptr) {
    return path;
  }
  path.header.stamp = node->now();
  path.header.frame_id = costmap_ros_->getGlobalFrameID();
  if ((!start.header.frame_id.empty() && start.header.frame_id != path.header.frame_id) ||
    (!goal.header.frame_id.empty() && goal.header.frame_id != path.header.frame_id))
  {
    RCLCPP_ERROR(
      logger_, "%s requires start and goal in frame '%s'", name_.c_str(),
      path.header.frame_id.c_str());
    return path;
  }

  GridSnapshot grid;
  {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
    grid.width = costmap_->getSizeInCellsX();
    grid.height = costmap_->getSizeInCellsY();
    grid.resolution = costmap_->getResolution();
    grid.origin_x = costmap_->getOriginX();
    grid.origin_y = costmap_->getOriginY();
    const unsigned char * first = costmap_->getCharMap();
    grid.costs.assign(first, first + grid.size());
  }

  int start_x = 0;
  int start_y = 0;
  int goal_x = 0;
  int goal_y = 0;
  if (!grid.worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
    !grid.worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y))
  {
    RCLCPP_ERROR(logger_, "%s start or goal lies outside the global costmap", name_.c_str());
    return path;
  }

  const Parameters params = readParameters();
  const auto planning_started = std::chrono::steady_clock::now();
  const std::vector<double> clearance = distanceField(grid, params.allow_unknown);
  const std::vector<int> density_integral = obstacleIntegralImage(grid, params.allow_unknown);
  const std::vector<double> risk = traversalRisks(
    grid, clearance, density_integral, params,
    start.pose.position.x, start.pose.position.y,
    goal.pose.position.x, goal.pose.position.y);

  const int start_index = grid.index(start_x, start_y);
  if (!std::isfinite(risk[start_index])) {
    RCLCPP_ERROR(logger_, "%s start pose is in a collision cell", name_.c_str());
    return path;
  }
  const int requested_goal = grid.index(goal_x, goal_y);
  const int goal_index = nearestTraversableGoal(
    grid, goal_x, goal_y, risk, params.tolerance);
  if (goal_index < 0) {
    RCLCPP_ERROR(logger_, "%s goal is blocked within %.2f m tolerance", name_.c_str(), params.tolerance);
    return path;
  }

  std::size_t expanded = 0;
  const double travel_yaw = poseYaw(goal.pose.orientation);
  const std::vector<double> time_to_goal = fastestTimeField(
    grid, goal_index, risk, travel_yaw, params, planning_started, expanded);
  if (time_to_goal.empty() || !std::isfinite(time_to_goal[start_index])) {
    RCLCPP_WARN(logger_, "%s found no time-feasible route", name_.c_str());
    return path;
  }
  const std::vector<int> fastest_cells = reconstructFastestPath(
    grid, start_index, goal_index, risk, time_to_goal, travel_yaw, params);
  if (fastest_cells.empty()) {
    RCLCPP_WARN(logger_, "%s could not reconstruct its fastest route", name_.c_str());
    return path;
  }
  const double fastest_time = time_to_goal[start_index];
  const double time_budget = fastest_time + std::max(
    params.min_time_slack,
    fastest_time * (params.max_time_ratio - 1.0));
  std::vector<int> cells = constrainedSafePath(
    grid, start_index, goal_index, risk, time_to_goal, time_budget,
    travel_yaw, params, planning_started, expanded);
  if (cells.empty()) {
    cells = fastest_cells;
    RCLCPP_WARN(
      logger_, "%s safety search timed out; using %.2f s fastest route",
      name_.c_str(), fastest_time);
  }
  cells = simplifyPath(
    grid, cells, risk, params.simplification_cost_tolerance,
    params.simplification_time_tolerance, travel_yaw, params);

  std::vector<std::pair<double, double>> controls;
  controls.emplace_back(start.pose.position.x, start.pose.position.y);
  for (std::size_t i = 1; i + 1 < cells.size(); ++i) {
    const auto [x, y] = grid.coordinates(cells[i]);
    controls.push_back(grid.mapToWorld(x, y));
  }
  if (goal_index == requested_goal) {
    controls.emplace_back(goal.pose.position.x, goal.pose.position.y);
  } else {
    const auto [x, y] = grid.coordinates(goal_index);
    controls.push_back(grid.mapToWorld(x, y));
  }

  std::vector<std::pair<double, double>> points;
  points.push_back(controls.front());
  for (std::size_t i = 1; i < controls.size(); ++i) {
    const auto [ax, ay] = controls[i - 1];
    const auto [bx, by] = controls[i];
    const double length = distanceBetween(ax, ay, bx, by);
    const int samples = std::max(
      1, static_cast<int>(std::ceil(length / grid.resolution)));
    for (int sample = 1; sample <= samples; ++sample) {
      const double t = static_cast<double>(sample) / samples;
      points.emplace_back(ax + (bx - ax) * t, ay + (by - ay) * t);
    }
  }

  path.poses.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = points[i].first;
    pose.pose.position.y = points[i].second;
    // Omni translation is independent of task yaw. Copy it onto every pose so
    // an MPPI-pruned path cannot expose a segment tangent as the goal heading.
    pose.pose.orientation = goal.pose.orientation;
    path.poses.push_back(std::move(pose));
  }

  RCLCPP_DEBUG(
    logger_,
    "%s planned %zu poses via %zu controls: fastest=%.2f s budget=%.2f s expanded=%zu",
    name_.c_str(), path.poses.size(), controls.size(), fastest_time, time_budget, expanded);
  return path;
}

}  // namespace medical_clearance_planner

PLUGINLIB_EXPORT_CLASS(
  medical_clearance_planner::ClearancePlanner,
  nav2_core::GlobalPlanner)
