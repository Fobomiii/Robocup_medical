#ifndef MEDICAL_CLEARANCE_PLANNER__PARKING_EXEMPTION_LAYER_HPP_
#define MEDICAL_CLEARANCE_PLANNER__PARKING_EXEMPTION_LAYER_HPP_

#include <string>

#include "medical_clearance_planner/parking_exemption_geometry.hpp"
#include "nav2_costmap_2d/layer.hpp"

namespace medical_clearance_planner
{

class ParkingExemptionLayer : public nav2_costmap_2d::Layer
{
public:
  ParkingExemptionLayer() = default;
  ~ParkingExemptionLayer() override = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override;

private:
  struct FrameTransform
  {
    double translation_x{0.0};
    double translation_y{0.0};
    double cosine{1.0};
    double sine{0.0};
    bool valid{false};
  };

  bool updateFrameTransform();
  void zoneToGlobal(double zone_x, double zone_y, double & global_x, double & global_y) const;
  void globalToZone(double global_x, double global_y, double & zone_x, double & zone_y) const;

  std::string zone_frame_{"map"};
  ParkingExemptionGeometry geometry_;
  FrameTransform frame_transform_;
};

}  // namespace medical_clearance_planner

#endif  // MEDICAL_CLEARANCE_PLANNER__PARKING_EXEMPTION_LAYER_HPP_
