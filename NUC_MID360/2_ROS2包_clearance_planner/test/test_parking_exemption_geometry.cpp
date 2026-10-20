#include <gtest/gtest.h>

#include "medical_clearance_planner/parking_exemption_geometry.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace medical_clearance_planner
{

TEST(ParkingExemptionGeometry, IncludesBothBedCirclesAndHomeSquare)
{
  const ParkingExemptionGeometry geometry;
  EXPECT_TRUE(insideParkingExemption(5.4, 2.25, geometry));
  EXPECT_TRUE(insideParkingExemption(5.4, -2.2, geometry));
  EXPECT_TRUE(insideParkingExemption(5.699, 2.25, geometry));
  EXPECT_FALSE(insideParkingExemption(5.71, 2.25, geometry));
  EXPECT_TRUE(insideParkingExemption(0.49, -0.49, geometry));
  EXPECT_FALSE(insideParkingExemption(0.51, 0.0, geometry));
}

TEST(ParkingExemptionGeometry, IncludesGridCellsIntersectingPhysicalZones)
{
  const ParkingExemptionGeometry geometry;
  EXPECT_TRUE(cellIntersectsParkingExemption(5.724, 2.25, 0.05, geometry));
  EXPECT_FALSE(cellIntersectsParkingExemption(5.735, 2.25, 0.05, geometry));
  EXPECT_TRUE(cellIntersectsParkingExemption(0.524, 0.0, 0.05, geometry));
  EXPECT_FALSE(cellIntersectsParkingExemption(0.535, 0.0, 0.05, geometry));
}

TEST(ParkingExemptionGeometry, ClearsInflationButPreservesHardObstacles)
{
  EXPECT_EQ(
    clearParkingInflation(nav2_costmap_2d::FREE_SPACE),
    nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(clearParkingInflation(1), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(clearParkingInflation(252), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(
    clearParkingInflation(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE),
    nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(
    clearParkingInflation(nav2_costmap_2d::LETHAL_OBSTACLE),
    nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(
    clearParkingInflation(nav2_costmap_2d::NO_INFORMATION),
    nav2_costmap_2d::NO_INFORMATION);
}

}  // namespace medical_clearance_planner
