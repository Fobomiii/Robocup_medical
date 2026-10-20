#include <cmath>

#include "gtest/gtest.h"
#include "medical_clearance_planner/corner_geometry.hpp"

namespace medical_clearance_planner
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

TEST(CornerGeometry, CreatesExactRightAngleFillet)
{
  const auto fillet = makeCircularFillet(
    {0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0},
    0.75, 0.55, 0.45, 20.0 * kPi / 180.0, 120.0 * kPi / 180.0);
  ASSERT_TRUE(fillet.has_value());
  EXPECT_NEAR(fillet->radius, 0.75, 1.0e-9);
  EXPECT_NEAR(fillet->entry.x, 1.25, 1.0e-9);
  EXPECT_NEAR(fillet->entry.y, 0.0, 1.0e-9);
  EXPECT_NEAR(fillet->exit.x, 2.0, 1.0e-9);
  EXPECT_NEAR(fillet->exit.y, 0.75, 1.0e-9);
  EXPECT_NEAR(fillet->sweep_angle, 0.5 * kPi, 1.0e-9);

  const auto samples = sampleCircularFillet(*fillet, 0.025);
  ASSERT_GT(samples.size(), 2U);
  for (const auto & point : samples) {
    EXPECT_NEAR(
      std::hypot(point.x - fillet->center.x, point.y - fillet->center.y),
      fillet->radius, 1.0e-9);
  }
  for (std::size_t index = 1; index < samples.size(); ++index) {
    EXPECT_LE(
      std::hypot(
        samples[index].x - samples[index - 1].x,
        samples[index].y - samples[index - 1].y),
      0.025 + 1.0e-9);
  }
}

TEST(CornerGeometry, ReducesRadiusToPreventAdjacentFilletOverlap)
{
  const auto fillet = makeCircularFillet(
    {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0},
    0.75, 0.40, 0.45, 20.0 * kPi / 180.0, 120.0 * kPi / 180.0);
  ASSERT_TRUE(fillet.has_value());
  EXPECT_NEAR(fillet->radius, 0.45, 1.0e-9);
}

TEST(CornerGeometry, KeepsCornerWhenMinimumRadiusDoesNotFit)
{
  const auto fillet = makeCircularFillet(
    {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0},
    0.75, 0.55, 0.45, 20.0 * kPi / 180.0, 120.0 * kPi / 180.0);
  EXPECT_FALSE(fillet.has_value());
}

TEST(CornerGeometry, DoesNotRoundNearReversal)
{
  const auto fillet = makeCircularFillet(
    {0.0, 0.0}, {1.0, 0.0}, {0.2, 0.4},
    0.75, 0.20, 0.45, 20.0 * kPi / 180.0, 120.0 * kPi / 180.0);
  EXPECT_FALSE(fillet.has_value());
}

TEST(CornerGeometry, ReplacesShortDiagonalTransitionWithOneFillet)
{
  const auto fillet = makeTransitionFillet(
    {0.0, -0.5}, {0.0, 1.0}, {0.2, 1.2}, {2.0, 1.2},
    0.60, 0.75, 0.55, 0.45,
    20.0 * kPi / 180.0, 120.0 * kPi / 180.0);
  ASSERT_TRUE(fillet.has_value());
  EXPECT_NEAR(fillet->corner.x, 0.0, 1.0e-9);
  EXPECT_NEAR(fillet->corner.y, 1.2, 1.0e-9);
  EXPECT_NEAR(fillet->radius, 0.75, 1.0e-9);
  EXPECT_NEAR(fillet->entry.x, 0.0, 1.0e-9);
  EXPECT_NEAR(fillet->entry.y, 0.45, 1.0e-9);
  EXPECT_NEAR(fillet->exit.x, 0.75, 1.0e-9);
  EXPECT_NEAR(fillet->exit.y, 1.2, 1.0e-9);
}

}  // namespace
}  // namespace medical_clearance_planner
