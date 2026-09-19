#include "PathFollower.h"
#include "ChassisCtrl.h"
#include <math.h>

#define PATH_PASS_DISTANCE_MM 100.0f
#define PATH_PASS_YAW_DEG       8.0f

static NUC_NavGoal s_goal;
static NUC_NavStatus s_status;
static uint16_t s_path_generation;
static uint16_t s_path_id;
static uint8_t s_waypoint_index;

static float path_angle_error(float target_deg, float current_deg)
{
  float error = target_deg - current_deg;
  while (error > 180.0f)
  {
    error -= 360.0f;
  }
  while (error < -180.0f)
  {
    error += 360.0f;
  }
  return fabsf(error);
}

static uint8_t path_can_pass_intermediate(const NUC_NavWaypoint *waypoint,
                                          float pos_x, float pos_y, float pos_z)
{
  float dx = (float)waypoint->x_mm - pos_x;
  float dy = (float)waypoint->y_mm - pos_y;
  float yaw = (float)waypoint->yaw_cdeg / 100.0f;
  return ((dx * dx + dy * dy) <=
          (PATH_PASS_DISTANCE_MM * PATH_PASS_DISTANCE_MM) &&
          path_angle_error(yaw, pos_z) <= PATH_PASS_YAW_DEG) ? 1U : 0U;
}

void PathFollower_Init(void)
{
  s_goal = NUC_NAV_GOAL_NONE;
  s_status = NUC_NAV_IDLE;
  s_path_generation = 0U;
  s_path_id = 0U;
  s_waypoint_index = 0U;
}

bool PathFollower_Update(NUC_NavGoal goal, float pos_x, float pos_y, float pos_z)
{
  uint16_t generation;
  uint8_t count;
  NUC_NavWaypoint waypoint;

  if (goal == NUC_NAV_GOAL_NONE)
  {
    s_status = NUC_NAV_ERROR;
    return false;
  }

  if (goal != s_goal)
  {
    s_goal = goal;
    s_status = NUC_NAV_WAIT_PATH;
    s_path_generation = 0U;
    s_path_id = 0U;
    s_waypoint_index = 0U;
    /* Hold the current pose while the NUC prepares the first complete path. */
    ChassisCtrl_MoveTarget(pos_x, pos_y, pos_z, pos_x, pos_y, pos_z);
    NUC_Nav_RequestGoal(goal);
  }

  if (!NUC_Nav_HasRequestedPath())
  {
    s_status = NUC_NAV_WAIT_PATH;
    ChassisCtrl_MoveTarget(pos_x, pos_y, pos_z, pos_x, pos_y, pos_z);
    return false;
  }

  generation = NUC_Nav_GetPathGeneration();
  if (generation != s_path_generation)
  {
    s_path_generation = generation;
    s_path_id = NUC_Nav_GetPathId();
    s_waypoint_index = 0U;
    s_status = NUC_NAV_FOLLOWING;
  }

  count = NUC_Nav_GetPathCount();
  if (count == 0U || s_waypoint_index >= count)
  {
    s_status = (count == 0U) ? NUC_NAV_ERROR : NUC_NAV_REACHED;
    return s_status == NUC_NAV_REACHED;
  }

  if (!NUC_Nav_GetWaypoint(s_waypoint_index, &waypoint))
  {
    s_status = NUC_NAV_ERROR;
    return false;
  }

  ChassisCtrl_MoveTarget((float)waypoint.x_mm,
                         (float)waypoint.y_mm,
                         (float)waypoint.yaw_cdeg / 100.0f,
                         pos_x, pos_y, pos_z);

  if (ChassisCtrl_ReachFlag() ||
      (s_waypoint_index + 1U < count &&
       path_can_pass_intermediate(&waypoint, pos_x, pos_y, pos_z)))
  {
    s_waypoint_index++;
    if (s_waypoint_index >= count)
    {
      s_status = NUC_NAV_REACHED;
      return true;
    }

    if (NUC_Nav_GetWaypoint(s_waypoint_index, &waypoint))
    {
      ChassisCtrl_MoveTarget((float)waypoint.x_mm,
                             (float)waypoint.y_mm,
                             (float)waypoint.yaw_cdeg / 100.0f,
                             pos_x, pos_y, pos_z);
    }
  }

  s_status = NUC_NAV_FOLLOWING;
  return false;
}

NUC_NavStatus PathFollower_GetStatus(void)
{
  return s_status;
}

uint16_t PathFollower_GetPathId(void)
{
  return s_path_id;
}

uint8_t PathFollower_GetWaypointIndex(void)
{
  return s_waypoint_index;
}
