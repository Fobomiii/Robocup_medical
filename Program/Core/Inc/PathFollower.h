/**
 * @file PathFollower.h
 * @brief Execute NUC-provided global waypoints with the existing position loop.
 */
#ifndef PATH_FOLLOWER_H
#define PATH_FOLLOWER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "NUC_Obstacle.h"
#include <stdbool.h>
#include <stdint.h>

void PathFollower_Init(void);

/** Request goal and update its current waypoint. Returns true at final point. */
bool PathFollower_Update(NUC_NavGoal goal, float pos_x, float pos_y, float pos_z);

NUC_NavStatus PathFollower_GetStatus(void);
uint16_t PathFollower_GetPathId(void);
uint8_t PathFollower_GetWaypointIndex(void);

#ifdef __cplusplus
}
#endif

#endif /* PATH_FOLLOWER_H */
