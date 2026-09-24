#ifndef CHASSIS_CTRL_H
#define CHASSIS_CTRL_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void ChassisCtrl_MoveTarget(float x_target, float y_target, float yaw_target,
                            float pos_x, float pos_y, float yaw);
void ChassisCtrl_Enable(bool enable);
bool ChassisCtrl_Update(float pos_x, float pos_y, float yaw);
bool ChassisCtrl_ReachFlag(void);

#ifdef __cplusplus
}
#endif

#endif
