#include "ChassisCtrl.h"
#include "PID.h"
#include "DJIMotorCtrlSTM32.h"
#include <math.h>

static float long_distance_thd = 350.0f;/* 最小长距离变量 */
static float get_target_thd_x = 2.0f;   /* 到达判断 */
static float get_target_thd_y = 2.0f;
static float get_target_thd_z = 0.5f;
static float target_distance = 0.0f;    /* 初始距离 */
static float max_vel = 200.0f;         /* 最大合速度 */
static float result_vel = 0.0f;        /* 合速度 */
static uint8_t velPIDCtrlFlag = 0;     /* 是否将 PID 直接输出到分速度 */
static float cosf_Vx = 0.0f;
static float sinf_Vy = 0.0f;

static float Vx, Vy, W;

static volatile float target_x, target_y, target_z;
static bool set_enabled;
static volatile bool reachFlag = 0;

/* 世界系避障偏置：跟踪点 = target + oa */
static volatile float oa_dx = 0.0f;
static volatile float oa_dy = 0.0f;

void ChassisCtrl_SetOAOffset(float dx, float dy)
{
    oa_dx = dx;
    oa_dy = dy;
}

void ChassisCtrl_ClearOA(void)
{
    oa_dx = 0.0f;
    oa_dy = 0.0f;
}

/********设置目标点**********/
void ChassisCtrl_MoveTarget(float x_tgt, float y_tgt, float z_tgt,
                            float pos_x, float pos_y, float pos_z)
{
    if ((target_x == x_tgt) && (target_y == y_tgt) && (target_z == z_tgt)) {
        return;
    }
    target_x = x_tgt;
    target_y = y_tgt;
    target_z = z_tgt;
    result_vel = 0.0f;
    Vx = 0.0f;
    Vy = 0.0f;
    W = 0.0f;
    velPIDCtrlFlag = 0;
    reachFlag = 0;
    PID_Reset();
    ChassisCtrl_ClearOA();

    target_distance = sqrtf((target_x - pos_x) * (target_x - pos_x) +
                            (target_y - pos_y) * (target_y - pos_y));
    if (target_distance < 1e-3f) {
        target_distance = 0.f;
        cosf_Vx = 1.f;
        sinf_Vy = 0.f;
    } else {
        cosf_Vx = (target_x - pos_x) / target_distance;
        sinf_Vy = (target_y - pos_y) / target_distance;
    }
}

/**********使能设置***********/
void ChassisCtrl_Enable(bool enable)
{
    set_enabled = enable;
}

/*************后台运行************/
bool ChassisCtrl_Update(float pos_x, float pos_y, float pos_z)
{
    static float newVx, newVy, newW;
    float cos_f32 = cosf(pos_z * 3.14159265f / 180.f);
    float sin_f32 = sinf(pos_z * 3.14159265f / 180.f);
    static float result_vel_step = 10.0f;
    static float angle_step = 0.01f;

    float track_x = target_x + oa_dx;
    float track_y = target_y + oa_dy;
    float track_z = target_z;
    uint8_t oa_active = (fabsf(oa_dx) > 1e-3f) || (fabsf(oa_dy) > 1e-3f);

    newVx = PID_UpdateX(track_x, pos_x);
    newVy = PID_UpdateY(track_y, pos_y);
    newW = PID_UpdateZ(track_z, pos_z);

    /* 到位：偏置已清且靠近原航点，避免侧移点误触发 */
    if (!oa_active &&
        fabsf(target_x - pos_x) < get_target_thd_x &&
        fabsf(target_y - pos_y) < get_target_thd_y &&
        fabsf(target_z - pos_z) < get_target_thd_z) {
        DJI_Chassis_SetCommand(0.0f, 0.0f, 0.0f);
        PID_Reset();
        result_vel = 0.0f;
        velPIDCtrlFlag = 0;
        reachFlag = 1;
        return 1;
    }

    /* OA 激活：强制全程 PID，侧移更跟手 */
    if (oa_active) {
        velPIDCtrlFlag = 1;
    } else if (target_distance <= long_distance_thd) {
        velPIDCtrlFlag = 1;
    } else {
        float remain = sqrtf((track_x - pos_x) * (track_x - pos_x) +
                             (track_y - pos_y) * (track_y - pos_y));
        float now_result_vel = sqrtf(newVx * newVx + newVy * newVy);
        if (remain > long_distance_thd) {
            if (result_vel < max_vel) {
                result_vel += result_vel_step;
            }
            Vx = result_vel * cosf_Vx * cos_f32 - result_vel * sinf_Vy * sin_f32;
            Vy = result_vel * cosf_Vx * sin_f32 + result_vel * sinf_Vy * cos_f32;
        } else if (result_vel > now_result_vel) {
            if (result_vel > 0) {
                result_vel -= result_vel_step;
                if (now_result_vel > 1e-3f) {
                    float cos_pid = newVx / now_result_vel;
                    float sin_pid = newVy / now_result_vel;

                    if (cosf_Vx > cos_pid + angle_step) {
                        cosf_Vx -= angle_step;
                    } else if (cosf_Vx < cos_pid - angle_step) {
                        cosf_Vx += angle_step;
                    }

                    if (sinf_Vy > sin_pid + angle_step) {
                        sinf_Vy -= angle_step;
                    } else if (sinf_Vy < sin_pid - angle_step) {
                        sinf_Vy += angle_step;
                    }
                }
                Vx = result_vel * cosf_Vx * cos_f32 - result_vel * sinf_Vy * sin_f32;
                Vy = result_vel * cosf_Vx * sin_f32 + result_vel * sinf_Vy * cos_f32;
            }
        } else if (result_vel <= now_result_vel) {
            velPIDCtrlFlag = 1;
        }
    }

    if (velPIDCtrlFlag) {
        Vx = newVx * cos_f32 - newVy * sin_f32;
        Vy = newVx * sin_f32 + newVy * cos_f32;
    } else {
        PID_ResetXY();
    }

    W = newW;
    DJI_Chassis_SetCommand(Vx, Vy, W);

    reachFlag = 0;
    return 0;
}

bool ChassisCtrl_ReachFlag(void)
{
    return reachFlag;
}
