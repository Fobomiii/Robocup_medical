#include "Navigation.h"
#include "ChassisCtrl.h"
#include "GM65.h"
#include "Servo.h"
#include "OPS.h"
#include "HWT101CT.h"
#include <stdlib.h>
#include <math.h>

/* OPS.h 的 pos_x/pos_y 宏会撞上 MapPosFunction.pos_x；反馈统一用 getter */
#undef pos_x
#undef pos_y

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ---- 避障参数（与 OPS 单位一致，当前工程常用 mm） ---- */
#define OA_DIST_MM       300.0f   /* 侧向让开距离 */
#define OA_MAX_ALONG_MM  600.0f   /* 进入绕障：障碍在前方此距离内 */
#define OA_MIN_ALONG_MM  80.0f    /* 越过障碍：前方距离小于此则回中 */
#define OA_DETECT_EPS    1e-3f

static uint8_t  s_oa_enable = 1U;
static uint8_t  s_path_have_obstacle;
static float    s_path_obs_x, s_path_obs_y;
static float    s_oa_bx, s_oa_by;          /* 最新车体障碍 */
static uint8_t  s_oa_body_valid;          /* 本帧障碍是否有效 */
static int      s_oa_fsm_state = -1;      /* FSM 换态清锁存（在 FSM_Update 里更新） */

void OA_Enable(uint8_t enable)
{
    s_oa_enable = enable ? 1U : 0U;
    if (!enable) {
        OA_Reset();
    }
}

void OA_Reset(void)
{
    s_path_have_obstacle = 0U;
    s_path_obs_x = 0.f;
    s_path_obs_y = 0.f;
    s_oa_bx = 0.f;
    s_oa_by = 0.f;
    s_oa_body_valid = 0U;
    ChassisCtrl_ClearOA();
}

void OA_SetObstacleBody(float bx, float by, uint8_t valid)
{
    if (!valid) {
        s_oa_bx = 0.f;
        s_oa_by = 0.f;
        s_oa_body_valid = 0U;
        return;
    }
    s_oa_bx = bx;
    s_oa_by = by;
    s_oa_body_valid = 1U;
}

/**
 * 坐标系：全局 +Y 前、+X 右、θ 顺时针(°)；
 * left = (-cosθ, sinθ)，forward = (sinθ, cosθ)，right = (cosθ, -sinθ)
 */
void OA_Update(void)
{
    float th, c, s;
    float fwd_x, fwd_y, right_x, right_y, left_x, left_y;
    float rx, ry, along, across, side;

    if (!s_oa_enable) {
        ChassisCtrl_ClearOA();
        return;
    }

    {
        float px = OPS_GetX();
        float py = OPS_GetY();

        th = pos_z * ((float)M_PI / 180.0f);
        c = cosf(th);
        s = sinf(th);
        fwd_x = s;
        fwd_y = c;
        right_x = c;
        right_y = -s;
        left_x = -c;
        left_y = s;

        /* 首次检测到车体障碍 → 锁存世界坐标（本段只锁一次） */
        if (!s_path_have_obstacle && s_oa_body_valid &&
            (fabsf(s_oa_bx) > OA_DETECT_EPS || fabsf(s_oa_by) > OA_DETECT_EPS)) {
            s_path_have_obstacle = 1U;
            /* obs = pos + BX*right + BY*forward */
            s_path_obs_x = px + s_oa_bx * c + s_oa_by * s;
            s_path_obs_y = py - s_oa_bx * s + s_oa_by * c;
        }

        if (!s_path_have_obstacle) {
            ChassisCtrl_ClearOA();
            return;
        }

        rx = s_path_obs_x - px;
        ry = s_path_obs_y - py;
    }
    along = rx * fwd_x + ry * fwd_y;
    across = rx * right_x + ry * right_y;

    if (along < OA_MAX_ALONG_MM && along > OA_MIN_ALONG_MM) {
        /* 障碍偏右 → 往左让；偏左 → 往右让 */
        side = (across > 0.0f) ? 1.0f : -1.0f;
        ChassisCtrl_SetOAOffset(side * OA_DIST_MM * left_x,
                                side * OA_DIST_MM * left_y);
    } else if (along <= OA_MIN_ALONG_MM) {
        ChassisCtrl_ClearOA();
        s_path_have_obstacle = 0U;
        s_oa_bx = 0.f;
        s_oa_by = 0.f;
        s_oa_body_valid = 0U;
    } else {
        ChassisCtrl_ClearOA();
    }
}

/*      枚举说明
GO_TO   去到
SCAN    扫描
OPERATE 操作
PREPARE 准备
TRANSIT 过渡态
BACK    回到
*/
typedef enum{
    /********任务初始 确定先去哪里********/
    
    GO_TO_NURSE,//去护士台
    SCAN_QR_CODE,//扫描护士台二维码确定先去哪床送哪药
    OPERATE_SCAN_QR_CODE,//扫不到二维码就到这个状态挪动下

    /*************去一号床***************/
    GO_TO_BED_ONE,//去一号床
    SCAN_BED_ONE_BAR_CODE,//扫码一号床条形码
    OPERATE_BED_ONE_LASER_POSITIONING,//激光测距定位
    OPERATE_BED_ONE,//放置一床药品 播报语音
    PREPARE_TO_BED_THREE,//准备去三床 先旋转-180度再走 
    
    /**************从一床到三床***************/
    TRANSIT_BED_ONE_TO_THREE_STEP_ONE,//从一床到三床 阶段1 直行
    TRANSIT_BED_ONE_TO_THREE_STEP_TWO,//从一床到三床 阶段2 转到90度
    TRANSIT_BED_ONE_TO_THREE_STEP_THREE,//从一床到三床 阶段3 直行
    TRANSIT_BED_ONE_TO_THREE_STEP_FOUR,//从一床到三床 阶段4 转到0度
    
    /*************去三床***************/
    GO_TO_BED_THREE,//去三床
    SCAN_BED_THREE_BAR_CODE,//扫码三床条形码
    OPERATE_BED_THREE_LASER_POSITIONING,//激光测距定位
    OPERATE_BED_THREE,//放置三床药品 播报语音
    PREPARE_TO_BED_ONE,//准备去一床 先旋转-180度再走  

    /**************从三床到一床***************/
    TRANSIT_BED_THREE_TO_ONE_STEP_ONE,//从三床到一床 阶段1 直行
    TRANSIT_BED_THREE_TO_ONE_STEP_TWO,//从三床到一床 阶段2 转到-90度
    TRANSIT_BED_THREE_TO_ONE_STEP_THREE,//从三床到一床 阶段3 直行
    TRANSIT_BED_THREE_TO_ONE_STEP_FOUR,//从三床到一床 阶段4 转到0度

    /****************准备回去*************/
    PREPARE_TO_BACK_HOME,//准备回原点 先旋转-180度

    BACK_HOME//回到原点完成任务

}SystemState_TypeDef;//FSM枚举

typedef enum{
    LEFT_MEDICINE,//左边的药箱
    RIGHT_MEDICINE//右边的药箱
}medicine_TypeDef;//药箱枚举

typedef struct{
    float pos_x;//目标X
    float pos_y;//目标Y
    float heading;//目标航向
    bool order;//出发顺序 1先 0后
    medicine_TypeDef medicine;//目标药箱
    int bar_code;//床头柜的条形码
}MapPosFunction_Typedef;//地图点位 任务映射

SystemState_TypeDef currentSystemState = GO_TO_NURSE;
MapPosFunction_Typedef mapPosFunction[30];

void MapPos_Init(void){
    /**护士台**/
    mapPosFunction[GO_TO_NURSE].pos_x = 0.0f;
    mapPosFunction[GO_TO_NURSE].pos_y = 0.0f;
    mapPosFunction[GO_TO_NURSE].heading = 0.0f;

    /***去一号床*/
    mapPosFunction[GO_TO_BED_ONE].pos_x = 0.0f;
    mapPosFunction[GO_TO_BED_ONE].pos_y = 0.0f;
    mapPosFunction[GO_TO_BED_ONE].heading = 0.0f;

    mapPosFunction[PREPARE_TO_BED_THREE].pos_x = 0.0f;
    mapPosFunction[PREPARE_TO_BED_THREE].pos_y = 0.0f;
    mapPosFunction[PREPARE_TO_BED_THREE].heading = -180.0f;

    /***去三号床*/
    mapPosFunction[GO_TO_BED_THREE].pos_x = 0.0f;
    mapPosFunction[GO_TO_BED_THREE].pos_y = 0.0f;
    mapPosFunction[GO_TO_BED_THREE].heading = 0.0f;

    mapPosFunction[PREPARE_TO_BED_ONE].pos_x = 0.0f;
    mapPosFunction[PREPARE_TO_BED_ONE].pos_y = 0.0f;
    mapPosFunction[PREPARE_TO_BED_ONE].heading = -180.0f;

    /****从一号床到三号床*/
    mapPosFunction[TRANSIT_BED_ONE_TO_THREE_STEP_ONE].pos_x = 0.0f;
    mapPosFunction[TRANSIT_BED_ONE_TO_THREE_STEP_ONE].pos_y = 0.0f;
    mapPosFunction[TRANSIT_BED_ONE_TO_THREE_STEP_ONE].heading = -180.0f;

    mapPosFunction[TRANSIT_BED_ONE_TO_THREE_STEP_TWO].pos_x = 0.0f;
    mapPosFunction[TRANSIT_BED_ONE_TO_THREE_STEP_TWO].pos_y = 0.0f;
    mapPosFunction[TRANSIT_BED_ONE_TO_THREE_STEP_TWO].heading = 90.0f;

    mapPosFunction[TRANSIT_BED_ONE_TO_THREE_STEP_THREE].pos_x = 0.0f;
    mapPosFunction[TRANSIT_BED_ONE_TO_THREE_STEP_THREE].pos_y = 0.0f;
    mapPosFunction[TRANSIT_BED_ONE_TO_THREE_STEP_THREE].heading = 90.0f;

    mapPosFunction[TRANSIT_BED_ONE_TO_THREE_STEP_FOUR].pos_x = 0.0f;
    mapPosFunction[TRANSIT_BED_ONE_TO_THREE_STEP_FOUR].pos_y = 0.0f;
    mapPosFunction[TRANSIT_BED_ONE_TO_THREE_STEP_FOUR].heading = 0.0f;

    /****从三号床到一号床*/
    mapPosFunction[TRANSIT_BED_THREE_TO_ONE_STEP_ONE].pos_x = 0.0f;
    mapPosFunction[TRANSIT_BED_THREE_TO_ONE_STEP_ONE].pos_y = 0.0f;
    mapPosFunction[TRANSIT_BED_THREE_TO_ONE_STEP_ONE].heading = -180.0f;

    mapPosFunction[TRANSIT_BED_THREE_TO_ONE_STEP_TWO].pos_x = 0.0f;
    mapPosFunction[TRANSIT_BED_THREE_TO_ONE_STEP_TWO].pos_y = 0.0f;
    mapPosFunction[TRANSIT_BED_THREE_TO_ONE_STEP_TWO].heading = -90.0f;

    mapPosFunction[TRANSIT_BED_THREE_TO_ONE_STEP_THREE].pos_x = 0.0f;
    mapPosFunction[TRANSIT_BED_THREE_TO_ONE_STEP_THREE].pos_y = 0.0f;
    mapPosFunction[TRANSIT_BED_THREE_TO_ONE_STEP_THREE].heading = -90.0f;

    mapPosFunction[TRANSIT_BED_THREE_TO_ONE_STEP_FOUR].pos_x = 0.0f;
    mapPosFunction[TRANSIT_BED_THREE_TO_ONE_STEP_FOUR].pos_y = 0.0f;
    mapPosFunction[TRANSIT_BED_THREE_TO_ONE_STEP_FOUR].heading = 0.0f;

    /****回家 */
    mapPosFunction[PREPARE_TO_BACK_HOME].pos_x = 0.0f;
    mapPosFunction[PREPARE_TO_BACK_HOME].pos_y = 0.0f;
    mapPosFunction[PREPARE_TO_BACK_HOME].heading = -180.0f;

    mapPosFunction[BACK_HOME].pos_x = 0.0f;
    mapPosFunction[BACK_HOME].pos_y = 0.0f;
    mapPosFunction[BACK_HOME].heading = 0.0f;
}
void FSM_Update(void){/* 纯 FSM，不含避障 */

    /* 换航段时清 OA 锁存；扫码/放药/原地转关闭避障 */
    if (s_oa_fsm_state != (int)currentSystemState) {
        s_oa_fsm_state = (int)currentSystemState;
        s_path_have_obstacle = 0U;
        s_oa_bx = 0.f;
        s_oa_by = 0.f;
        s_oa_body_valid = 0U;
        ChassisCtrl_ClearOA();

        switch (currentSystemState) {
        case GO_TO_NURSE:
        case GO_TO_BED_ONE:
        case GO_TO_BED_THREE:
        case TRANSIT_BED_ONE_TO_THREE_STEP_ONE:
        case TRANSIT_BED_ONE_TO_THREE_STEP_THREE:
        case TRANSIT_BED_THREE_TO_ONE_STEP_ONE:
        case TRANSIT_BED_THREE_TO_ONE_STEP_THREE:
        case BACK_HOME:
            OA_Enable(1);
            break;
        default:
            OA_Enable(0);
            break;
        }
    }

    switch(currentSystemState){
        /********任务初始 确定先去哪里********/
        case GO_TO_NURSE:{
            ChassisCtrl_MoveTarget(mapPosFunction[GO_TO_NURSE].pos_x,
                                   mapPosFunction[GO_TO_NURSE].pos_y,
                                   mapPosFunction[GO_TO_NURSE].heading,
                                   OPS_GetX(),OPS_GetY(),pos_z);
            
            if(ChassisCtrl_ReachFlag()){
                currentSystemState = SCAN_QR_CODE;//到达后到下一个任务点
            }
            break;
        }
        case SCAN_QR_CODE:{
            static uint32_t lastTimeStamp = 0;
            static uint32_t timeOut = 100000;//100s没扫到码
            static int codeIndex = 0;
            static int code[4] = {0};
            static bool firstEnter = 1;
            if(firstEnter){
                lastTimeStamp = HAL_GetTick();
                firstEnter = 0;
            }
            if (GM65_FrameReady() && codeIndex < 4)
            {
              const char *s = GM65_GetLastCode();
              code[codeIndex++] = atoi(s);                   
              GM65_ClearFrameReady();
              lastTimeStamp = HAL_GetTick();//扫到码了不算
            }
            if(codeIndex >= 4){
                for(int i=0;i<3;i++){//对比这四组数
                    if(code[i] != code[i+1]){
                        codeIndex = 0;//重新回去扫码
                        break;
                    }
                }
                if(codeIndex){//扫码成功
                    if(code[0]/10u == 1){//先一床
                        mapPosFunction[GO_TO_BED_ONE].order = 1;
                        mapPosFunction[GO_TO_BED_THREE].order = 0;
                        if(code[0]%10 == 1){//左边
                            mapPosFunction[GO_TO_BED_ONE].medicine = LEFT_MEDICINE;
                            mapPosFunction[GO_TO_BED_THREE].medicine = RIGHT_MEDICINE;
                        }else if(code[0]%10 == 3){//右边
                            mapPosFunction[GO_TO_BED_ONE].medicine = RIGHT_MEDICINE;
                            mapPosFunction[GO_TO_BED_THREE].medicine = LEFT_MEDICINE;
                        }
                        currentSystemState = GO_TO_BED_ONE;//先去一床
                    }
                    if(code[0]/10u == 3){//先三床
                        mapPosFunction[GO_TO_BED_ONE].order = 0;
                        mapPosFunction[GO_TO_BED_THREE].order = 1;
                        if(code[0]%10 == 1){//左边
                            mapPosFunction[GO_TO_BED_THREE].medicine = LEFT_MEDICINE;
                            mapPosFunction[GO_TO_BED_ONE].medicine = RIGHT_MEDICINE;
                        }else if(code[0]%10 == 3){//右边
                            mapPosFunction[GO_TO_BED_THREE].medicine = RIGHT_MEDICINE;
                            mapPosFunction[GO_TO_BED_ONE].medicine = LEFT_MEDICINE;
                        }
                        currentSystemState = GO_TO_BED_THREE;//先去三床
                    }
                }
            }
            //超时处理
            if(HAL_GetTick() - lastTimeStamp >= timeOut && codeIndex < 4){
                lastTimeStamp = HAL_GetTick();
                codeIndex = 0;
                firstEnter = 1;
                currentSystemState = OPERATE_SCAN_QR_CODE;//去调整车体位置
            }
            break;
        }
        case OPERATE_SCAN_QR_CODE:{//如果扫不到二维码就到这个状态
            static float operate_pos_y_step = 50.0;//每次操作距离
            static float operate_pos_y = 0.0;//累计操作
            static uint8_t operateTimes = 0;//操作次数
            static bool firstEnter = 1;
            if(firstEnter){
                operate_pos_y -= operate_pos_y_step;
                operateTimes++;
                firstEnter = 0;
            }
            ChassisCtrl_MoveTarget(mapPosFunction[GO_TO_NURSE].pos_x,
                                   mapPosFunction[GO_TO_NURSE].pos_y + operate_pos_y,
                                   mapPosFunction[GO_TO_NURSE].heading,
                                   OPS_GetX(),OPS_GetY(),pos_z);
            if(ChassisCtrl_ReachFlag()){
                currentSystemState = SCAN_QR_CODE;//调整后回去扫码
                firstEnter = 1;
            }
            break;
        }

        /*************去一号床***************/
        case GO_TO_BED_ONE:{
            ChassisCtrl_MoveTarget(mapPosFunction[GO_TO_BED_ONE].pos_x,
                                   mapPosFunction[GO_TO_BED_ONE].pos_y,
                                   mapPosFunction[GO_TO_BED_ONE].heading,
                                   OPS_GetX(),OPS_GetY(),pos_z);
            if(ChassisCtrl_ReachFlag()){
                currentSystemState = OPERATE_BED_ONE_LASER_POSITIONING;
            }
            break;
        }
        case OPERATE_BED_ONE_LASER_POSITIONING:{
            currentSystemState = SCAN_BED_ONE_BAR_CODE;
            break;
        }
        case SCAN_BED_ONE_BAR_CODE:{
            static uint32_t lastTimeStamp = 0;
            static uint32_t timeOut = 100000;//100s没扫到码
            static int codeIndex = 0;
            static int code[4] = {0};
            static bool firstEnter = 1;
            if(firstEnter){
                lastTimeStamp = HAL_GetTick();
                firstEnter = 0;
            }
            if (GM65_FrameReady() && codeIndex < 4)
            {
              const char *s = GM65_GetLastCode();
              code[codeIndex++] = atoi(s);                   
              GM65_ClearFrameReady();
              lastTimeStamp = HAL_GetTick();//扫到码了不算
            }
            if(codeIndex >= 4){
                for(int i=0;i<3;i++){//对比这四组数
                    if(code[i] != code[i+1]){
                        codeIndex = 0;//重新回去扫码
                        break;
                    }
                }
                if(codeIndex){
                    mapPosFunction[SCAN_BED_ONE_BAR_CODE].bar_code = code[0];
                    currentSystemState = OPERATE_BED_ONE;
                }
            }
            break;
        }
        case OPERATE_BED_ONE:{
            if(mapPosFunction[GO_TO_BED_ONE].medicine == LEFT_MEDICINE){
                SERVO1_OPEN();
            }else{
                SERVO2_OPEN();
            }
            currentSystemState = PREPARE_TO_BED_THREE;
            break;
        }
        case PREPARE_TO_BED_THREE:{
            ChassisCtrl_MoveTarget(mapPosFunction[PREPARE_TO_BED_THREE].pos_x,
                                   mapPosFunction[PREPARE_TO_BED_THREE].pos_y,
                                   mapPosFunction[PREPARE_TO_BED_THREE].heading,
                                   OPS_GetX(),OPS_GetY(),pos_z);
            if(ChassisCtrl_ReachFlag()){
                if(mapPosFunction[GO_TO_BED_ONE].order){//一号床先行
                    currentSystemState = TRANSIT_BED_ONE_TO_THREE_STEP_ONE;//那么准备去三号床
                }else{//一号床是最后的
                    currentSystemState = PREPARE_TO_BACK_HOME;//那么准备回家
                }
            }
            break;
        }

        /*************去三床***************/
        case GO_TO_BED_THREE:{
            ChassisCtrl_MoveTarget(mapPosFunction[GO_TO_BED_THREE].pos_x,
                                   mapPosFunction[GO_TO_BED_THREE].pos_y,
                                   mapPosFunction[GO_TO_BED_THREE].heading,
                                   OPS_GetX(),OPS_GetY(),pos_z);
            if(ChassisCtrl_ReachFlag()){
                currentSystemState = OPERATE_BED_THREE_LASER_POSITIONING;
            }
            break;
        }
        case OPERATE_BED_THREE_LASER_POSITIONING:{
            currentSystemState = SCAN_BED_THREE_BAR_CODE;
            break;
        }
        case SCAN_BED_THREE_BAR_CODE:{
            static uint32_t lastTimeStamp = 0;
            static uint32_t timeOut = 100000;//100s没扫到码
            static int codeIndex = 0;
            static int code[4] = {0};
            static bool firstEnter = 1;
            if(firstEnter){
                lastTimeStamp = HAL_GetTick();
                firstEnter = 0;
            }
            if (GM65_FrameReady() && codeIndex < 4)
            {
              const char *s = GM65_GetLastCode();
              code[codeIndex++] = atoi(s);                   
              GM65_ClearFrameReady();
              lastTimeStamp = HAL_GetTick();//扫到码了不算
            }
            if(codeIndex >= 4){
                for(int i=0;i<3;i++){//对比这四组数
                    if(code[i] != code[i+1]){
                        codeIndex = 0;//重新回去扫码
                        break;
                    }
                }
                if(codeIndex){
                    mapPosFunction[SCAN_BED_THREE_BAR_CODE].bar_code = code[0];
                    currentSystemState = OPERATE_BED_THREE;
                }
            }
            break;
        }
        case OPERATE_BED_THREE:{
            if(mapPosFunction[GO_TO_BED_THREE].medicine == LEFT_MEDICINE){
                SERVO1_OPEN();
            }else{
                SERVO2_OPEN();
            }
            currentSystemState = PREPARE_TO_BED_ONE;
            break;
        }
        case PREPARE_TO_BED_ONE:{
            ChassisCtrl_MoveTarget(mapPosFunction[PREPARE_TO_BED_ONE].pos_x,
                                   mapPosFunction[PREPARE_TO_BED_ONE].pos_y,
                                   mapPosFunction[PREPARE_TO_BED_ONE].heading,
                                   OPS_GetX(),OPS_GetY(),pos_z);
            if(ChassisCtrl_ReachFlag()){
                if(mapPosFunction[GO_TO_BED_THREE].order){//三号床先行
                    currentSystemState = TRANSIT_BED_THREE_TO_ONE_STEP_ONE;//那么准备去一号床
                }else{//三号床是最后的
                    currentSystemState = PREPARE_TO_BACK_HOME;//那么准备回家
                }
            }
            break;
        }

        /**************从一床到三床***************/
        case TRANSIT_BED_ONE_TO_THREE_STEP_ONE:{
            ChassisCtrl_MoveTarget(mapPosFunction[TRANSIT_BED_ONE_TO_THREE_STEP_ONE].pos_x,
                                   mapPosFunction[TRANSIT_BED_ONE_TO_THREE_STEP_ONE].pos_y,
                                   mapPosFunction[TRANSIT_BED_ONE_TO_THREE_STEP_ONE].heading,
                                   OPS_GetX(),OPS_GetY(),pos_z);
            if(ChassisCtrl_ReachFlag()){
                currentSystemState = TRANSIT_BED_ONE_TO_THREE_STEP_TWO;
            }
            break;
        }
        case TRANSIT_BED_ONE_TO_THREE_STEP_TWO:{
            ChassisCtrl_MoveTarget(mapPosFunction[TRANSIT_BED_ONE_TO_THREE_STEP_TWO].pos_x,
                                   mapPosFunction[TRANSIT_BED_ONE_TO_THREE_STEP_TWO].pos_y,
                                   mapPosFunction[TRANSIT_BED_ONE_TO_THREE_STEP_TWO].heading,
                                   OPS_GetX(),OPS_GetY(),pos_z);
            if(ChassisCtrl_ReachFlag()){
                currentSystemState = TRANSIT_BED_ONE_TO_THREE_STEP_THREE;
            }
            break;
        }
        case TRANSIT_BED_ONE_TO_THREE_STEP_THREE:{
            ChassisCtrl_MoveTarget(mapPosFunction[TRANSIT_BED_ONE_TO_THREE_STEP_THREE].pos_x,
                                   mapPosFunction[TRANSIT_BED_ONE_TO_THREE_STEP_THREE].pos_y,
                                   mapPosFunction[TRANSIT_BED_ONE_TO_THREE_STEP_THREE].heading,
                                   OPS_GetX(),OPS_GetY(),pos_z);
            if(ChassisCtrl_ReachFlag()){
                currentSystemState = TRANSIT_BED_ONE_TO_THREE_STEP_FOUR;
            }
            break;
        }
        case TRANSIT_BED_ONE_TO_THREE_STEP_FOUR:{
            ChassisCtrl_MoveTarget(mapPosFunction[TRANSIT_BED_ONE_TO_THREE_STEP_FOUR].pos_x,
                                   mapPosFunction[TRANSIT_BED_ONE_TO_THREE_STEP_FOUR].pos_y,
                                   mapPosFunction[TRANSIT_BED_ONE_TO_THREE_STEP_FOUR].heading,
                                   OPS_GetX(),OPS_GetY(),pos_z);
            if(ChassisCtrl_ReachFlag()){
                currentSystemState = GO_TO_BED_THREE;
            }
            break;
        }

        /**************从三床到一床***************/
        case TRANSIT_BED_THREE_TO_ONE_STEP_ONE:{
            ChassisCtrl_MoveTarget(mapPosFunction[TRANSIT_BED_THREE_TO_ONE_STEP_ONE].pos_x,
                                   mapPosFunction[TRANSIT_BED_THREE_TO_ONE_STEP_ONE].pos_y,
                                   mapPosFunction[TRANSIT_BED_THREE_TO_ONE_STEP_ONE].heading,
                                   OPS_GetX(),OPS_GetY(),pos_z);
            if(ChassisCtrl_ReachFlag()){
                currentSystemState = TRANSIT_BED_THREE_TO_ONE_STEP_TWO;
            }
            break;
        }
        case TRANSIT_BED_THREE_TO_ONE_STEP_TWO:{
            ChassisCtrl_MoveTarget(mapPosFunction[TRANSIT_BED_THREE_TO_ONE_STEP_TWO].pos_x,
                                   mapPosFunction[TRANSIT_BED_THREE_TO_ONE_STEP_TWO].pos_y,
                                   mapPosFunction[TRANSIT_BED_THREE_TO_ONE_STEP_TWO].heading,
                                   OPS_GetX(),OPS_GetY(),pos_z);
            if(ChassisCtrl_ReachFlag()){
                currentSystemState = TRANSIT_BED_THREE_TO_ONE_STEP_THREE;
            }
            break;
        }
        case TRANSIT_BED_THREE_TO_ONE_STEP_THREE:{
            ChassisCtrl_MoveTarget(mapPosFunction[TRANSIT_BED_THREE_TO_ONE_STEP_THREE].pos_x,
                                   mapPosFunction[TRANSIT_BED_THREE_TO_ONE_STEP_THREE].pos_y,
                                   mapPosFunction[TRANSIT_BED_THREE_TO_ONE_STEP_THREE].heading,
                                   OPS_GetX(),OPS_GetY(),pos_z);
            if(ChassisCtrl_ReachFlag()){
                currentSystemState = TRANSIT_BED_THREE_TO_ONE_STEP_FOUR;
            }
            break;
        }
        case TRANSIT_BED_THREE_TO_ONE_STEP_FOUR:{
            ChassisCtrl_MoveTarget(mapPosFunction[TRANSIT_BED_THREE_TO_ONE_STEP_FOUR].pos_x,
                                   mapPosFunction[TRANSIT_BED_THREE_TO_ONE_STEP_FOUR].pos_y,
                                   mapPosFunction[TRANSIT_BED_THREE_TO_ONE_STEP_FOUR].heading,
                                   OPS_GetX(),OPS_GetY(),pos_z);
            if(ChassisCtrl_ReachFlag()){
                currentSystemState = GO_TO_BED_ONE;
            }
            break;
        }

        //回到原点完成任务
        case PREPARE_TO_BACK_HOME:{
            ChassisCtrl_MoveTarget(mapPosFunction[PREPARE_TO_BACK_HOME].pos_x,
                                   mapPosFunction[PREPARE_TO_BACK_HOME].pos_y,
                                   mapPosFunction[PREPARE_TO_BACK_HOME].heading,
                                   OPS_GetX(),OPS_GetY(),pos_z);
            if(ChassisCtrl_ReachFlag()){
                currentSystemState = BACK_HOME;
            }
            break;
        }
        case BACK_HOME:{
            ChassisCtrl_MoveTarget(mapPosFunction[BACK_HOME].pos_x,
                                   mapPosFunction[BACK_HOME].pos_y,
                                   mapPosFunction[BACK_HOME].heading,
                                   OPS_GetX(),OPS_GetY(),pos_z);
            break;
        }

    }
}

void Nav_Update(void)
{
    FSM_Update();
    OA_Update();
    ChassisCtrl_Update(OPS_GetX(), OPS_GetY(), pos_z);
}
