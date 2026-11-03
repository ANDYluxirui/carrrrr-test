/**
  ******************************************************************************
  * @file    chassis.h
  * @brief   差速底盘：运动学正逆解 + 里程计 + 航向环 + 任务序列
  ******************************************************************************
  */
#ifndef __CHASSIS_H
#define __CHASSIS_H

#include <stdint.h>

typedef struct {
    float x;        /* mm，向右为正 */
    float y;        /* mm，向前为正 */
    float theta;    /* rad，逆时针为正，0 = 车头朝 +y */
} Pose_t;

typedef enum {
    MISSION_IDLE = 0,
    MISSION_DRIVE,
    MISSION_SETTLE,
    MISSION_TURN,
    MISSION_TURN_SETTLE,   /* 转到位后停一下，再复核角度 */
    MISSION_ALIGN,         /* 定点导航：先转向目标方位 */
    MISSION_GOTO,          /* 定点导航：直行到目标 */
    MISSION_DONE
} MissionState_t;

extern Pose_t         g_pose;         /* 当前位姿 */
extern float          g_v_meas;       /* 实测线速度 mm/s */
extern float          g_w_meas;       /* 实测角速度 rad/s */
extern float          g_odo_dist;     /* 累计路径长度 mm */
extern float          g_target_l;     /* 逆解得到的左右轮目标速度 */
extern float          g_target_r;
extern MissionState_t g_mission;

void Chassis_Init(void);
void Chassis_Update(void);                  /* 每 20ms 调一次 */

/* 正解：把左右轮速换算成车体速度 */
void Chassis_Forward(float vl, float vr, float *v, float *w);

/* 逆解：给定车体线速度/角速度，算出左右轮目标速度 */
void Chassis_SetVel(float v, float w);

void Chassis_Stop(void);
void Chassis_ResetPose(void);

/* 任务：走正方形 */
void Chassis_StartSquare(float side_mm);
void Chassis_StartStraight(float dist_mm);
void Chassis_MissionUpdate(void);           /* 每 20ms 调一次 */

/* 定点导航：走到绝对坐标 (tx, ty)，单位 mm */
void Chassis_StartGoto(float tx, float ty);

#endif /* __CHASSIS_H */
