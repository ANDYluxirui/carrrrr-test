/**
  ******************************************************************************
  * @file    chassis.c
  * @brief   差速底盘：运动学 + 里程计 + 航向环 + 任务序列
  *
  *   正解（轮速 → 车体速度）：
  *       v = (vL + vR) / 2
  *       w = (vR − vL) / L
  *
  *   逆解（车体速度 → 轮速）：
  *       vL = v − w·L/2
  *       vR = v + w·L/2
  *
  *   里程计用「中点积分」：把这一小段位移按行进方向的中点角度投影到 x/y，
  *   走圆弧时比用步末角度更准。
  ******************************************************************************
  */
#include "chassis.h"
#include "app_config.h"
#include "encoder.h"
#include "mpu6050.h"
#include "main.h"
#include <math.h>

#ifndef M_PI
#define M_PI  3.14159265358979f
#endif

Pose_t         g_pose      = {0.0f, 0.0f, 0.0f};
float          g_v_meas    = 0.0f;
float          g_w_meas    = 0.0f;
float          g_odo_dist  = 0.0f;    /* 累计路径长度（带符号）*/
float          g_target_l  = 0.0f;
float          g_target_r  = 0.0f;
MissionState_t g_mission   = MISSION_IDLE;

static int32_t s_last_l = 0, s_last_r = 0;
static float   s_theta_enc  = 0.0f;
static float   s_theta_gyro = 0.0f;

#define MM_PER_COUNT   (3.14159265f * WHEEL_DIAMETER_MM / ENCODER_CPR)

/* ---- 任务状态机 ---- */
static uint8_t  s_is_square   = 0;
static uint8_t  s_seg         = 0;      /* 当前是第几段（1~4）*/
static float    s_seg_target  = 0.0f;   /* 本段要走的距离 */
static float    s_seg_start   = 0.0f;   /* 本段开始时的 g_odo_dist */
static float    s_head_target = 0.0f;   /* 目标航向（绝对）*/
static uint32_t s_t_settle    = 0;
static float    s_tx = 0.0f, s_ty = 0.0f;   /* 导航目标点 */

/* ------------------------------------------------------------------ */
static float wrap_pi(float a)
{
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

static float clampf(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

/* ------------------------------------------------------------------ */
void Chassis_Init(void)
{
    g_pose.x = g_pose.y = g_pose.theta = 0.0f;
    g_v_meas = g_w_meas = g_odo_dist = 0.0f;
    g_target_l = g_target_r = 0.0f;
    s_theta_enc = s_theta_gyro = 0.0f;
    s_last_l = g_cnt_l;
    s_last_r = g_cnt_r;
    g_mission = MISSION_IDLE;
}

void Chassis_ResetPose(void)
{
    g_pose.x = g_pose.y = g_pose.theta = 0.0f;
    s_theta_enc = s_theta_gyro = 0.0f;
    g_odo_dist = 0.0f;
    s_last_l = g_cnt_l;
    s_last_r = g_cnt_r;
}

/* ------------------------------------------------------------------ */
void Chassis_Forward(float vl, float vr, float *v, float *w)
{
    *v = 0.5f * (vl + vr);
    *w = (vr - vl) / WHEEL_BASE_MM;
}

void Chassis_SetVel(float v, float w)
{
    float half = 0.5f * WHEEL_BASE_MM * w;

    g_target_l = v - half;
    g_target_r = v + half;
}

void Chassis_Stop(void)
{
    g_target_l = 0.0f;
    g_target_r = 0.0f;
}

/* ------------------------------------------------------------------ */
/**
  * @brief 每 20ms 调一次：更新里程计、融合航向
  */
void Chassis_Update(void)
{
    int32_t cl = g_cnt_l;
    int32_t cr = g_cnt_r;

    float dL = (float)(cl - s_last_l) * MM_PER_COUNT;
    float dR = (float)(cr - s_last_r) * MM_PER_COUNT;
    s_last_l = cl;
    s_last_r = cr;

    float d       = 0.5f * (dL + dR);
    float dth_enc = (dR - dL) / WHEEL_BASE_MM;

    g_odo_dist += d;
    s_theta_enc += dth_enc;

    /* 用轮速反推车体速度，供遥测 */
    Chassis_Forward(dL / ODO_DT_S, dR / ODO_DT_S, &g_v_meas, &g_w_meas);

    float th0 = g_pose.theta;

    if (g_mpu_ok) {
        /* 互补滤波：短时间信陀螺仪（准但会漂），长时间信编码器（会打滑但不漂）*/
        s_theta_gyro += MPU6050_ReadGyroZ() * DEG2RAD * ODO_DT_S;
        g_pose.theta  = COMP_ALPHA * s_theta_gyro
                      + (1.0f - COMP_ALPHA) * s_theta_enc;
    } else {
        g_pose.theta = s_theta_enc;
    }

    /* 中点积分的补偿：这一小段是按 θ 的中间值走的 */
    float th_mid = 0.5f * (th0 + g_pose.theta);
    g_pose.x += d * cosf(th_mid);
    g_pose.y += d * sinf(th_mid);
}

/* ==================================================================
 * 任务序列
 * ================================================================== */
void Chassis_StartStraight(float dist_mm)
{
    s_is_square  = 0;
    s_seg        = 1;
    s_seg_target = dist_mm;
    s_seg_start  = g_odo_dist;
    s_head_target = g_pose.theta;
    g_mission = MISSION_DRIVE;
}

void Chassis_StartSquare(float side_mm)
{
    Chassis_ResetPose();               /* 原点 = 出发位置 */
    s_is_square  = 1;
    s_seg        = 1;
    s_seg_target = side_mm;
    s_seg_start  = 0.0f;
    s_head_target = g_pose.theta;      /* 以出发时的车头方向为 0 度 */
    g_mission = MISSION_DRIVE;
}

/**
  * @brief 任务推进，每 20ms 调一次
  */
void Chassis_MissionUpdate(void)
{
    float err, w;

    switch (g_mission) {

    case MISSION_DRIVE:
        if (fabsf(g_odo_dist - s_seg_start) >= s_seg_target) {
            Chassis_Stop();
            s_t_settle = HAL_GetTick();
            g_mission  = MISSION_SETTLE;
            break;
        }
        /* 直行 + 航向保持 */
        err = wrap_pi(s_head_target - g_pose.theta);
        w   = clampf(HEADING_KP * err, -HEADING_W_MAX, HEADING_W_MAX);
        Chassis_SetVel(MISSION_SPEED_MM_S, w);
        break;

    case MISSION_SETTLE:
        Chassis_Stop();
        if (HAL_GetTick() - s_t_settle < 300u) break;

        if (!s_is_square || s_seg >= 4) {
#if SQUARE_THEN_GOTO
            /* 方形跑完 → 用定位反馈回到精确原点 */
            if (s_is_square) { g_mission = MISSION_ALIGN; s_tx = 0.0f; s_ty = 0.0f; break; }
            s_is_square = 0;              /* 防止回头再来一次 */
#endif
            g_mission = MISSION_DONE;
            break;
        }
        s_head_target = wrap_pi(s_head_target + (float)M_PI * 0.5f);
        g_mission = MISSION_TURN;
        break;

    case MISSION_TURN:
        err = wrap_pi(s_head_target - g_pose.theta);
        if (fabsf(err) < TURN_TOL_RAD) {
            Chassis_Stop();
            s_t_settle = HAL_GetTick();
            g_mission  = MISSION_TURN_SETTLE;   /* 先停稳再复核 */
            break;
        }
        w = clampf(TURN_KP * err, -TURN_W_MAX, TURN_W_MAX);
        Chassis_SetVel(0.0f, w);        /* 原地转 */
        break;

    case MISSION_TURN_SETTLE:
        /* 停车后惯性还会让车再转一点，等 250ms 停稳再量一次角度。
         * 如果超出容差就回去继续微调 —— 这样每次都是"停稳后再判定"，
         * 不会因为冲过头而累积误差。 */
        Chassis_Stop();
        if (HAL_GetTick() - s_t_settle < 250u) break;

        err = wrap_pi(s_head_target - g_pose.theta);
        if (fabsf(err) > TURN_TOL_RAD * 1.5f) {
            g_mission = MISSION_TURN;           /* 还差，再微调一次 */
            break;
        }
        s_seg++;
        s_seg_start = g_odo_dist;
        g_mission   = MISSION_DRIVE;            /* 转准了，走下一段 */
        break;

    case MISSION_ALIGN:
        /* 先把车头转到指向目标的方位，再走 */
        {
            float dx = s_tx - g_pose.x;
            float dy = s_ty - g_pose.y;
            err = wrap_pi(atan2f(dx, dy) - g_pose.theta);

            if (fabsf(err) < TURN_TOL_RAD * 2.0f) {
                g_mission = MISSION_GOTO;
                break;
            }
            w = clampf(TURN_KP * err, -TURN_W_MAX, TURN_W_MAX);
            Chassis_SetVel(0.0f, w);
        }
        break;

    case MISSION_GOTO:
        {
            float dx = s_tx - g_pose.x;
            float dy = s_ty - g_pose.y;
            float dist = sqrtf(dx * dx + dy * dy);
            float v;

            if (dist < GOTO_TOL_MM) {          /* 到点 */
                Chassis_Stop();
                g_mission = MISSION_DONE;
                break;
            }

            /* 方位角每周期重算，边走边修正 */
            err = wrap_pi(atan2f(dx, dy) - g_pose.theta);

            v = GOTO_SPEED_MM_S;
            if (dist < GOTO_SLOW_MM) v = dist * 2.0f;   /* 接近时减速，停得准 */
            if (v < 80.0f) v = 80.0f;                   /* 但别慢到走不动 */

            w = clampf(HEADING_KP * err, -HEADING_W_MAX, HEADING_W_MAX);
            Chassis_SetVel(v, w);
        }
        break;

    case MISSION_IDLE:
    case MISSION_DONE:
    default:
        Chassis_Stop();
        break;
    }
}

/**
  * @brief 定点导航：走到绝对坐标 (tx, ty)
  * @note  会先转向目标方位再直行，接近时自动减速
  */
void Chassis_StartGoto(float tx, float ty)
{
    s_tx = tx;
    s_ty = ty;
    g_mission = MISSION_ALIGN;
}
