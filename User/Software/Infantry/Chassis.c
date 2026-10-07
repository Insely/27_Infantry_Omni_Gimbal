#include "robot_param.h"
#if ROBOT_TYPE == ROBOT_INFANTRY
#include "Chassis.h"
#include "Global_status.h"
#include "Gimbal.h"
#include "BoardLink.h"
#include "pid.h"
#include <math.h>

#define PI_F 3.14159265358979323846f
#define DEG_TO_RAD_F (PI_F / 180.0f)
#define RPM_TO_RAD_S_F (PI_F / 30.0f)

/* Final chassis-frame targets: m/s, m/s, rad/s. */
float X_speed, Y_speed, R_speed;
static pid_t chassis_follow_pid;
static float relative_angle, feedforward_angle;

static float AngleLimit(float angle)
{
    angle = fmodf(angle + PI_F, 2.0f * PI_F);
    if (angle < 0)
        angle += 2.0f * PI_F;
    return angle - PI_F;
}

void Chassis_Init(void)
{
    Global.Chassis.mode = FLOW;
    Global.Chassis.input.x = Global.Chassis.input.y = Global.Chassis.input.r = 0;
    X_speed = Y_speed = R_speed = 0;
    relative_angle = feedforward_angle = 0;
#if CHASSIS_TYPE == 0
    PID_Set(&chassis_follow_pid, 5.0f, 0, 0, 0, 200, 40);
#else
    PID_Set(&chassis_follow_pid, 10.0f, 0, 1.0f, 0, 200, 40);
#endif
}

void Chassis_Tasks(void)
{
    float raw = BoardLink.yaw_angle_cnt * DEG_TO_RAD_F;
    X_speed = Y_speed = R_speed = 0;
    Global.Chassis.input.r = 0;
    feedforward_angle = 0;
    if (!isfinite(raw))
        return;

    // 多圈转单圈
    relative_angle = AngleLimit(raw);

    if (CHASSIS_TYPE == 2 || Global.Control.mode == LOCK || Gimbal.State != NORMALLY)
    {
        // 清除积分，防止积分饱和
        PID_Reset(&chassis_follow_pid, relative_angle, 0);
        return;
    }
    switch (Global.Chassis.mode)
    {
    case FLOW:
        R_speed = -PID_Cal(&chassis_follow_pid, relative_angle, 0);
        break;
    case SPIN_P:
    case SPIN_N:
#if CHASSIS_TYPE == 0 // 舵轮底盘R_speed
        R_speed = 80.0f * RPM_TO_RAD_S_F;
#else                 // 全向轮底盘R_speed
        R_speed = 60.0f * RPM_TO_RAD_S_F;
#endif
        if (Global.Chassis.mode == SPIN_N)
            R_speed = -R_speed;
#if CHASSIS_TYPE == 0
        feedforward_angle = 0.03f * R_speed;
#endif
        PID_Reset(&chassis_follow_pid, relative_angle, 0);
        break;
    case NO_FOLLOW:
        PID_Reset(&chassis_follow_pid, relative_angle, 0);
        break;
    default:
        return;
    }
    float Vx = Global.Chassis.input.x, Vy = Global.Chassis.input.y;
#if CHASSIS_TYPE == 0
    float beta = relative_angle - feedforward_angle;
    X_speed = x * cosf(beta) + y * sinf(beta);
    Y_speed = -x * sinf(beta) + y * cosf(beta);
#else
    /* The original 0.0001 * measured chassis W_now compensation cannot move
       here: 0x107 yaw_spd is gimbal angular velocity, not chassis wheel odometry. */
    X_speed = Vx * cosf(relative_angle) - Vy * sinf(relative_angle);
    Y_speed = Vx * sinf(relative_angle) + Vy * cosf(relative_angle);
#endif
    Global.Chassis.input.r = R_speed;
}
void Chassis_SetX(float x) { Global.Chassis.input.x = x; }
void Chassis_SetY(float y) { Global.Chassis.input.y = y; }
void Chassis_SetR(float r) { Global.Chassis.input.r = r; }
void Chassis_SetAccel(float acc) { (void)acc; /* No acceleration field in BoardLink. */ }
#endif
