#include "Gimbal.h"
#include "Global_status.h"
#include "remote_control.h"
#include "Auto_control.h"
#include "BoardLink.h"

#include "User_math.h"

#include "IMU_updata.h"
#include "dm_imu.h"
#include "USB_VirCom.h"
#include "Chassis_omni.h"

Gimbal_t Gimbal;

#define CHASSIS_DECOUPLE_FF_GAIN (1.07f)

static bool ReadyCheck(float pitch_pos)
{
    static int time;
    static int total_time; // 总计时，用于超时强制通过

    Gimbal.pitch_location_set = pitch_pos * RAD_TO_DEG;

    float d_pitch = fabsf(Gimbal.pitch_location_now * DEG_TO_RAD - pitch_pos);

    total_time++;

    if (d_pitch < 0.1)
        time++;
    else
        time = 0;
    if (time < 100 && total_time < 3000) // 最多等3秒，超时强制通过
        return false;
    else
        return true;
}
/*----------------------------------- 初始化 -----------------------------------*/

/**
 * @brief          初始化
 * @param          none
 * @retval         none
 */
void Gimbal_Init()
{
    // 云台电机初始化
    GIMBALMotor_init(GIMBAL_PITCH_MOTOR_TYPE, PITCHMotor);
    /*PID速度环初始化*/
    // 遥控
    // PID_Set(&Gimbal.pitch_speed_pid, 10.0f, 0.0f, 0.0f, 0.0f, 1000000.0f, 1000000.0f);
    // 自瞄
    // PID_Set(&Gimbal.pitch_auto_speed_pid, 1500.0f, 0.0f, 0.0f, 0.0f, 1000000.0f, 1000000.0f);
    /*PID位置环初始化*/
    // 遥控
    PID_Set(&Gimbal.pitch_location_pid, 8.0f, 0.0f, 0.5f, 0.0f, 1718, 1000);
    // 自瞄
    PID_Set(&Gimbal.pitch_auto_location_pid, 5.1f, 0.0f, 0.0f, 0.0f, 1500, 1000);

    /* Yaw 电机位于底盘板；云台板在此初始化姿态位置外环。 */
    PID_Set(&Gimbal.yaw_speed_pid, 500.0f, 0.0f, 10.0f, 0.0f,
            GIMBALMOTOR_MAX_CURRENT, GIMBALMOTOR_MAX_CURRENT);
    PID_Set(&Gimbal.yaw_auto_speed_pid, 200.0f, 0.0f, 50.0f, 0.0f,
            GIMBALMOTOR_MAX_CURRENT, GIMBALMOTOR_MAX_CURRENT);
    PID_Set(&Gimbal.yaw_location_pid, 14.0f, 0.0f, 0.0f, 0.0f,
            GIMBALMOTOR_MAX_CURRENT, 100);
    PID_Set(&Gimbal.yaw_auto_location_pid, 15.0f, 0.03f, 1.0f, 0.0f,
            GIMBALMOTOR_MAX_CURRENT, 100);

    // 上电进入纠偏状态，等待云台到位
    Gimbal.State = RIGHTING;
}

/*--------------------------------- 状态量更新 ---------------------------------*/

/**
 * @brief          控制量更新（包括状态量和目标量）
 * @param          none
 * @retval         none
 */
void Gimbal_Updater()
{
    /*------状态量更新------*/
    // 速度
    Gimbal.yaw_speed_now = cosf(DEG_TO_RAD * GIMBAL_IMU_DATA.pitch) * GIMBAL_IMU_DATA.gyro[2]
                         - sinf(DEG_TO_RAD * GIMBAL_IMU_DATA.pitch) * GIMBAL_IMU_DATA.gyro[0];
    Gimbal.pitch_speed_now = -(GIMBAL_IMU_DATA.gyro[1]);
    // 位置
    static float last_yaw;
    static uint8_t yaw_initialized;
    if (!yaw_initialized)
    {
        last_yaw = GIMBAL_IMU_DATA.yaw;
        yaw_initialized = 1U;
    }

    float yaw_delta = GIMBAL_IMU_DATA.yaw - last_yaw;
    if (yaw_delta > 180.0f)
        yaw_delta -= 360.0f;
    else if (yaw_delta < -180.0f)
        yaw_delta += 360.0f;

    Gimbal.yaw_location_now += yaw_delta;
    last_yaw = GIMBAL_IMU_DATA.yaw;
    Gimbal.pitch_location_now = -GIMBAL_IMU_DATA.pitch;
    /*------目标量更新------*/
    Gimbal.yaw_location_set = Global.Gimbal.input.yaw;
    Gimbal.pitch_location_set = Global.Gimbal.input.pitch;
}

/*----------------------------------- 解算 -------------------------------------*/

/**
 * @brief          控制量解算
 * @param          none
 * @retval         none
 */
#define PITCH_VEL_FF_GAIN (0.65f)  // pitch速度前馈增益: 1.0=直接跟随目标pitch角速度，单位匹配时取小
#define PITCH_ACC_FF_GAIN (0.013f) // pitch加速度前馈增益 增大可更快跟上但易超
#define PITCH_VEL_FF_LIMIT (0.20f)
#define PITCH_ACC_FF_LIMIT (0.15f)
#define PITCH_FF_TOTAL_LIMIT (0.30f)

/**
 * @brief 计算 Yaw 姿态位置外环，并生成发送到底盘板的速度目标。
 */
void Gimbal_YawCalculater(void)
{
    static enum control_mode_e last_control_mode = LOCK;
    static uint8_t last_auto_active;
    static float body_gyro_filtered;
    static float spin_gyro_filtered;

    if (last_control_mode == LOCK && Global.Control.mode != LOCK)
    {
        Global.Gimbal.input.yaw = Gimbal.yaw_location_now;
        Gimbal.yaw_location_set = Gimbal.yaw_location_now;
    }
    last_control_mode = Global.Control.mode;

    if ((Global.Auto.input.Auto_control_online <= 0 ||
         Global.Auto.mode == NONE ||
         Global.Auto.input.control_mode == 0) &&
        (Global.Gimbal.mode == NORMAL || Global.Gimbal.mode == SHOOT))
    {
        /* 从自瞄切回手动时同步当前位置，防止目标跳变。 */
        if (last_auto_active)
        {
            Gimbal.yaw_location_set = Gimbal.yaw_location_now;
            Global.Gimbal.input.yaw = Gimbal.yaw_location_set;
        }
        last_auto_active = 0U;

        Gimbal.yaw_speed_set =
            PID_Cal(&Gimbal.yaw_location_pid,
                    Gimbal.yaw_location_now,
                    Gimbal.yaw_location_set) * DEG_TO_RAD -BoardLink.body_gyro_z;

        if (Global.Chassis.mode == FLOW)
        {
            body_gyro_filtered += 0.15f * (BoardLink.body_gyro_z - body_gyro_filtered);
            Gimbal.yaw_speed_set -= CHASSIS_DECOUPLE_FF_GAIN * body_gyro_filtered;
        }
        else if (Global.Chassis.mode == SPIN_P || Global.Chassis.mode == SPIN_N)
        {
            spin_gyro_filtered += 0.28f * (BoardLink.body_gyro_z - spin_gyro_filtered);
            Gimbal.yaw_speed_set -= spin_gyro_filtered;
        }
        else
        {
            spin_gyro_filtered = 0.0f;
        }
    }
    else
    {
        Gimbal.yaw_speed_set =
            PID_Cal(&Gimbal.yaw_auto_location_pid,
                    Gimbal.yaw_location_now,
                    Gimbal.yaw_location_set) * DEG_TO_RAD +
            Global.Auto.input.yaw_ff - BoardLink.body_gyro_z;

        if (Global.Auto.input.control_mode == 0)
            last_auto_active = 1U;
        else
            last_auto_active = 0U;
    }

}

/**
 * @brief 在 Yaw 外环计算后执行锁定保护和速度限幅。
 */
static void Gimbal_YawProtect(void)
{
    if (Global.Control.mode == LOCK)
        Gimbal.yaw_speed_set = 0.0f;

    if (Gimbal.yaw_speed_set > BOARD_LINK_YAW_SPEED_LIMIT_RAD_S)
        Gimbal.yaw_speed_set = BOARD_LINK_YAW_SPEED_LIMIT_RAD_S;
    if (Gimbal.yaw_speed_set < -BOARD_LINK_YAW_SPEED_LIMIT_RAD_S)
        Gimbal.yaw_speed_set = -BOARD_LINK_YAW_SPEED_LIMIT_RAD_S;
}

void Gimbal_Calculater()
{
    static uint8_t last_auto_active = 0;
    if ((Global.Auto.input.Auto_control_online <= 0 || Global.Auto.mode == NONE || Global.Auto.input.control_mode == 0) && (Global.Gimbal.mode == NORMAL || Global.Gimbal.mode == SHOOT))
    {

        if (last_auto_active)
        {
            Gimbal.pitch_location_set = -normalize_angle(GIMBALMotor_get_data(PITCHMotor).motor_data.para.pos) * RAD_TO_DEG;
            Global.Gimbal.input.pitch = Gimbal.pitch_location_set;
        }
        last_auto_active = 0;
        Gimbal.position[0] = -Gimbal.pitch_location_set * DEG_TO_RAD;
        // Gimbal.pitch_speed_set = PID_Cal(&Gimbal.pitch_location_pid, Gimbal.pitch_location_now, Gimbal.pitch_location_set) * DEG_TO_RAD;

    }
    else
    {
        // 自瞄
        // static float pitch_ff_filt = 0;
        // pitch_ff_filt += 0.5f * (Global.Auto.input.pitch_ff - pitch_ff_filt);
        static float pitch_ff_filt = 0;
        pitch_ff_filt += 0.5f * (Global.Auto.input.pitch_ff - pitch_ff_filt);

        float vel_ff = PITCH_VEL_FF_GAIN * pitch_ff_filt;
        float acc_ff = PITCH_ACC_FF_GAIN * Global.Auto.input.pitch_acc_ff;
        if (vel_ff > PITCH_VEL_FF_LIMIT)
            vel_ff = PITCH_VEL_FF_LIMIT;
        if (vel_ff < -PITCH_VEL_FF_LIMIT)
            vel_ff = -PITCH_VEL_FF_LIMIT;
        if (acc_ff > PITCH_ACC_FF_LIMIT)
            acc_ff = PITCH_ACC_FF_LIMIT;
        if (acc_ff < -PITCH_ACC_FF_LIMIT)
            acc_ff = -PITCH_ACC_FF_LIMIT;

        float ff_total = vel_ff + acc_ff;
        if (ff_total > PITCH_FF_TOTAL_LIMIT)
            ff_total = PITCH_FF_TOTAL_LIMIT;
        if (ff_total < -PITCH_FF_TOTAL_LIMIT)
            ff_total = -PITCH_FF_TOTAL_LIMIT;

        Gimbal.position[0] = normalize_angle(GIMBALMotor_get_data(PITCHMotor).motor_data.para.pos) + (Gimbal.pitch_location_set - Gimbal.pitch_location_now * DEG_TO_RAD) + ff_total;
        // Gimbal.position[0] = Global.Auto.input.shoot_pitch;

        // pitch轴重力补偿
        // Gimbal.force[0] += -(0.45 * cosf(DEG_TO_RAD * Gimbal.pitch_location_now));

        if (Global.Auto.input.control_mode == 0)
        {
            last_auto_active = 1;
        }
        else
        {
            last_auto_active = 0;
        }
    }
}

/*----------------------------------- 控制 -------------------------------------*/

/**
 * @brief          电流值设置
 * @param          none
 * @retval         none
 */
void Gimbal_Controller()
{
    if (Global.Control.mode != LOCK)
    {
        GIMBALMotor_set(PITCHMotor, -Gimbal.position[0], 0, 0.0f, 40.0f, 2.0f);
    }
    else
    {
        GIMBALMotor_set(PITCHMotor, 0, 0, 0, 0, 0);
    }
}

/*----------------------------------- 任务 -------------------------------------*/

/**
 * @brief          云台任务
 * @param          none
 * @retval         none
 */
void Gimbal_Tasks(void)
{
#if (USE_GIMBAL != 0)

    // 云台数据更新
    Gimbal_Updater();

    /* 与 fold 一致，Yaw 姿态外环由 Gimbal 模块统一解算。 */
    Gimbal_YawCalculater();
    Gimbal_YawProtect();

    if (Gimbal.State != NORMALLY)
    {
        // 上电纠偏阶段：驱动云台回到零位
        if (ReadyCheck(0))
        {
            Gimbal.State = NORMALLY;
            // 同步全局输入为当前位置，防止切换到正常控制时跳变
            Global.Gimbal.input.pitch = Gimbal.pitch_location_now;
        }
        Gimbal_Calculater();
        // 纠偏阶段强制输出，不受 LOCK 模式影响
        GIMBALMotor_set(PITCHMotor, Gimbal.position[0], 0, 0, 50.0f, 1.5f);
    }
    else
    {
        // 正常控制
        Gimbal_Calculater();
        Gimbal_Controller();
    }
#endif
}

/*--------------------------------- 目标值设置 ---------------------------------*/
/**
 * @brief 设置云台PITCHI轴角度
 *
 * @param angle 云台PITCHI轴角度
 */
void Gimbal_SetPitchAngle(float angle)
{
    if (angle < PITCHI_MIN_ANGLE)
        angle = PITCHI_MIN_ANGLE;
    if (angle > PITCHI_MAX_ANGLE)
        angle = PITCHI_MAX_ANGLE;
    Global.Gimbal.input.pitch = angle;
}

/**
 * @brief 设置云台YAW轴角度
 *
 * @param angle 云台YAW轴角度
 */
void Gimbal_SetYawAngle(float angle)
{
    Global.Gimbal.input.yaw = angle;
}
