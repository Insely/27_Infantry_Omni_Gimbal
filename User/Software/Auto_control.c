#include "Auto_control.h"
#include "Global_status.h"
#include "Gimbal.h"

#include "IMU_updata.h"
#include "referee_system.h"
#include "UART_data_txrx.h"

#include "CRC8_CRC16.h"
#include "string.h"

STM32_data_t toMINIPC;    // 发送给 MiniPC 的云台姿态、角速度和裁判系统数据
MINIPC_data_t fromMINIPC; // MiniPC 下发的控制模式、目标角度及前馈数据

uint8_t data[128];    // 串口发送缓冲区，存放打包后的 STM32_data_t
uint8_t rx_data[100]; // 接收缓冲区（当前文件未使用，实际接收使用 UART1_data.rev_data）

// 按协议结构体布局直接拷贝；此处不检查长度或 CRC，调用方需保证长度和数据有效性。
void decodeMINIPCdata(MINIPC_data_t *target, unsigned char buff[], unsigned int len)
{
    memcpy(target, buff, len);
}

// 将发送结构体按内存布局拷贝到字节缓冲区，两端需保持字段布局及字节序一致。
int encodeSTM32(STM32_data_t *target, unsigned char tx_buff[], unsigned int len)
{
    memcpy(tx_buff, target, len);
    return 0;
}

// 组装并发送一帧云台状态数据，供 MiniPC 进行自瞄计算。
void STM32_to_MINIPC()
{
    toMINIPC.header[0] = 'S';
    toMINIPC.header[1] = 'P';
    toMINIPC.mode = MODE_AUTO_AIM; // 当前发送模式固定为自瞄
    // IMU 欧拉角由度转为协议要求的弧度；pitch 取反以匹配视觉侧正方向。
    toMINIPC.yaw = degree2rad(dm_imu_gimbal.yaw);
    toMINIPC.pitch = degree2rad(-dm_imu_gimbal.pitch);
    // 根据当前俯仰角投影 IMU 的 X/Z 轴角速度，得到 yaw 角速度（rad/s）。
    toMINIPC.yaw_vel = (cos(dm_imu_gimbal.pitch * DEG_TO_RAD) * dm_imu_gimbal.gyro[2] - sin(dm_imu_gimbal.pitch * DEG_TO_RAD) * dm_imu_gimbal.gyro[0]);
    toMINIPC.pitch_vel = -dm_imu_gimbal.gyro[1]; // pitch 角速度同步调整正方向（rad/s）
    // 四元数沿用 IMU 原始分量顺序。
    toMINIPC.q[0] = dm_imu_gimbal.q[0];
    toMINIPC.q[1] = dm_imu_gimbal.q[1];
    toMINIPC.q[2] = dm_imu_gimbal.q[2];
    toMINIPC.q[3] = dm_imu_gimbal.q[3];
    toMINIPC.bullet_speed = Referee_data.Initial_SPEED; // 裁判系统反馈的弹丸初速度（m/s）
    toMINIPC.bullet_count = Referee_data.Launching_Frequency; // 当前填入发射频率，并非累计发射数量
    // 用 crc16 字段相对帧起始地址的偏移确定校验长度，不包含 CRC 字段本身。
    int len = (uint8_t *)&toMINIPC.crc16 - (uint8_t *)&toMINIPC;
    toMINIPC.crc16 = get_CRC16_check_sum(toMINIPC.header, len, 0xFFFF);
    encodeSTM32(&toMINIPC, data, sizeof(STM32_data_t));
    UART_SendData(UART1_data, data, sizeof(STM32_data_t));   // 通过 UART1 发送完整协议帧
}

// 接收后更新自瞄目标：处理零值数据、转换 yaw 坐标，并过滤目标突变。
void MINIPC_to_STM32(void)
{
    float yaw_error;   // 视觉目标 yaw 与当前 IMU yaw 的角度差（度）

    // 跨次调用保留滤波状态；比较基准是上一次接受的目标，而非上一次收到的目标。
    static float    last_shoot_yaw   = 0; // 上次接受的云台绝对 yaw 目标（度）
    static float    last_shoot_pitch = 0; // 上次接受的 pitch 目标（rad）
    static uint8_t  target_init  = 0; // 是否已建立目标比较基准
    static uint16_t reject_count = 0; // 未更新目标的累计次数，包含跳变拒绝和控制有效时的零值数据
    static uint16_t zero_count   = 0; // 连续收到 yaw、pitch 同时为零的次数

    // 跳变阈值分别使用度和弧度；计数阈值按本函数调用次数累计，并非毫秒。
    const float    YAW_JUMP_MAX_DEG = 12.0f; // 相对上次接受目标的 yaw 最大变化量（度）
    const float    PITCH_JUMP_MAX   = 0.26f; // pitch 最大变化量（rad，约 14.9 度）
    const uint16_t REJECT_MAX       = 50; // 累计拒绝达到此值后，允许下一次非零目标重新建立基准
    const uint16_t ZERO_MAX         = 10; // 连续零值达到此次数后退出视觉控制

    Global.Auto.input.control_mode = fromMINIPC.mode;

    // 将双零角度视为无效目标；短暂出现时保留旧目标，连续达到阈值才退出控制。
    if (fromMINIPC.pitch == 0 && fromMINIPC.yaw == 0)
    {
        zero_count++;
        if (zero_count >= ZERO_MAX)
            Global.Auto.input.control_mode = CTRL_NO_CONTROL;
    }
    else
    {
        zero_count = 0;
    }

    if (Global.Auto.input.control_mode != CTRL_NO_CONTROL)
    {
        // 只有非零目标才参与角度换算和跳变判断。
        uint8_t is_zero_frame = (fromMINIPC.pitch == 0 && fromMINIPC.yaw == 0);

        if (!is_zero_frame)
        {
            // 先统一为度，再修正跨越 +/-180 度时的角度差，取较短转动方向。
            yaw_error = rad2degree(fromMINIPC.yaw) - dm_imu_gimbal.yaw;
            if (yaw_error > 180.0f)  yaw_error -= 360.0f;
            if (yaw_error < -180.0f) yaw_error += 360.0f;
            // 将 IMU 坐标下的角度差叠加到当前云台 yaw 位置，锁定本次绝对目标。
            // pitch 保留视觉协议的弧度单位，后续 Auto_Control 直接使用这两个目标。
            float new_shoot_yaw   = Gimbal.yaw_location_now + yaw_error;
            float new_shoot_pitch = fromMINIPC.pitch;

            // 首次进入控制或退出后重新进入时，以当前目标初始化，使首个非零目标直接通过。
            if (!target_init)
            {
                last_shoot_yaw   = new_shoot_yaw;
                last_shoot_pitch = new_shoot_pitch;
                target_init = 1;
            }

            // 分别计算两个轴相对已接受目标的变化量。
            float d_yaw   = fabsf(new_shoot_yaw   - last_shoot_yaw);
            float d_pitch = fabsf(new_shoot_pitch - last_shoot_pitch);

            // 两轴变化均在阈值内才接受；拒绝次数过多时放行，避免目标切换后一直无法更新。
            if ((d_yaw <= YAW_JUMP_MAX_DEG && d_pitch <= PITCH_JUMP_MAX) || reject_count >= REJECT_MAX)
            {
                Global.Auto.input.shoot_yaw   = new_shoot_yaw;
                Global.Auto.input.shoot_pitch = new_shoot_pitch;
                last_shoot_yaw   = new_shoot_yaw;
                last_shoot_pitch = new_shoot_pitch;
                reject_count = 0;

                // 仅在目标被接受时同步更新速度、加速度前馈（rad/s、rad/s^2）。
                Global.Auto.input.yaw_ff       = fromMINIPC.yaw_vel;
                Global.Auto.input.pitch_ff     = fromMINIPC.pitch_vel;
                Global.Auto.input.yaw_acc_ff   = fromMINIPC.yaw_acc;
                Global.Auto.input.pitch_acc_ff = fromMINIPC.pitch_acc;
            }
            else
            {
                reject_count++; // 跳变过大：保持之前的目标和前馈，仅累计拒绝次数
            }
        }
        else
        {
            reject_count++; // 尚未达到零值退出阈值，保持旧目标并累计未更新次数
        }
    }
    else
    {
        // 退出视觉控制时清除比较基准和拒绝计数，便于下次重新捕获目标。
        target_init  = 0;
        reject_count = 0;
    }
}

// 控制周期中将已接受的目标写入云台输入；此处不再重复叠加 yaw 误差。
void Auto_Control()
{
    if (Global.Auto.input.control_mode != CTRL_NO_CONTROL)
    {
        // yaw 使用绝对目标（度），pitch 使用弧度；俯仰限幅由 Gimbal_SetPitchAngle 完成。
        Gimbal_SetYawAngle(Global.Auto.input.shoot_yaw);
        Gimbal_SetPitchAngle(Global.Auto.input.shoot_pitch);
    }
}
