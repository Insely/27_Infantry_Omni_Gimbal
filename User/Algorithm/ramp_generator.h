#ifndef RAMP_GENERATOR_H
#define RAMP_GENERATOR_H

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 斜坡发生器模块
     *
     * 按设定的加速率和减速率，将当前值平滑地移动到目标值。
     * 可用于云台角度、速度等指令的平滑过渡。
     */

    typedef struct
    {
        float current_value;            // 当前输出值，单位由调用方决定
        float target_value;             // 目标值
        unsigned int interval_ms;       // 更新间隔，单位为毫秒
        float accel;                    // 加速率，单位/秒
        float decel;                    // 减速率，单位/秒
        float max_limit;                // 输出绝对值上限
        unsigned long last_update_time; // 上次更新时间戳
    } RampGenerator;

    /**
     * @brief 初始化斜坡发生器
     * @param ramp 实例指针
     * @param interval_ms 更新间隔，至少为 1 ms
     * @param accel 加速率，不小于 0
     * @param decel 减速率，不小于 0
     * @param max_limit 输出绝对值上限
     */
    void RampGenerator_Init(RampGenerator *ramp,
                            unsigned int interval_ms,
                            float accel,
                            float decel,
                            float max_limit);

    /**
     * @brief 设置目标值
     */
    void RampGenerator_SetTarget(RampGenerator *ramp, float target);

    /**
     * @brief 获取当前输出值
     */
    float RampGenerator_GetCurrent(const RampGenerator *ramp);

    /**
     * @brief 按当前时间更新斜坡输出
     * @param current_time_ms 当前时间戳，单位为毫秒
     */
    void RampGenerator_Update(RampGenerator *ramp, unsigned long current_time_ms);

    /**
     * @brief 修改更新间隔，立即生效
     * @param interval_ms 新间隔，范围 1~1000 ms
     */
    void RampGenerator_SetInterval(RampGenerator *ramp, unsigned int interval_ms);

    /**
     * @brief 修改加速率，下次更新时生效
     * @param accel 新加速率，不小于 0
     */
    void RampGenerator_SetAccel(RampGenerator *ramp, float accel);

    /**
     * @brief 修改减速率，下次更新时生效
     * @param decel 新减速率，不小于 0
     */
    void RampGenerator_SetDecel(RampGenerator *ramp, float decel);

    /**
     * @brief 修改输出上限，并立即限制当前值
     */
    void RampGenerator_SetMaxLimit(RampGenerator *ramp, float max_limit);

    /**
     * @brief 强制设置当前值
     *
     */
    void RampGenerator_SetCurrent(RampGenerator *ramp, float current_value);

#ifdef __cplusplus
}
#endif

#endif // RAMP_GENERATOR_H
