/**
 * @file  Chassis.h
 * @brief 云台板使用的底盘指令接口。
 *
 * @note 云台板只维护待发送的底盘目标值，底盘解算和电机控制均在底盘板执行。
 */

#ifndef INCLUDED_CHASSIS_H
#define INCLUDED_CHASSIS_H

#include "Global_status.h"

static inline void Chassis_Init(void) {}
static inline void Chassis_Tasks(void) {}
static inline void Chassis_SetX(float x) { Global.Chassis.input.x = x; }
static inline void Chassis_SetY(float y) { Global.Chassis.input.y = y; }
static inline void Chassis_SetR(float r) { Global.Chassis.input.r = r; }
static inline void Chassis_SetAccel(float acc) { (void)acc; }

#endif /* INCLUDED_CHASSIS_H */
