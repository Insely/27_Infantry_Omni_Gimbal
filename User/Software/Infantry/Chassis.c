#include "robot_param.h"
#if ROBOT_TYPE == ROBOT_INFANTRY
#include "Chassis.h"
#include "Global_status.h"

void Chassis_Init(void)
{
    Global.Chassis.mode = FLOW;
    Global.Chassis.input.x = Global.Chassis.input.y = Global.Chassis.input.r = 0;
}

void Chassis_Tasks(void)
{
    /* 底盘角速度和坐标变换由底盘板统一结算。 */
}
void Chassis_SetX(float x) { Global.Chassis.input.x = x; }
void Chassis_SetY(float y) { Global.Chassis.input.y = y; }
void Chassis_SetR(float r) { Global.Chassis.input.r = r; }
void Chassis_SetAccel(float acc) { (void)acc; /* No acceleration field in BoardLink. */ }
#endif
