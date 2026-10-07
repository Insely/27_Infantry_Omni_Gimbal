#ifndef CHASSIS_H
#define CHASSIS_H
extern float X_speed, Y_speed, R_speed;
void Chassis_Init(void);
void Chassis_Tasks(void);
void Chassis_SetX(float x);
void Chassis_SetY(float y);
void Chassis_SetR(float r);
void Chassis_SetAccel(float acc);
#endif
