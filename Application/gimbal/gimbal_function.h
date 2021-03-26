#ifndef GIMBAL_FUNCTION_H
#define GIMBAL_FUNCTION_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "gimbal_app.h"
#include "ramp.h"
#include "arm_math.h"
#include "user_lib.h"

/* 类型定义 ------------------------------------------------------------------*/

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
fp32 Gimbal_PID_Calc(Gimbal_PID_t* pid, fp32 angle_ref, fp32 angle_fdb, fp32 speed_fdb);
void Gimbal_PID_Clear(Gimbal_PID_t* pid);
void GimbalMotorChangeProtect(GimbalMotor_t* motor);
void GimbalMotorControl(GimbalMotor_t* motor);
fp32 AngleTransform(fp32 target_angle, fp32 gyro_angle);

#endif  // GIMBAL_FUNCTION_H

