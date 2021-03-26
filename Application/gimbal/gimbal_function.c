/* 包含头文件 ----------------------------------------------------------------*/
#include "gimbal_function.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
fp32 Gimbal_PID_Calc(Gimbal_PID_t* pid, fp32 angle_ref, fp32 angle_fdb, fp32 speed_fdb)
{
    pid->angle_ref = angle_ref;
    pid->angle_fdb = angle_fdb;
    pid_calc(&pid->outer_pid, pid->angle_fdb, pid->angle_ref);
    pid->speed_ref = pid->outer_pid.out;
    pid->speed_fdb = speed_fdb;
    pid_calc(&pid->inter_pid, pid->speed_fdb, pid->speed_ref);
    return pid->inter_pid.out;
}

void Gimbal_PID_Clear(Gimbal_PID_t* pid)
{
    pid_clear(&pid->outer_pid);
    pid_clear(&pid->inter_pid);
}

void GimbalMotorChangeProtect(GimbalMotor_t* motor)
{
    if (motor->last_mode != motor->mode)
    {
        if(motor->mode == RAW_VALUE_MODE)
        {
            motor->given_value = motor->current_set;
        }
        else if (motor->mode == GYRO_MODE)
        {
            motor->given_value = motor->sensor.gyro_angle;
        }
        else if (motor->mode == ENCONDE_MODE)
        {
            motor->given_value = motor->sensor.relative_angle;
        }
    }
    motor->last_mode = motor->mode;
}

void GimbalMotorControl(GimbalMotor_t* motor)
{
    GimbalMotorChangeProtect(motor);
    if (motor->mode == RAW_VALUE_MODE)
    {
        motor->current_set = motor->given_value;
        Gimbal_PID_Clear(&motor->pid);
    }
    else if(motor->mode == GYRO_MODE)
    {
        motor->current_set = Gimbal_PID_Calc(&motor->pid,
                                             motor->given_value,
                                             motor->sensor.gyro_angle,
                                             motor->sensor.palstance);
    }
    else if(motor->mode == ENCONDE_MODE)
    {
        motor->current_set = Gimbal_PID_Calc(&motor->pid,
                                             motor->given_value,
                                             motor->sensor.relative_angle,
                                             motor->sensor.palstance);
    }
}

fp32 AngleTransform(fp32 target_angle, fp32 gyro_angle)
{
    float offset = 0, now = 0, target = 0;

    ANGLE_LIMIT_360(target, target_angle);
    ANGLE_LIMIT_360(now, gyro_angle);

    offset = target - now;
    if (offset > 180)
    {
        offset = offset - 360;
    }
    else if (offset < -180)
    {
        offset = offset + 360;
    }
    return gyro_angle + offset;
}
