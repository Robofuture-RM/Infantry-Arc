#ifndef CONSOLE_H
#define CONSOLE_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "RemoteControl/remote_control.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
    PREPARE_MODE = 0,       //初始化
    NORMAL_MODE,            //正常运行模式
    SAFETY_MODE,              //安全模式（停止运动）
} CtrlMode_e;

typedef enum
{
    GIMBAL_RELEASE_CMD = 0,
    GIMBAL_INIT_CMD,
    GIMBAL_GYRO_CMD,
    GIMBAL_RELATIVE_CMD,
    GIMBAL_NORMAL_CMD,
} Gimbal_CMD_e;

typedef enum
{
    CHASSIS_RELEASE_CMD = 0,
    CHASSIS_STOP_CMD,               //底盘停止
    CHASSIS_FOLLOW_GIMBAL_CMD,      //底盘跟随云台
    CHASSIS_SEPARATE_GIMBAL_CMD,    //底盘云台分离
    CHASSIS_SPIN_CMD,               //底盘旋转
} Chassis_CMD_e;

typedef enum
{
    SHOOT_RELEASE_CMD = 0,
    SHOOT_START_CMD,
    SHOOT_STOP_CMD,
} Shoot_CMD_e;

typedef enum
{
    STOP_FIRE_CMD = 0,
    ONE_FIRE_CMD,
    RAPID_FIRE_CMD,
} ShootFire_CMD_e;

typedef struct
{
    RC_Info_t* rc;
    CtrlMode_e ctrl_mode;
    Gimbal_CMD_e gimbal_cmd;
    Chassis_CMD_e chassis_cmd;
    Shoot_CMD_e shoot_cmd;

    struct
    {
        fp32 vx;
        fp32 vy;
        fp32 vw;
    } chassis;

    struct
    {
        fp32 pitch_v;
        fp32 yaw_v;
    } gimbal;

    struct
    {
        ShootFire_CMD_e fire_cmd;
    } shoot;

} Console_t;

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void ConsoleTaskInit(void);
Console_t* Console_Pointer(void);

#endif  // CONSOLE_H

