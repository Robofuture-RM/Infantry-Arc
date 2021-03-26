#ifndef CHASSIS_APP_H
#define CHASSIS_APP_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "RemoteControl/remote_control.h"
#include "IMU/imu_driver.h"
#include "Motor/motor.h"
#include "pid.h"
#include "infantry_console.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
    CHASSIS_RELAX = 0,          //安全模式
    CHASSIS_STOP,               //底盘停止
    CHASSIS_FOLLOW_GIMBAL,      //底盘跟随云台
    CHASSIS_SEPARATE_GIMBAL,    //底盘云台分离
    CHASSIS_SPIN                //底盘旋转
} ChassisCtrlMode_e;

typedef struct
{
    fp32 wheel_perimeter; /* the perimeter(mm) of wheel */
    fp32 wheeltrack;      /* wheel track distance(mm) */
    fp32 wheelbase;       /* wheelbase distance(mm) */
    fp32 rotate_x_offset; /* rotate offset(mm) relative to the x-axis of the chassis center */
    fp32 rotate_y_offset; /* rotate offset(mm) relative to the y-axis of the chassis center */
} MechanicalStructure_t;

//typedef struct
//{
//    int16_t gyro_angle;
//    int16_t gyro_palstance;
//    int32_t position_x_mm;
//    int32_t position_y_mm;
//    int16_t angle_deg;
//    int16_t v_x_mm;
//    int16_t v_y_mm;
//} ChassisInfo_t;

typedef struct
{
    MotorInfo_t*    motor_info;
    pid_t           pid;
    fp32            given_speed;
    int16_t         current_set;
} ChassisMotor_t;

typedef struct
{
    Console_t*    console;
    IMU_Data_t*   imu;                      //底盘陀螺仪指针
    CAN_Object_t* chassis_can;              //

    ChassisCtrlMode_e       ctrl_mode;             //底盘控制状态
    MechanicalStructure_t   structure;
    ChassisMotor_t          chassis_motor[4];
    pid_t                   chassis_follow_pid;        //follow angle PID.

    fp32 vx;                      //
    fp32 vy;                      //
    fp32 vw;                      //
    fp32 wheel_rpm[4];

    fp32 gimbal_yaw_ecd_angle;
    fp32 chassis_yaw;   //the yaw angle calculated by gyro sensor and gimbal motor.
    fp32 chassis_pitch; //the pitch angle calculated by gyro sensor and gimbal motor.
    fp32 chassis_roll;  //the roll angle calculated by gyro sensor and gimbal motor.
} ChassisHandle_t;

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void ChassisAppConfig(void);

#endif  // CHASSIS_APP_H

