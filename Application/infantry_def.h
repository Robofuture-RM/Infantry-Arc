#ifndef INFANTRY_DEF_H
#define INFANTRY_DEF_H
/* 包含头文件 ----------------------------------------------------------------*/
#include "Motor/motor.h"

/* 宏定义 --------------------------------------------------------------------*/
/******************************************************************************
 *                              CAN通讯ID配置                                                                    *
 ******************************************************************************/
/*---------------↓ 底盘电机ID ↓---------------*/
#define CHASSIS_MOTOR_CONTROL_STD_ID    MOTOR_1TO4_CONTROL_STD_ID
#define CHASSIS_MOTOR_LF_MESSAGE_ID     MOTOR_1_FEEDBACK_ID     //底盘左前电机
#define CHASSIS_MOTOR_RF_MESSAGE_ID     MOTOR_2_FEEDBACK_ID     //底盘右前电机
#define CHASSIS_MOTOR_LB_MESSAGE_ID     MOTOR_3_FEEDBACK_ID     //底盘左后电机
#define CHASSIS_MOTOR_RB_MESSAGE_ID     MOTOR_4_FEEDBACK_ID     //底盘右后电机
/*---------------↓ 云台电机ID ↓---------------*/
#define GIMBAL_MOTOR_CONTROL_STD_ID     MOTOR_5TO8_CONTROL_STD_ID
#define GIMBAL_MOTOR_YAW_MESSAGE_ID     MOTOR_6_FEEDBACK_ID     //Yaw云台电机
#define GIMBAL_MOTOR_PITCH_MESSAGE_ID   MOTOR_5_FEEDBACK_ID     //Pitch云台电机
/*---------------↓ 射击电机ID ↓---------------*/
#define SHOOT_MOTOR_CONTROL_STD_ID      MOTOR_1TO4_CONTROL_STD_ID
#define FRICTION_WHEEL_1_MESSAGE_ID     MOTOR_1_FEEDBACK_ID
#define FRICTION_WHEEL_2_MESSAGE_ID     MOTOR_2_FEEDBACK_ID
#define TRIGGER_MOTOR_MESSAGE_ID        MOTOR_3_FEEDBACK_ID
#define MAGAZINE_MOTOR_MESSAGE_ID       MOTOR_4_FEEDBACK_ID


/******************************************************************************
 *                                                     云台主控与底盘主控交互ID                            *
 ******************************************************************************/
#define GIMBAL_DATA_STD_ID              (0x600)
#define CHASSIS_DATA_STD_ID             (0x500)
#define GIMBAL_TX_DATA_STD_ID           GIMBAL_DATA_STD_ID
#define GIMBAL_RX_DATA_STD_ID           CHASSIS_DATA_STD_ID
#define CHASSIS_TX_DATA_STD_ID          CHASSIS_DATA_STD_ID
#define CHASSIS_RX_DATA_STD_ID          GIMBAL_DATA_STD_ID
#define GIMBAL_CHASSIS_DATA_FIFO_SIZE   (1024u)
#define REFEREE_DATA_STD_ID             (0x700)


/******************************************************************************
 *                                                              机械安装参数                                                                      *
 ******************************************************************************/
#define WHEEL_RADIUS                (76)    //轮子半径(mm)
#define WHEEL_PERIMETER             (478)   //轮子周长(mm)
#define WHEELTRACK                  (401)   //轮距(mm)
#define WHEELBASE                   (350)   //轴距(mm)
#define GIMBAL_X_OFFSET             (0)     //云台相对底盘中心X轴偏移
#define GIMBAL_Y_OFFSET             (0)     //云台相对底盘中心Y轴偏移
#define PITCH_REDUCTION_RATIO       (1.0f)  //pitch减速比
#define YAW_REDUCTION_RATIO         (1.0f)  //yaw减速比
#define PITCH_MOTO_POSITIVE_DIR     (1.0f)  //pitch电机安装方向
#define YAW_MOTO_POSITIVE_DIR       (1.0f)  //yaw电机安装方向


/******************************************************************************
 *                                                                   移动控制                                                                         *
 ******************************************************************************/
#define MAX_CHASSIS_VX_SPEED        (4000.0f)
#define MAX_CHASSIS_VY_SPEED        (4000.0f)
#define MAX_CHASSIS_VW_SPEED        (300.0f)
/*-----------------↓ 遥控 ↓-----------------*/
#define RC_CHASSIS_MAX_SPEED_X      MAX_CHASSIS_VX_SPEED     //X轴方向最大速度(mm/s)
#define RC_CHASSIS_MAX_SPEED_Y      MAX_CHASSIS_VX_SPEED     //Y轴方向最大速度(mm/s)
#define RC_CHASSIS_MAX_SPEED_R      MAX_CHASSIS_VW_SPEED     //旋转最大速度(deg/s)
#define RC_GIMBAL_MOVE_RATIO_PIT    0.001f       //pitch移动比例
#define RC_GIMBAL_MOVE_RATIO_YAW    0.002f       //yaw移动比例
/*---------------↓ 鼠标键盘 ↓---------------*/
#define KB_CHASSIS_MAX_SPEED_X      MAX_CHASSIS_VX_SPEED     //X轴方向最大速度
#define KB_CHASSIS_MAX_SPEED_Y      MAX_CHASSIS_VX_SPEED     //Y轴方向最大速度
#define KB_CHASSIS_MAX_SPEED_R      MAX_CHASSIS_VW_SPEED      //旋转最大速度
#define KB_GIMBAL_MOVE_RATIO_PIT    0.005f       //pitch移动比例
#define KB_GIMBAL_MOVE_RATIO_YAW    0.01f        //yaw移动比例

#define CHASSIS_ACCEL_TIME      1500  //ms
#define ROTATE_ACCEL_TIME       3000  //ms


/******************************************************************************
 *                                                                   任务周期                                                                         *
 ******************************************************************************/
/*---------------↓ 通用任务 ↓---------------*/
#define START_TASK_PERIOD           100
#define IMU_TASK_PERIOD             5
#define CONSOLE_TASK_PERIOD         20
#define COMM_TASK_PERIOD            1
#define DETECT_TASK_PERIOD          10
/*---------------↓ 底盘任务 ↓---------------*/
#define CHASSIS_TASK_PERIOD         10
/*---------------↓ 云台任务 ↓---------------*/
#define GIMBAL_TASK_PERIOD          10
#define SHOOT_TASK_PERIOD           10
#define GIMBAL_UPLOAD_TIMER_PERIOD  20



#endif  // INFANTRY_DEF_H

