/* 包含头文件 ----------------------------------------------------------------*/
#include "chassis_task.h"
#include "infantry_def.h"
#include "cmsis_os.h"
#include "chassis_function.h"

#include "user_protocol.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
/* 任务 */
osThreadId ChassisTaskHandle;
#if INCLUDE_uxTaskGetStackHighWaterMark
static uint32_t chassis_task_stack = 0;
#endif

/* 扩展变量 ------------------------------------------------------------------*/
extern ChassisHandle_t chassis_handle;

/* 私有函数原形 --------------------------------------------------------------*/
static void ChassisCtrlModeSwitch(void);
static void ChassisSensorUpdata(void);

static void ChassisMotorSendCurrent(int16_t motor1_cur, int16_t motor2_cur, int16_t motor3_cur, int16_t motor4_cur);
static void ChassisStopMode(void);
static void ChassisFollowGimbalMode(void);
static void ChassisSeparateGimbalMode(void);
static void ChassisSpinMode(void);
/* 函数体 --------------------------------------------------------------------*/
void ChassisTask(void *argument)
{
    for(;;)
    {
        ChassisSensorUpdata();
        ChassisCtrlModeSwitch();

        switch (chassis_handle.ctrl_mode)
        {
            case CHASSIS_STOP:
            {
                ChassisStopMode();
            }break;
            case CHASSIS_FOLLOW_GIMBAL:
            {
                ChassisFollowGimbalMode();
            }break;
            case CHASSIS_SEPARATE_GIMBAL:
            {
                ChassisSeparateGimbalMode();
            }break;
            case CHASSIS_SPIN:
            {
                ChassisSpinMode();
            }break;
            default:
                break;
        }

        Chassis_ControlCalc(&chassis_handle);

        for (uint8_t i = 0; i < 4; i++)
        {
            chassis_handle.chassis_motor[i].given_speed = chassis_handle.wheel_rpm[i];
            chassis_handle.chassis_motor[i].current_set = pid_calc(&chassis_handle.chassis_motor[i].pid,
                                                                   chassis_handle.chassis_motor[i].motor_info->speed_rpm,
                                                                   chassis_handle.chassis_motor[i].given_speed);
        }

        Chassis_LimitPower(&chassis_handle);

        if(chassis_handle.ctrl_mode == CHASSIS_RELAX)
        {
            for (uint8_t i = 0; i < 4; i++)
            {
                chassis_handle.chassis_motor[i].current_set = 0;
            }
        }
        ChassisMotorSendCurrent(chassis_handle.chassis_motor[0].current_set,
                                chassis_handle.chassis_motor[1].current_set,
                                chassis_handle.chassis_motor[2].current_set,
                                chassis_handle.chassis_motor[3].current_set);
        osDelay(CHASSIS_TASK_PERIOD);
#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

void ChassisTaskInit(void)
{
    osThreadDef(chassis_task, ChassisTask, osPriorityNormal, 0, 256);
    ChassisTaskHandle = osThreadCreate(osThread(chassis_task), NULL);
}

static void ChassisSensorUpdata(void)
{
    Comm_GimbalInfo_t* gimbal_info = GimbalInfo_Pointer();
    chassis_handle.gimbal_yaw_ecd_angle = gimbal_info->yaw_ecd_angle;
    chassis_handle.chassis_pitch = chassis_handle.imu->attitude.pitch - gimbal_info->pitch_gyro_angle;
    chassis_handle.chassis_roll = chassis_handle.imu->attitude.roll;
    chassis_handle.chassis_yaw = chassis_handle.imu->attitude.yaw - gimbal_info->yaw_gyro_angle;
}

static void ChassisCtrlModeSwitch(void)
{
    if (chassis_handle.console->chassis_cmd == CHASSIS_RELEASE_CMD)
    {
        chassis_handle.ctrl_mode = CHASSIS_RELAX;
    }
    else if (chassis_handle.console->chassis_cmd == CHASSIS_STOP_CMD)
    {
        chassis_handle.ctrl_mode = CHASSIS_STOP;
    }
    else if (chassis_handle.console->chassis_cmd == CHASSIS_FOLLOW_GIMBAL_CMD)
    {
        chassis_handle.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
    }
    else if (chassis_handle.console->chassis_cmd == CHASSIS_SEPARATE_GIMBAL_CMD)
    {
        chassis_handle.ctrl_mode = CHASSIS_SEPARATE_GIMBAL;
    }
    else if (chassis_handle.console->chassis_cmd == CHASSIS_SPIN_CMD)
    {
        chassis_handle.ctrl_mode = CHASSIS_SPIN;
    }
}

static void ChassisMotorSendCurrent(int16_t motor1_cur, int16_t motor2_cur, int16_t motor3_cur, int16_t motor4_cur)
{
    Motor_SendMessage(chassis_handle.chassis_can, MOTOR_1TO4_CONTROL_STD_ID, motor1_cur, motor2_cur, motor3_cur, motor4_cur);
}

static void ChassisStopMode(void)
{
    chassis_handle.vx = 0;
    chassis_handle.vy = 0;
    chassis_handle.vw = 0;
}

static void ChassisFollowGimbalMode(void)
{
    chassis_handle.vx = chassis_handle.console->chassis.vx;
    chassis_handle.vy = chassis_handle.console->chassis.vy;
    chassis_handle.vw = pid_calc(&chassis_handle.chassis_follow_pid,
                                 -chassis_handle.gimbal_yaw_ecd_angle,
                                 0);
}

static void ChassisSeparateGimbalMode(void)
{
    chassis_handle.vx = chassis_handle.console->chassis.vx;
    chassis_handle.vw = chassis_handle.console->chassis.vw;
}

static void ChassisSpinMode(void)
{
    chassis_handle.vx = chassis_handle.console->chassis.vx;
    chassis_handle.vy = chassis_handle.console->chassis.vy;
    if (fabs(chassis_handle.vx) < 500 && fabs(chassis_handle.vy) < 500)
        chassis_handle.vw = 200;
    else if (fabs(chassis_handle.vx) < 1000 && fabs(chassis_handle.vy) < 1000)
        chassis_handle.vw = 100;
    else
        chassis_handle.vw = 0;
}

