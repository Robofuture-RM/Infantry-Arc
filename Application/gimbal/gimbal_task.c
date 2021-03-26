/* 包含头文件 ----------------------------------------------------------------*/
#include "gimbal_task.h"
#include "infantry_def.h"
#include "cmsis_os.h"
#include "gimbal_function.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/
/* gimbal back center time (ms) */
#define BACK_CENTER_TIME 2500

/* 私有变量 ------------------------------------------------------------------*/
/* 任务 */
osThreadId GimbalTaskHandle;
#if INCLUDE_uxTaskGetStackHighWaterMark
static uint32_t gimbal_task_stack = 0;
#endif

/* control ramp parameter */
static ramp_v0_t yaw_ramp = RAMP_GEN_DAFAULT;
static ramp_v0_t pitch_ramp = RAMP_GEN_DAFAULT;


/* 扩展变量 ------------------------------------------------------------------*/
extern GimbalHandle_t gimbal_handle;

/* 私有函数原形 --------------------------------------------------------------*/
static void GimbalSensorUpdata(void);
static void GimbalCtrlModeSwitch(void);
static void GimbalMotorSendCurrent(int16_t yaw, int16_t pitch);

static void GimbalInitMode(void);
static void GimbalGyroAngleMode(void);
static void GimbalRelativeAngleMode(void);
static void GimbalNormalMode(void);

/* 函数体 --------------------------------------------------------------------*/
void GimbalTask(void *argument)
{
    for(;;)
    {
        GimbalSensorUpdata();
        GimbalCtrlModeSwitch();
        switch (gimbal_handle.ctrl_mode)
        {
            case GIMBAL_INIT:
            {
                GimbalInitMode();
            }break;

            case GIMBAL_GYRO:
            {
                GimbalGyroAngleMode();
            }break;

            case GIMBAL_RELATIVE:
            {
                GimbalRelativeAngleMode();
            }break;

            case GIMBAL_NORMAL:
            {
                GimbalNormalMode();
            }break;
            default:
                break;
        }

        GimbalMotorControl(&gimbal_handle.yaw_motor);
        GimbalMotorControl(&gimbal_handle.pitch_motor);

        if (gimbal_handle.ctrl_mode == GIMBAL_RELAX)
        {
            pid_clear(&gimbal_handle.yaw_motor.pid.outer_pid);
            pid_clear(&gimbal_handle.yaw_motor.pid.inter_pid);
            pid_clear(&gimbal_handle.pitch_motor.pid.outer_pid);
            pid_clear(&gimbal_handle.pitch_motor.pid.inter_pid);
            gimbal_handle.yaw_motor.current_set = 0;
            gimbal_handle.pitch_motor.current_set = 0;
        }

        GimbalMotorSendCurrent((int16_t)YAW_MOTO_POSITIVE_DIR * gimbal_handle.yaw_motor.current_set,
                               (int16_t)PITCH_MOTO_POSITIVE_DIR * gimbal_handle.pitch_motor.current_set);
        osDelay(GIMBAL_TASK_PERIOD);

#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

void GimbalTaskInit(void)
{
    ramp_v0_init(&yaw_ramp, BACK_CENTER_TIME/GIMBAL_TASK_PERIOD);
    ramp_v0_init(&pitch_ramp, BACK_CENTER_TIME/GIMBAL_TASK_PERIOD);
    osThreadDef(gimbal_task, GimbalTask, osPriorityNormal, 0, 256);
    GimbalTaskHandle = osThreadCreate(osThread(gimbal_task), NULL);
}

static void GimbalSensorUpdata(void)
{
    gimbal_handle.yaw_motor.sensor.relative_angle =  gimbal_handle.yaw_motor.ecd_ratio * (fp32)Motor_RelativePosition(gimbal_handle.yaw_motor.motor_info->ecd,
                                                                                                                      gimbal_handle.yaw_motor.offset_ecd);
    gimbal_handle.pitch_motor.sensor.relative_angle =  gimbal_handle.pitch_motor.ecd_ratio * (fp32)Motor_RelativePosition(gimbal_handle.pitch_motor.motor_info->ecd,
                                                                                                                          gimbal_handle.pitch_motor.offset_ecd);
    gimbal_handle.yaw_motor.sensor.gyro_angle = gimbal_handle.imu->attitude.yaw;
    gimbal_handle.pitch_motor.sensor.gyro_angle = gimbal_handle.imu->attitude.pitch;
    gimbal_handle.yaw_motor.sensor.palstance = gimbal_handle.imu->gyro[2] * RAD_TO_ANGLE;
    gimbal_handle.pitch_motor.sensor.palstance = gimbal_handle.imu->gyro[1] * RAD_TO_ANGLE;
}

static void GimbalCtrlModeSwitch(void)
{
    gimbal_handle.last_ctrl_mode = gimbal_handle.ctrl_mode;
    if (gimbal_handle.console->gimbal_cmd == GIMBAL_RELEASE_CMD)
    {
        gimbal_handle.ctrl_mode = GIMBAL_RELAX;
    }
    else if (gimbal_handle.console->gimbal_cmd == GIMBAL_INIT_CMD)
    {
        gimbal_handle.ctrl_mode = GIMBAL_INIT;
    }
    else if (gimbal_handle.console->gimbal_cmd == GIMBAL_GYRO_CMD)
    {
        gimbal_handle.ctrl_mode = GIMBAL_GYRO;
    }
    else if (gimbal_handle.console->gimbal_cmd == GIMBAL_RELATIVE_CMD)
    {
        gimbal_handle.ctrl_mode = GIMBAL_RELATIVE;
    }
    else if (gimbal_handle.console->gimbal_cmd == GIMBAL_NORMAL_CMD)
    {
        gimbal_handle.ctrl_mode = GIMBAL_NORMAL;
    }
}


static void GimbalMotorSendCurrent(int16_t yaw_cur, int16_t pitch_cur)
{
    Motor_SendMessage(gimbal_handle.gimbal_can, GIMBAL_MOTOR_CONTROL_STD_ID, pitch_cur, yaw_cur, 0, 0);
}

static void GimbalInitMode(void)
{
    if(gimbal_handle.last_ctrl_mode != GIMBAL_INIT)
    {
        ramp_v0_init(&yaw_ramp, BACK_CENTER_TIME/GIMBAL_TASK_PERIOD);
        ramp_v0_init(&pitch_ramp, BACK_CENTER_TIME/GIMBAL_TASK_PERIOD);
    }

    gimbal_handle.yaw_motor.mode = ENCONDE_MODE;
    gimbal_handle.pitch_motor.mode = ENCONDE_MODE;

    gimbal_handle.yaw_motor.given_value = gimbal_handle.yaw_motor.sensor.relative_angle;
    gimbal_handle.pitch_motor.given_value = gimbal_handle.pitch_motor.sensor.relative_angle * (1 - ramp_v0_calculate(&pitch_ramp));
    if (fabsf(gimbal_handle.pitch_motor.sensor.relative_angle) <= 2.0f)
    {
        gimbal_handle.yaw_motor.given_value = gimbal_handle.yaw_motor.sensor.relative_angle * (1 - ramp_v0_calculate(&yaw_ramp));

        if (fabsf(gimbal_handle.yaw_motor.sensor.relative_angle) <= 3.0f )
        {
            gimbal_handle.ctrl_mode = GIMBAL_NORMAL;
        }
    }
}

static void GimbalGyroAngleMode(void)
{
    gimbal_handle.yaw_motor.mode = GYRO_MODE;
    gimbal_handle.pitch_motor.mode = GYRO_MODE;

    fp32 yaw_target = 0, pitch_target = 0;

    yaw_target = gimbal_handle.yaw_motor.given_value + gimbal_handle.console->gimbal.yaw_v;
    pitch_target = gimbal_handle.pitch_motor.given_value + gimbal_handle.console->gimbal.pitch_v;

    gimbal_handle.yaw_motor.given_value = AngleTransform(yaw_target, gimbal_handle.yaw_motor.sensor.gyro_angle);
    gimbal_handle.pitch_motor.given_value = AngleTransform(pitch_target, gimbal_handle.pitch_motor.sensor.gyro_angle);

    VAL_LIMIT(gimbal_handle.pitch_motor.given_value, gimbal_handle.pitch_motor.min_relative_angle, gimbal_handle.pitch_motor.max_relative_angle);

}

static void GimbalRelativeAngleMode(void)
{
    gimbal_handle.yaw_motor.mode = ENCONDE_MODE;
    gimbal_handle.pitch_motor.mode = ENCONDE_MODE;

    gimbal_handle.yaw_motor.given_value += gimbal_handle.console->gimbal.yaw_v;
    gimbal_handle.pitch_motor.given_value += gimbal_handle.console->gimbal.pitch_v;

    VAL_LIMIT(gimbal_handle.yaw_motor.given_value, gimbal_handle.yaw_motor.min_relative_angle, gimbal_handle.yaw_motor.max_relative_angle);
    VAL_LIMIT(gimbal_handle.pitch_motor.given_value, gimbal_handle.pitch_motor.min_relative_angle, gimbal_handle.pitch_motor.max_relative_angle);

}

static void GimbalNormalMode(void)
{
    fp32 yaw_target = 0;

    gimbal_handle.yaw_motor.mode = GYRO_MODE;
    gimbal_handle.pitch_motor.mode = ENCONDE_MODE;

    yaw_target = gimbal_handle.yaw_motor.given_value + gimbal_handle.console->gimbal.yaw_v;

    gimbal_handle.yaw_motor.given_value = AngleTransform(yaw_target, gimbal_handle.yaw_motor.sensor.gyro_angle);
    gimbal_handle.pitch_motor.given_value += gimbal_handle.console->gimbal.pitch_v;

    VAL_LIMIT(gimbal_handle.pitch_motor.given_value, gimbal_handle.pitch_motor.min_relative_angle, gimbal_handle.pitch_motor.max_relative_angle);

}
