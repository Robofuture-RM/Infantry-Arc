/* 包含头文件 ----------------------------------------------------------------*/
#include "shoot_task.h"
#include "infantry_def.h"
#include "cmsis_os.h"
#include "math.h"
#include "bsp_init.h"
#include "detect_task.h"
#include "referee_system.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/
#define MAGAZINE_MOTOR_INIT_SET         (1000)
#define MAGAZINE_MOTOR_POSITIVE_DIR     (-1.0f)
#define MAGAZINE_MOTOR_REDUCTION_RATIO  M2006_REDUCTION_RATIO
#define TRIGGER_MOTOR_POSITIVE_DIR      (-1.0f)
#define TRIGGER_MOTOR_REDUCTION_RATIO   M2006_REDUCTION_RATIO

#define LASER_ON()  BSP_GPIO_SetPwmValue(&laser_gpio, 8399)
#define LASER_OFF() BSP_GPIO_SetPwmValue(&laser_gpio, 0)

#define TRIGGER_PLATE_NUMBERS   (8.0f)
#define TRIGGER_BLOCKED_TIMER   (500)
#define REVERSE_TIMER           (300)
#define REVERSE_ANGLE           (15.0f)

#define ONE_BULLET_HEAT         (10u)

/* 私有变量 ------------------------------------------------------------------*/
osThreadId ShootTaskHandle;
#if INCLUDE_uxTaskGetStackHighWaterMark
static uint32_t shoot_task_stack = 0;
#endif

ShootHandle_t shoot_handle;

uint16_t friction_wheel_speed = 7000; //4900-16.7m/s  5900-20m/s    7000-24m/s      8000-25m/s
/* 扩展变量 ------------------------------------------------------------------*/
extern CAN_Object_t can2_obj;

/* 私有函数原形 --------------------------------------------------------------*/
static void ShootSensorUpdata(void);
static void ShootCtrlModeSwitch(void);

static void Shoot_LaserCtrl(ShootCtrlMode_e mode);
static void Shoot_MagazineMotorCtrl(ShootHandle_t* handle);
static void Shoot_TriggerMotorCtrl(ShootHandle_t* handle);
static void Shoot_FrictionWheelMotorCtrl(ShootCtrlMode_e mode, FrictionWheelMotor_t motor[2]);

static void ShootMotorSendCurrent(int16_t fric1_cur, int16_t fric2_cur, int16_t trigger_cur, int16_t magazine_cur);

/* 函数体 --------------------------------------------------------------------*/
void ShootTask(void *argument)
{
    for(;;)
    {
        ShootSensorUpdata();
        ShootCtrlModeSwitch();

        Shoot_LaserCtrl(shoot_handle.ctrl_mode);
        Shoot_MagazineMotorCtrl(&shoot_handle);
        Shoot_TriggerMotorCtrl(&shoot_handle);
        Shoot_FrictionWheelMotorCtrl(shoot_handle.ctrl_mode, shoot_handle.fric_wheel_motor);

        if (shoot_handle.ctrl_mode == SHOOT_RELAX)
        {
            shoot_handle.fric_wheel_motor[0].current_set = 0;
            shoot_handle.fric_wheel_motor[1].current_set = 0;
            shoot_handle.trigger_motor.current_set = 0;
        }
//        shoot_handle.magazine_motor.current_set = 0;
        ShootMotorSendCurrent(shoot_handle.fric_wheel_motor[0].current_set,
                              shoot_handle.fric_wheel_motor[1].current_set,
                              TRIGGER_MOTOR_POSITIVE_DIR * shoot_handle.trigger_motor.current_set,
                              MAGAZINE_MOTOR_POSITIVE_DIR * shoot_handle.magazine_motor.current_set);
        osDelay(SHOOT_TASK_PERIOD);

#if INCLUDE_uxTaskGetStackHighWaterMark
        shoot_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

void ShootTaskInit(void)
{
    shoot_handle.console = Console_Pointer();
    shoot_handle.shoot_can = &can2_obj;

    shoot_handle.magazine_motor.motor_info = MagazineMotor_Pointer();
    shoot_handle.magazine_state = MAGAZINE_INIT_STATE;
    shoot_handle.magazine_motor.ecd_ratio = MAGAZINE_MOTOR_POSITIVE_DIR * MAGAZINE_MOTOR_REDUCTION_RATIO / ENCODER_ANGLE_RATIO;
    pid_init(&shoot_handle.magazine_motor.pid.outer_pid, POSITION_PID, 1000.0f, 500.0f,
             60.0f, 0.0f, 60.0f);
    pid_init(&shoot_handle.magazine_motor.pid.inter_pid, POSITION_PID, 3000.0f, 500.0f,
             1.0f, 0.0f, 3.0f);
    Blocked_Reset(&shoot_handle.magazine_motor.blocked_handle, 2000, 3000);

    shoot_handle.trigger_motor.motor_info = TriggerMotor_Pointer();
    shoot_handle.trigger_state = TRIGGER_END;
    shoot_handle.trigger_motor.ecd_ratio = TRIGGER_MOTOR_POSITIVE_DIR * TRIGGER_MOTOR_REDUCTION_RATIO / ENCODER_ANGLE_RATIO;
    pid_init(&shoot_handle.trigger_motor.pid.outer_pid, POSITION_PID, 300.0f, 60.0f,
             5.0f, 0.0f, 0.0f);
    pid_init(&shoot_handle.trigger_motor.pid.inter_pid, POSITION_PID, M2006_MOTOR_MAX_CURRENT, 7000.0f,
             100.0f, 0.0f, 0.0f);
    Blocked_Reset(&shoot_handle.trigger_motor.blocked_handle, TRIGGER_BLOCKED_TIMER, 1000);

    shoot_handle.fric_wheel_motor[0].motor_info = FrictionWheelMotor_1_Pointer();
    shoot_handle.fric_wheel_motor[1].motor_info = FrictionWheelMotor_2_Pointer();
    for (uint8_t i = 0; i < 2; i++)
    {
        pid_init(&shoot_handle.fric_wheel_motor[i].pid, POSITION_PID, M3508_MOTOR_MAX_CURRENT, 500.0f,
                 6.0f, 0.0f, 0.0f);
    }
    LASER_OFF();

    osThreadDef(shoot_task, ShootTask, osPriorityNormal, 0, 256);
    ShootTaskHandle = osThreadCreate(osThread(shoot_task), NULL);
}

static void ShootSensorUpdata(void)
{
    shoot_handle.magazine_motor.speed = (fp32)shoot_handle.magazine_motor.motor_info->speed_rpm * MAGAZINE_MOTOR_REDUCTION_RATIO;
    shoot_handle.magazine_motor.angle = shoot_handle.magazine_motor.ecd_ratio
            * (fp32)(shoot_handle.magazine_motor.motor_info->total_ecd - shoot_handle.magazine_motor.offset_ecd);

    shoot_handle.trigger_motor.speed = (fp32)shoot_handle.trigger_motor.motor_info->speed_rpm * TRIGGER_MOTOR_REDUCTION_RATIO * TRIGGER_MOTOR_POSITIVE_DIR;
    shoot_handle.trigger_motor.angle = shoot_handle.trigger_motor.ecd_ratio
            * (fp32)(shoot_handle.trigger_motor.motor_info->total_ecd - shoot_handle.trigger_motor.offset_ecd);

    if (!CheckDeviceIsOffline(OFFLINE_REFEREE_SYSTEM))
    {
        shoot_handle.shooter_heat_cooling_rate = RefereeSystem_RobotState_Pointer()->shooter_heat0_cooling_rate;
        shoot_handle.shooter_heat_cooling_limit = RefereeSystem_RobotState_Pointer()->shooter_heat0_cooling_limit;
        shoot_handle.shooter_heat = RefereeSystem_PowerHeatData_Pointer()->shooter_heat0;
    }
    else
    {
        shoot_handle.shooter_heat_cooling_rate = 0;
        shoot_handle.shooter_heat_cooling_limit = 200;
        shoot_handle.shooter_heat = 0;
    }

}

static void ShootCtrlModeSwitch(void)
{
    if (shoot_handle.console->shoot_cmd == SHOOT_RELEASE_CMD)
    {
        shoot_handle.ctrl_mode = SHOOT_RELAX;
    }
    else if (shoot_handle.console->shoot_cmd == SHOOT_START_CMD)
    {
        shoot_handle.ctrl_mode = SHOOT_START;
    }
    else if (shoot_handle.console->shoot_cmd == SHOOT_STOP_CMD)
    {
        shoot_handle.ctrl_mode = SHOOT_STOP;
    }
}

static void Shoot_LaserCtrl(ShootCtrlMode_e mode)
{
    if (mode == SHOOT_START)
    {
        LASER_ON();
    }
    else
    {
        LASER_OFF();
    }
}

static void Shoot_MagazineMotorCtrl(ShootHandle_t* handle)
{
    static fp32 zero_angle = 0.0f;
    if (handle->magazine_state != MAGAZINE_INIT_STATE)
    {
        if (handle->ctrl_mode == SHOOT_START)
        {
            handle->magazine_motor.set_angle = zero_angle;
        }
        else //if (handle->ctrl_mode == SHOOT_STOP)
        {
            handle->magazine_motor.set_angle = zero_angle - 90;
        }
        handle->magazine_motor.current_set = DoublePID_Calc(&handle->magazine_motor.pid,
                                                            handle->magazine_motor.set_angle,
                                                            handle->magazine_motor.angle,
                                                            handle->magazine_motor.speed);
    }
    else
    {
        handle->magazine_motor.current_set = MAGAZINE_MOTOR_INIT_SET;
        BlockedState_t blocked = Blocked_Process(&handle->magazine_motor.blocked_handle, handle->magazine_motor.speed);
        if (blocked == BLOCKED)
        {
            handle->magazine_state = MAGAZINE_OFF_STATE;
            zero_angle = handle->magazine_motor.angle;
            handle->magazine_motor.set_angle = handle->magazine_motor.angle;
        }
    }
}

static void Shoot_TriggerMotorCtrl(ShootHandle_t* handle)
{
    static uint8_t max_bullet_nums = 0;
    static uint32_t reverse_time = 0;
    if (handle->ctrl_mode == SHOOT_START)
    {
        if (handle->trigger_state == TRIGGER_END)
        {
            /*可以发弹量 = (最大热量 - 枪口热量) / 一发子弹热量 */
            max_bullet_nums = (handle->shooter_heat_cooling_limit - handle->shooter_heat) / ONE_BULLET_HEAT;

            if (handle->console->shoot.fire_cmd == ONE_FIRE_CMD && max_bullet_nums >= 1)
                handle->fire_bullet_number = 1;
            else if (handle->console->shoot.fire_cmd == RAPID_FIRE_CMD && max_bullet_nums >= 5)
                handle->fire_bullet_number = 5;

            if (handle->fire_bullet_number != 0)
                handle->trigger_state = TRIGGER_BEGIN;
        }
        else if (handle->trigger_state == TRIGGER_BEGIN)
        {
            reverse_time = 0;
            if (handle->fire_bullet_number != 0)
            {
                handle->trigger_state = TRIGGERING;
                handle->trigger_motor.set_angle = handle->trigger_motor.angle + (360.0f / TRIGGER_PLATE_NUMBERS);
                Blocked_Reset(&handle->trigger_motor.blocked_handle, TRIGGER_BLOCKED_TIMER, 1000);
            }
            else
            {
                handle->trigger_state = TRIGGER_END;
            }
        }
        else if (handle->trigger_state == TRIGGERING)
        {
            BlockedState_t blocked = Blocked_Process(&handle->trigger_motor.blocked_handle, handle->trigger_motor.speed);
            if ( fabs(handle->trigger_motor.set_angle - handle->trigger_motor.angle) < 0.5f )
            {
                handle->fire_bullet_number--;
                handle->trigger_state = TRIGGER_BEGIN;
            }
            else if (blocked == BLOCKED)
            {
                if (reverse_time == 0)
                {
                    reverse_time = BSP_GetTime_ms();
                    handle->trigger_motor.set_angle = handle->trigger_motor.angle - REVERSE_ANGLE;
                }
                else if (BSP_GetTime_ms() - reverse_time > REVERSE_TIMER)
                {
                    handle->trigger_state = TRIGGER_BEGIN;
                }
            }
        }
    }
    else
    {
        handle->trigger_state = TRIGGER_END;
        handle->trigger_motor.set_angle = handle->trigger_motor.angle;
    }
    handle->trigger_motor.current_set = DoublePID_Calc(&handle->trigger_motor.pid,
                                                       handle->trigger_motor.set_angle,
                                                       handle->trigger_motor.angle,
                                                       handle->trigger_motor.speed);
}

static void Shoot_FrictionWheelMotorCtrl(ShootCtrlMode_e mode, FrictionWheelMotor_t motor[2])
{
    if (mode == SHOOT_START)
    {
        motor[0].set_speed =-friction_wheel_speed;
        motor[1].set_speed = friction_wheel_speed;
    }
    else
    {
        motor[0].set_speed = 0;
        motor[1].set_speed = 0;
    }

    for (uint8_t i = 0; i < 2; i++)
    {
        motor[i].current_set = pid_calc(&motor[i].pid, motor[i].motor_info->speed_rpm, motor[i].set_speed);
    }
}


static void ShootMotorSendCurrent(int16_t fric1_cur, int16_t fric2_cur, int16_t trigger_cur, int16_t magazine_cur)
{
    Motor_SendMessage(shoot_handle.shoot_can, SHOOT_MOTOR_CONTROL_STD_ID, fric1_cur, fric2_cur, trigger_cur, magazine_cur);
}
