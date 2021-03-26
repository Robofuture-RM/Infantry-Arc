/* 包含头文件 ----------------------------------------------------------------*/
#include "infantry_console.h"
#include "infantry_def.h"
#include "cmsis_os.h"
#include "RemoteControl/remote_control.h"
#include "user_protocol.h"
#include "detect_task.h"
#include "ramp.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
osThreadId ConsoleTaskHandle;
#if INCLUDE_uxTaskGetStackHighWaterMark
static uint32_t console_task_stack = 0;
#endif

Console_t console;
RC_Info_t last_rc;
RC_Switch_t wheel_switch;

ramp_v0_t front_back_ramp = RAMP_GEN_DAFAULT;
ramp_v0_t left_right_ramp = RAMP_GEN_DAFAULT;

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/
static void RemoteControlWheelAction(void);
static void RemoteControl_Operation(void);
static void Keyboard_Operation(void);
static void Other_Operation(void);

/* 函数体 --------------------------------------------------------------------*/
void ConsoleTask(void *argument)
{
    for(;;)
    {
        RemoteControlWheelAction();
        switch (console.ctrl_mode)
        {
            case PREPARE_MODE:
            {
                if(GimbalInfo_Pointer()->mode != GIMBAL_INIT
                        && GimbalInfo_Pointer()->mode != GIMBAL_RELAX)
                {
                    console.ctrl_mode = NORMAL_MODE;
                    console.gimbal_cmd = GIMBAL_NORMAL_CMD;
                    console.chassis_cmd = CHASSIS_FOLLOW_GIMBAL_CMD;
                    console.shoot_cmd = SHOOT_STOP_CMD;
                }
                else
                {
                    console.gimbal_cmd  = GIMBAL_INIT_CMD;
                    console.chassis_cmd = CHASSIS_STOP_CMD;
                    console.shoot_cmd = SHOOT_STOP_CMD;
                }
            }break;
            case NORMAL_MODE:
            {
                if (console.rc->sw1 == REMOTE_SWITCH_VALUE_CENTRAL)
                {
                    RemoteControl_Operation();
                }
                else if(console.rc->sw1 == REMOTE_SWITCH_VALUE_UP)
                {
                    Keyboard_Operation();
                }
                else if(console.rc->sw1 == REMOTE_SWITCH_VALUE_DOWN)
                {
                    Other_Operation();
                }
            }break;
            case SAFETY_MODE:
            {
                if(!CheckDeviceIsOffline(OFFLINE_DBUS))
                {
                    console.ctrl_mode = PREPARE_MODE;
                }
                else
                {
                    console.gimbal_cmd  = GIMBAL_RELEASE_CMD;
                    console.chassis_cmd  = CHASSIS_RELEASE_CMD;
                    console.shoot_cmd = SHOOT_RELEASE_CMD;
                }
            }break;
            default:
                break;
        }

        if(CheckDeviceIsOffline(OFFLINE_DBUS))
        {
            console.ctrl_mode = SAFETY_MODE;
        }
        last_rc = *console.rc;
        osDelay(CONSOLE_TASK_PERIOD);
#if INCLUDE_uxTaskGetStackHighWaterMark
        console_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

void ConsoleTaskInit(void)
{
    console.rc = RC_GetDataPointer();
    console.ctrl_mode = PREPARE_MODE;
    console.chassis_cmd = CHASSIS_STOP_CMD;
    console.gimbal_cmd = GIMBAL_INIT_CMD;
    console.shoot_cmd = SHOOT_STOP_CMD;
    console.shoot.fire_cmd = STOP_FIRE_CMD;

    ramp_v0_init(&front_back_ramp, CHASSIS_ACCEL_TIME/CONSOLE_TASK_PERIOD);
    ramp_v0_init(&left_right_ramp, CHASSIS_ACCEL_TIME/CONSOLE_TASK_PERIOD);

    osThreadDef(console_task, ConsoleTask, osPriorityNormal, 0, 256);
    ConsoleTaskHandle = osThreadCreate(osThread(console_task), NULL);
}

Console_t* Console_Pointer(void)
{
    return &console;
}

static void RemoteControlWheelAction(void)
{
    static uint8_t wheel_sw = REMOTE_SWITCH_VALUE_CENTRAL;
    if (console.rc->wheel < -440)
    {
        wheel_sw = REMOTE_SWITCH_VALUE_UP;
    }
    else if (console.rc->wheel > -220 && console.rc->wheel < 220)
    {
        wheel_sw = REMOTE_SWITCH_VALUE_CENTRAL;
    }
    else if (console.rc->wheel > 440)
    {
        wheel_sw = REMOTE_SWITCH_VALUE_DOWN;
    }
    RC_SwitchAction(&wheel_switch, wheel_sw);
}

static void RemoteControl_Operation(void)
{
    static uint32_t shoot_time = 0;
    if (console.rc->sw2 == REMOTE_SWITCH_VALUE_UP)
    {
        console.gimbal_cmd = GIMBAL_RELATIVE_CMD;
        console.chassis_cmd = CHASSIS_SEPARATE_GIMBAL_CMD;

        console.chassis.vx = console.rc->ch2 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_X;
        console.chassis.vw = console.rc->ch1 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_R;
        console.gimbal.pitch_v = console.rc->ch4 * RC_GIMBAL_MOVE_RATIO_PIT;
        console.gimbal.yaw_v = -console.rc->ch3 * RC_GIMBAL_MOVE_RATIO_YAW;
    }
    else if(console.rc->sw2 == REMOTE_SWITCH_VALUE_CENTRAL)
    {
        console.gimbal_cmd = GIMBAL_NORMAL_CMD;
        console.chassis_cmd = CHASSIS_FOLLOW_GIMBAL_CMD;

        console.chassis.vx = console.rc->ch2 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_X;
        console.chassis.vy = console.rc->ch1 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_Y;
        console.gimbal.pitch_v = console.rc->ch4 * RC_GIMBAL_MOVE_RATIO_PIT;
        console.gimbal.yaw_v = -console.rc->ch3 * RC_GIMBAL_MOVE_RATIO_YAW;
    }
    else if(console.rc->sw2 == REMOTE_SWITCH_VALUE_DOWN)
    {
        console.gimbal_cmd = GIMBAL_NORMAL_CMD;
        console.chassis_cmd = CHASSIS_SPIN_CMD;

        console.chassis.vx = console.rc->ch2 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_X;
        console.chassis.vy = console.rc->ch1 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_Y;
        console.gimbal.pitch_v = console.rc->ch4 * RC_GIMBAL_MOVE_RATIO_PIT;
        console.gimbal.yaw_v = -console.rc->ch3 * RC_GIMBAL_MOVE_RATIO_YAW;
    }

    if (console.shoot_cmd == SHOOT_STOP_CMD)
    {
        if (wheel_switch.switch_state == REMOTE_SWITCH_CHANGE_3TO1)
        {
            console.shoot_cmd = SHOOT_START_CMD;
        }
    }
    else if (console.shoot_cmd == SHOOT_START_CMD)
    {
        if (wheel_switch.switch_state == REMOTE_SWITCH_CHANGE_3TO1)
        {
            console.shoot_cmd = SHOOT_STOP_CMD;
        }
        else if (wheel_switch.switch_state == REMOTE_SWITCH_CHANGE_3TO2)
        {
            console.shoot.fire_cmd = ONE_FIRE_CMD;
        }
        else if (wheel_switch.switch_value_raw == REMOTE_SWITCH_VALUE_DOWN)
        {
            shoot_time++;
            if(shoot_time > 50)
                console.shoot.fire_cmd = ONE_FIRE_CMD;
            else
                console.shoot.fire_cmd = STOP_FIRE_CMD;
        }
        else
        {
            console.shoot.fire_cmd = STOP_FIRE_CMD;
            shoot_time = 0;
        }
    }
}

static void Keyboard_Operation(void)
{
    fp32 chassis_vx = 0;
    fp32 chassis_vy = 0;

    if (console.rc->kb.bit.F)
    {
        console.gimbal_cmd = GIMBAL_NORMAL_CMD;
        console.chassis_cmd = CHASSIS_FOLLOW_GIMBAL_CMD;
    }
    else if (console.rc->kb.bit.C)
    {
        console.gimbal_cmd = GIMBAL_NORMAL_CMD;
        console.chassis_cmd = CHASSIS_SPIN_CMD;
    }


    if (console.rc->kb.bit.SHIFT)
    {
        chassis_vx = KB_CHASSIS_MAX_SPEED_X;
        chassis_vy = KB_CHASSIS_MAX_SPEED_Y;
    }
    else
    {
        chassis_vx = KB_CHASSIS_MAX_SPEED_X * 0.5f;
        chassis_vy = KB_CHASSIS_MAX_SPEED_Y * 0.5f;
    }

    if (console.rc->kb.bit.W)
    {
        console.chassis.vx = chassis_vx * ramp_v0_calculate(&front_back_ramp);
    }
    else if (console.rc->kb.bit.S)
    {
        console.chassis.vx = -chassis_vx * ramp_v0_calculate(&front_back_ramp);
    }
    else
    {
        console.chassis.vx = 0;
        ramp_v0_init(&front_back_ramp, CHASSIS_ACCEL_TIME/CONSOLE_TASK_PERIOD);
    }

    if (console.rc->kb.bit.A)
    {
        console.chassis.vy = chassis_vy * ramp_v0_calculate(&left_right_ramp);
    }
    else if(console.rc->kb.bit.D)
    {
        console.chassis.vy = -chassis_vy * ramp_v0_calculate(&left_right_ramp);
    }
    else
    {
        console.chassis.vy = 0;
        ramp_v0_init(&left_right_ramp, CHASSIS_ACCEL_TIME/CONSOLE_TASK_PERIOD);
    }


    console.gimbal.yaw_v = -console.rc->mouse.x * KB_GIMBAL_MOVE_RATIO_YAW;
    console.gimbal.pitch_v = -console.rc->mouse.y * KB_GIMBAL_MOVE_RATIO_PIT;


    if (console.shoot_cmd == SHOOT_STOP_CMD)
    {
        if (!last_rc.kb.bit.V && console.rc->kb.bit.V)
        {
            console.shoot_cmd = SHOOT_START_CMD;
        }
    }
    else if (console.shoot_cmd == SHOOT_START_CMD)
    {
        if (!last_rc.kb.bit.V && console.rc->kb.bit.V)
        {
            console.shoot_cmd = SHOOT_STOP_CMD;
        }
        else if (console.rc->mouse.l)
        {
            console.shoot.fire_cmd = ONE_FIRE_CMD;
        }
        else if (!last_rc.mouse.r && console.rc->mouse.r)
        {
            console.shoot.fire_cmd = RAPID_FIRE_CMD;
        }
        else
        {
            console.shoot.fire_cmd = STOP_FIRE_CMD;
        }
    }
}

static void Other_Operation(void)
{
    /* 控制数值全部赋值0进行保护 */
    console.chassis.vx = 0;
    console.chassis.vy = 0;
    console.chassis.vw = 0;
    console.gimbal.pitch_v = 0;
    console.gimbal.yaw_v = 0;
    console.chassis_cmd = CHASSIS_RELEASE_CMD;
    console.gimbal_cmd = GIMBAL_RELEASE_CMD;
    console.shoot_cmd = SHOOT_RELEASE_CMD;
}
