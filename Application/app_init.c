/* 包含头文件 ----------------------------------------------------------------*/
#include "app_init.h"
#include "Buzzer/buzzer.h"
#include "imu_task.h"
#include "comm_task.h"
#include "timer_task.h"
#include "detect_task.h"

#include "chassis/chassis_app.h"
#include "chassis/chassis_task.h"
#include "gimbal/gimbal_app.h"
#include "gimbal/gimbal_task.h"
#include "gimbal/shoot_task.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
AppType_e app_type;

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/
void StartMusic(void);

/* 函数体 --------------------------------------------------------------------*/
void AppInit(void)
{
    BSP_Init();

    if (BSP_GPIO_ReadPin(&app_gpio))
    {
        app_type = GIMBAL_APP;
    }
    else
    {
        app_type = CHASSIS_APP;
    }

    if (app_type == GIMBAL_APP)
    {
        HAL_Delay(1000);
    }
    StartMusic();

    SoftwareTimerTaskInit();
    IMU_TaskInit();
    ConsoleTaskInit();
    Comm_TaskInit();
    DetectTaskInit();

    if (app_type == GIMBAL_APP)
    {
        GimbalAppConfig();
        ShootTaskInit();
        GimbalTaskInit();
    }
    else if (app_type == CHASSIS_APP)
    {
        ChassisAppConfig();
        ChassisTaskInit();
    }
}

AppType_e GetAppType(void)
{
    return app_type;
}

void StartMusic(void)
{
    Buzzer_SetBeep(SO, 150);
    HAL_Delay(100);
    Buzzer_SetBeep(0, 0);
    HAL_Delay(100);
    Buzzer_SetBeep(FA, 150);
    HAL_Delay(100);
    Buzzer_SetBeep(0, 0);
    HAL_Delay(100);
    Buzzer_SetBeep(LA, 150);
    HAL_Delay(100);
    Buzzer_SetBeep(0, 0);
    HAL_Delay(100);
}
