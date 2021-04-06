/* 包含头文件 ----------------------------------------------------------------*/
#include "blocked.h"
#include "math.h"
#include "main.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
/*************************************************
 * Function: GetTime_ms
 * Description: 获取系统时间
 * Input: 无
 * Return: 系统时间
*************************************************/
static uint32_t GetTime_ms(void)
{
    return HAL_GetTick();
}

/*************************************************
 * Function: Blocked_Reset
 * Description: 堵转处理句柄复位
 * Input: handle 堵转处理句柄
 *        blocked_timer 堵转时间
 *        guard_timer 堵转保护时间
 * Return: 无
*************************************************/
void Blocked_Reset(BlockedHandle_t* handle, uint32_t blocked_timer, uint32_t guard_timer)
{
    handle->start_time = 0;
    handle->blocked_time = 0;
    handle->blocked_timer = blocked_timer;
    handle->guard_timer = guard_timer;
}

/*************************************************
 * Function: Blocked_Process
 * Description: 堵转处理进程
 * Input: handle 堵转处理句柄
 *        speed 电机速度
 * Return: 无
*************************************************/
BlockedState_t Blocked_Process(BlockedHandle_t* handle, fp32 speed)
{
    uint32_t time_now = GetTime_ms();
    if (handle->start_time == 0)
    {
        handle->start_time = time_now;
        handle->turn_time = time_now;
        return NO_BLOCKED;
    }
    else if (time_now - handle->start_time > handle->guard_timer)
    {
        handle->blocked_time = time_now;
        return BLOCKED;
    }

    if ( fabs(speed) < 1.0f)
    {
        if (time_now - handle->turn_time > handle->blocked_timer)
        {
            handle->blocked_time = time_now;
            return BLOCKED;
        }
    }
    else
    {
        handle->turn_time = time_now;
    }
    return NO_BLOCKED;
}

