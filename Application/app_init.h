#ifndef APP_INIT_H
#define APP_INIT_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_init.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
    CHASSIS_APP = 1,
    GIMBAL_APP,
}AppType_e;

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void AppInit(void);
AppType_e GetAppType(void);

#endif  // APP_INIT_H
