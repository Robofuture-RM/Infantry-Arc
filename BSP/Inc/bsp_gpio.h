#ifndef BSP_GPIO_H
#define BSP_GPIO_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "main.h"
#include "bsp_def.h"
#include "gpio.h"
#ifdef HAL_TIM_MODULE_ENABLED
#include "tim.h"
#endif

/* 类型定义 ------------------------------------------------------------------*/
/* GPIO外部中断回调函数指针 */
typedef void (*GPIO_EXIT_Callback_t)(void);

/* GPIO设备种类枚举  */
typedef enum
{
    GPIO_EMPTY_DEV = 0,     /* 未定义GPIO设备  */
    GPIO_OUTPUT_PP_DEV,     /* GPIO推挽输出设备  */
    GPIO_OUTPUT_OD_DEV,     /* GPIO开漏输出设备  */
    GPIO_INPUT_DEV,         /* GPIO输入设备  */
    GPIO_PWM_DEV,           /* PWM设备  */
    GPIO_EXTI_DEV,          /* GPIO外部中断设备 */
} GPIO_Device_e;

/* GPIO管理对象  */
typedef struct
{
    void* handle;       /* 根据类型保存指针  */
    uint32_t value;     /* 根据类型保存控制值  */
    GPIO_Device_e device;       /* 设备类型  */
    GPIO_EXIT_Callback_t callback;      /* 外部中断  */
} GPIO_Object_t;

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
BSP_Status_e BSP_GPIO_NewObject(GPIO_Object_t* obj, GPIO_Device_e dev, GPIO_TypeDef* GPIOx, uint16_t pin);
GPIO_PinState BSP_GPIO_ReadPin(GPIO_Object_t* obj);
void BSP_GPIO_WritePin(GPIO_Object_t* obj, GPIO_PinState pin_state);
void BSP_GPIO_TogglePin(GPIO_Object_t* obj);
BSP_Status_e BSP_GPIO_NewObject_EXTI(GPIO_Object_t* obj, GPIO_TypeDef* GPIOx, uint16_t pin, GPIO_EXIT_Callback_t fun);

#ifdef HAL_TIM_MODULE_ENABLED
BSP_Status_e BSP_GPIO_NewObject_PWM(GPIO_Object_t* obj, TIM_HandleTypeDef* tim, uint32_t channel);
void BSP_GPIO_SetPwmValue(GPIO_Object_t* obj, uint16_t value);
void BSP_GPIO_SetPwmPrescaler(GPIO_Object_t* obj, uint16_t psc);
#endif

#endif /* BSP_GPIO_H */

