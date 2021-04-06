#ifndef BSP_UART_H
#define BSP_UART_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "main.h"
#include "bsp_def.h"
#include "usart.h"
#include "fifo.h"

/* 类型定义 ------------------------------------------------------------------*/
/* 串口接收回调函数指针 */
typedef void (*UART_RxCallback_t)(uint8_t* buffer, uint16_t len);

typedef struct
{
    UART_HandleTypeDef* huart;          /* Handle */
    BSP_FunctionalStatus_e rx_en;       /* 接收使能 */
    uint16_t rx_buffer_size;            /* 接收缓存字节 */
    uint8_t* rx_buffer;                 /* 接收缓存  */
    UART_RxCallback_t rx_callback;      /* 接收回调 */
    BSP_FunctionalStatus_e tx_en;       /* 发送使能 */
    uint8_t* tx_buffer;                 /* DMA发送缓存 */
    uint16_t tx_buffer_size;            /* 发送缓存字节 */
    fifo_s_t tx_fifo;                   /* 发送FIFO */
    uint8_t* tx_fifo_buffer;            /* 发送FIFO缓存 */
    uint16_t tx_fifo_size;              /* 发送FIFO字节 */
    uint8_t is_sending;                 /* 发送状态标记 */
} UART_Object_t;

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
BSP_Status_e BSP_UART_NewObject(UART_Object_t* obj, UART_HandleTypeDef* huart);
BSP_Status_e BSP_UART_TransmitConfig(UART_Object_t* obj, uint8_t* tx_buffer, uint16_t tx_buffer_size, uint8_t* tx_fifo_buffer, uint16_t tx_fifo_size);
BSP_Status_e BSP_UART_ReceiveConfig(UART_Object_t* obj, uint8_t* rx_buffer, uint16_t rx_buffer_size, UART_RxCallback_t fun);
BSP_Status_e BSP_UART_SetRxCallback(UART_Object_t* obj, UART_RxCallback_t fun);
BSP_Status_e BSP_UART_WriteData(UART_HandleTypeDef* huart, uint8_t* data, uint16_t len);
BSP_Status_e BSP_UART_TransmitData(UART_Object_t* obj, uint8_t* data, uint16_t len);
void BSP_UART_IDLE_Callback(UART_HandleTypeDef* huart);

#endif /* BSP_UART_H */


