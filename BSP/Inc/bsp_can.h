#ifndef BSP_CAN_H
#define BSP_CAN_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "main.h"
#include "bsp_def.h"
#include "can.h"
#include "fifo.h"

/* 类型定义 ------------------------------------------------------------------*/
/* CAN接收回调函数指针 */
typedef void (*BSP_CAN_RxCallback_t)(uint32_t std_id, uint8_t* data, uint32_t dlc);

/* CAN发送接收管理对象类型 */
typedef struct 
{
    CAN_HandleTypeDef* hcan;    /* Handle */
    fifo_t tx_fifo;             /* 发送FIFO */
    uint8_t* tx_fifo_buffer;    /* 发送FIFO缓存指针 */
    uint8_t is_sending;         /* 发送状态 */
    BSP_CAN_RxCallback_t rx_callback;     /* 接收函数指针 */
}CAN_Object_t;

/* CAN发送消息类型 */
typedef struct
{
    uint32_t std_id;
    uint8_t dlc;
    uint8_t data[8];
}CAN_TxMsg_t;

/* 宏定义 --------------------------------------------------------------------*/
#define CAN_TX_FIFO_UNIT_NUM (256)      /* CAN发送FIFO单元数 */
#define CAN_TX_FIFO_SIZE (CAN_TX_FIFO_UNIT_NUM * sizeof(CAN_TxMsg_t)) /* CAN发送FIFO缓存字节长 */

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void BSP_CAN_Init(CAN_HandleTypeDef* hcan, uint32_t active_it);
BSP_Status_e BSP_CAN_NewObject(CAN_Object_t* obj, CAN_HandleTypeDef* hcan, uint8_t* tx_fifo_buff, BSP_CAN_RxCallback_t fun);
BSP_Status_e BSP_CAN_SetRxCallback(CAN_Object_t* obj, BSP_CAN_RxCallback_t fun);
BSP_Status_e BSP_CAN_WriteData(CAN_HandleTypeDef* hcan, uint32_t std_id, uint8_t* data, uint16_t len);
BSP_Status_e BSP_CAN_TransmitData(CAN_Object_t* obj, uint32_t std_id, uint8_t* data, uint16_t len);

#endif  // BSP_CAN_H

