/* 包含头文件 ----------------------------------------------------------------*/
#include "chassis_app.h"
#include "infantry_def.h"
#include "app_init.h"

#include "comm_protocol.h"
#include "user_protocol.h"
#include "referee_system.h"
#include "timer_task.h"
#include "detect_task.h"
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
ChassisHandle_t chassis_handle;

static TransmitHandle_t chassis_tx_handle;
static uint8_t chassis_tx_fifo_buffer[GIMBAL_CHASSIS_DATA_FIFO_SIZE];
static ReceiveHandle_t chassis_rx_handle;
static uint8_t chassis_rx_fifo_buffer[GIMBAL_CHASSIS_DATA_FIFO_SIZE];

static TransmitHandle_t referee_tx_handle;
static uint8_t referee_tx_fifo_buffer[REFEREE_SYSTEM_FIFO_SIZE];
static ReceiveHandle_t referee_rx_handle;
static uint8_t referee_rx_fifo_buffer[REFEREE_SYSTEM_FIFO_SIZE];

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/
static void CAN1_UploadDataHook(uint8_t *data, uint16_t len);
static void CAN1_RefereeDataHook(uint8_t *data, uint16_t len);
static int32_t Transmit_RefereeData(void *argc);
static void DBUS_ReceiveCallback(uint8_t* data, uint16_t len);
static void COM1_ReceiveCallback(uint8_t* data, uint16_t len);
static void COM2_ReceiveCallback(uint8_t* data, uint16_t len);
static void CAN1_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc);
static void CAN2_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc);

/* 函数体 --------------------------------------------------------------------*/
void ChassisAppConfig(void)
{
    chassis_handle.console      = Console_Pointer();
    chassis_handle.imu          = IMU_GetDataPointer();
    chassis_handle.chassis_can  = &can2_obj;
    chassis_handle.ctrl_mode  = CHASSIS_RELAX;
    chassis_handle.structure.wheel_perimeter = WHEEL_PERIMETER;
    chassis_handle.structure.wheeltrack = WHEELTRACK;
    chassis_handle.structure.wheelbase = WHEELBASE;
    chassis_handle.structure.rotate_x_offset = GIMBAL_X_OFFSET;
    chassis_handle.structure.rotate_y_offset = GIMBAL_Y_OFFSET;
    for (uint8_t i=0; i<4; i++)
    {
        chassis_handle.chassis_motor[i].motor_info = ChassisMotor_Pointer(i);
        pid_init(&chassis_handle.chassis_motor[i].pid, POSITION_PID, M3508_MOTOR_MAX_CURRENT, 2000.0f,
                 6.5f, 0.1f, 0.0f);
    }
    pid_init(&chassis_handle.chassis_follow_pid, POSITION_PID, 300.0f, 50.0f,
             8.0f, 0.0f, 2.0f);

    /*--------------------event-----------------|-------enable-------|-offline time-|-beep_times-*/
    OfflineHandle_Init(OFFLINE_CHASSIS_MOTOR1,  OFFLINE_ERROR_LEVEL,       100,         1);
    OfflineHandle_Init(OFFLINE_CHASSIS_MOTOR2,  OFFLINE_ERROR_LEVEL,       100,         2);
    OfflineHandle_Init(OFFLINE_CHASSIS_MOTOR3,  OFFLINE_ERROR_LEVEL,       100,         3);
    OfflineHandle_Init(OFFLINE_CHASSIS_MOTOR4,  OFFLINE_ERROR_LEVEL,       100,         4);
    OfflineHandle_Init(OFFLINE_REFEREE_SYSTEM,  OFFLINE_WARNING_LEVEL,     100,         1);
    OfflineHandle_Init(OFFLINE_GIMBAL_INFO,     OFFLINE_WARNING_LEVEL,     100,         3);
    OfflineHandle_Init(OFFLINE_DBUS,            OFFLINE_WARNING_LEVEL,     100,         0);

    Comm_TransmitInit(&chassis_tx_handle, chassis_tx_fifo_buffer, GIMBAL_CHASSIS_DATA_FIFO_SIZE, CAN1_UploadDataHook);
    Comm_ReceiveInit(&chassis_rx_handle, USER_PROTOCOL_HEADER_SOF, chassis_rx_fifo_buffer, GIMBAL_CHASSIS_DATA_FIFO_SIZE, UserProtocol_ParseHandler);

    Comm_TransmitInit(&referee_tx_handle, referee_tx_fifo_buffer, REFEREE_SYSTEM_FIFO_SIZE, CAN1_RefereeDataHook);
    Comm_ReceiveInit(&referee_rx_handle, REFEREE_SYSTEM_HEADER_SOF, referee_rx_fifo_buffer, REFEREE_SYSTEM_FIFO_SIZE, RefereeSystem_ParseHandler);
    SoftwareTimerRegister(Transmit_RefereeData, (void*)NULL, 10);

    BSP_UART_SetRxCallback(&dbus_obj, DBUS_ReceiveCallback);
    BSP_UART_SetRxCallback(&com1_obj, COM1_ReceiveCallback);
    BSP_UART_SetRxCallback(&com2_obj, COM2_ReceiveCallback);
    BSP_CAN_SetRxCallback(&can1_obj, CAN1_ReceiveCallback);
    BSP_CAN_SetRxCallback(&can2_obj, CAN2_ReceiveCallback);
}

static void CAN1_UploadDataHook(uint8_t *data, uint16_t len)
{
    BSP_CAN_TransmitData(&can1_obj, CHASSIS_TX_DATA_STD_ID, data, len);
}

static void CAN1_RefereeDataHook(uint8_t *data, uint16_t len)
{
    BSP_CAN_TransmitData(&can1_obj, REFEREE_DATA_STD_ID, data, len);
}

static int32_t Transmit_RefereeData(void *argc)
{
    if (CheckDeviceIsOffline(OFFLINE_REFEREE_SYSTEM))
        return 0;
    ext_game_robot_state_t* robot_state = RefereeSystem_RobotState_Pointer();
    Comm_TransmitData(&referee_tx_handle, REFEREE_SYSTEM_HEADER_SOF, GAME_ROBOT_STATE_CMD_ID, (uint8_t*)robot_state, sizeof(ext_game_robot_state_t));
    ext_power_heat_data_t* power_heat_data = RefereeSystem_PowerHeatData_Pointer();
    Comm_TransmitData(&referee_tx_handle, REFEREE_SYSTEM_HEADER_SOF, POWER_HEAT_DATA_CMD_ID, (uint8_t*)power_heat_data, sizeof(ext_power_heat_data_t));
    return 0;
}

static void DBUS_ReceiveCallback(uint8_t* data, uint16_t len)
{
    RC_DataParser(RC_GetDataPointer(), data, len);
    Comm_TransmitData(&chassis_tx_handle, USER_PROTOCOL_HEADER_SOF, RC_DATA_CMD_ID, data, len);
    OfflineHandle_TimeUpdate(OFFLINE_DBUS);
}

static void COM1_ReceiveCallback(uint8_t* data, uint16_t len)
{
    Comm_ReceiveData(&referee_rx_handle, data, len);
    OfflineHandle_TimeUpdate(OFFLINE_REFEREE_SYSTEM);
}

static void COM2_ReceiveCallback(uint8_t* data, uint16_t len)
{
}

static void CAN1_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc)
{
    switch (std_id)
    {
        case CHASSIS_RX_DATA_STD_ID:
        {
            Comm_ReceiveData(&chassis_rx_handle, data, dlc);
        }break;
        default:
            break;
    }
}

static void CAN2_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc)
{
    switch (std_id)
    {
        case CHASSIS_MOTOR_LF_MESSAGE_ID:
        case CHASSIS_MOTOR_RF_MESSAGE_ID:
        case CHASSIS_MOTOR_LB_MESSAGE_ID:
        case CHASSIS_MOTOR_RB_MESSAGE_ID:
        {
            uint8_t i = std_id - CHASSIS_MOTOR_LF_MESSAGE_ID;
            Motor_DataParse(chassis_handle.chassis_motor[i].motor_info, data);
            OfflineHandle_TimeUpdate(OFFLINE_CHASSIS_MOTOR1+i);
        }break;
        default:
            break;
    }
}
