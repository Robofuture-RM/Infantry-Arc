/* 包含头文件 ----------------------------------------------------------------*/
#include "gimbal_app.h"
#include "infantry_def.h"
#include "app_init.h"

#include "comm_protocol.h"
#include "referee_system.h"
#include "user_protocol.h"

#include "timer_task.h"
#include "detect_task.h"
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
GimbalHandle_t gimbal_handle;

static TransmitHandle_t gimbal_tx_handle;
static uint8_t gimbal_tx_fifo_buffer[GIMBAL_CHASSIS_DATA_FIFO_SIZE];
static ReceiveHandle_t gimbal_rx_handle;
static uint8_t gimbal_rx_fifo_buffer[GIMBAL_CHASSIS_DATA_FIFO_SIZE];

static ReceiveHandle_t referee_rx_handle;
static uint8_t referee_rx_fifo_buffer[REFEREE_SYSTEM_FIFO_SIZE];

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/
static void CAN1_UploadDataHook(uint8_t *data, uint16_t len);
static int32_t GimbalInfoUploadCallback(void *argc);
static void DBUS_ReceiveCallback(uint8_t* data, uint16_t len);
static void COM1_ReceiveCallback(uint8_t* data, uint16_t len);
static void COM2_ReceiveCallback(uint8_t* data, uint16_t len);
static void CAN1_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc);
static void CAN2_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc);

/* 函数体 --------------------------------------------------------------------*/
void GimbalAppConfig(void)
{
    gimbal_handle.console     = Console_Pointer();
    gimbal_handle.imu  = IMU_GetDataPointer();
    gimbal_handle.gimbal_can  = &can1_obj;
    gimbal_handle.ctrl_mode = GIMBAL_INIT;
    gimbal_handle.yaw_motor.motor_info = GimbalMotorYaw_Pointer();
    gimbal_handle.pitch_motor.motor_info = GimbalMotorPitch_Pointer();
    gimbal_handle.yaw_motor.offset_ecd = 1540;
    gimbal_handle.pitch_motor.offset_ecd = 4400;
    gimbal_handle.yaw_motor.ecd_ratio = YAW_MOTO_POSITIVE_DIR * YAW_REDUCTION_RATIO / ENCODER_ANGLE_RATIO;
    gimbal_handle.pitch_motor.ecd_ratio = PITCH_MOTO_POSITIVE_DIR * PITCH_REDUCTION_RATIO / ENCODER_ANGLE_RATIO;
    gimbal_handle.yaw_motor.max_relative_angle = 90;
    gimbal_handle.yaw_motor.min_relative_angle = -90;
    gimbal_handle.pitch_motor.max_relative_angle = 30;
    gimbal_handle.pitch_motor.min_relative_angle = -25;
    pid_init(&gimbal_handle.yaw_motor.pid.outer_pid, POSITION_PID, 2000.0f, 0.0f,
             40.0f, 0.0f, 40.0f);
    pid_init(&gimbal_handle.yaw_motor.pid.inter_pid, POSITION_PID, GM6020_MOTOR_MAX_CURRENT, 3000.0f,
             60.0f, 0.1f, 40.0f);
    pid_init(&gimbal_handle.pitch_motor.pid.outer_pid, POSITION_PID, 2000.0f, 0.0f,
             40.0f, 0.0f, 0.0f);
    pid_init(&gimbal_handle.pitch_motor.pid.inter_pid, POSITION_PID, GM6020_MOTOR_MAX_CURRENT, 6000.0f,
             60.0f, 0.5f, 0.0f);

    /*--------------------event------------------------|-------enable-------|-offline time-|-beep_times-*/
    OfflineHandle_Init(OFFLINE_GIMBAL_PITCH,            OFFLINE_ERROR_LEVEL,       100,         5);
    OfflineHandle_Init(OFFLINE_GIMBAL_YAW,              OFFLINE_ERROR_LEVEL,       100,         6);
    OfflineHandle_Init(OFFLINE_FRICTION_WHEEL_MOTOR1,   OFFLINE_ERROR_LEVEL,       100,         1);
    OfflineHandle_Init(OFFLINE_FRICTION_WHEEL_MOTOR2,   OFFLINE_ERROR_LEVEL,       100,         2);
    OfflineHandle_Init(OFFLINE_TRIGGER_MOTOR,           OFFLINE_ERROR_LEVEL,       100,         3);
    OfflineHandle_Init(OFFLINE_MAGAZINE_MOTOR,          OFFLINE_ERROR_LEVEL,       100,         4);
    OfflineHandle_Init(OFFLINE_REFEREE_SYSTEM,          OFFLINE_WARNING_LEVEL,     100,         1);
//    OfflineHandle_Init(OFFLINE_CHASSIS_INFO,            OFFLINE_WARNING_LEVEL,     100,         2);
    OfflineHandle_Init(OFFLINE_DBUS,                    OFFLINE_WARNING_LEVEL,     100,         0);

    Comm_TransmitInit(&gimbal_tx_handle, gimbal_tx_fifo_buffer, GIMBAL_CHASSIS_DATA_FIFO_SIZE, CAN1_UploadDataHook);
    Comm_ReceiveInit(&gimbal_rx_handle, USER_PROTOCOL_HEADER_SOF, gimbal_rx_fifo_buffer, GIMBAL_CHASSIS_DATA_FIFO_SIZE, UserProtocol_ParseHandler);
    SoftwareTimerRegister(GimbalInfoUploadCallback, (void*)NULL, GIMBAL_UPLOAD_TIMER_PERIOD);

    Comm_ReceiveInit(&referee_rx_handle, REFEREE_SYSTEM_HEADER_SOF, referee_rx_fifo_buffer, REFEREE_SYSTEM_FIFO_SIZE, RefereeSystem_ParseHandler);

    BSP_UART_SetRxCallback(&dbus_obj, DBUS_ReceiveCallback);
    BSP_UART_SetRxCallback(&com1_obj, COM1_ReceiveCallback);
    BSP_UART_SetRxCallback(&com2_obj, COM2_ReceiveCallback);
    BSP_CAN_SetRxCallback(&can1_obj, CAN1_ReceiveCallback);
    BSP_CAN_SetRxCallback(&can2_obj, CAN2_ReceiveCallback);
}

static void CAN1_UploadDataHook(uint8_t *data, uint16_t len)
{
    BSP_CAN_TransmitData(&can1_obj, GIMBAL_TX_DATA_STD_ID, data, len);
}

static int32_t GimbalInfoUploadCallback(void *argc)
{
    Comm_GimbalInfo_t* info = GimbalInfo_Pointer();
    info->mode = gimbal_handle.ctrl_mode;
    info->pitch_ecd_angle   = gimbal_handle.pitch_motor.sensor.relative_angle;
    info->yaw_ecd_angle     = gimbal_handle.yaw_motor.sensor.relative_angle;
    info->pitch_gyro_angle  = gimbal_handle.pitch_motor.sensor.gyro_angle;
    info->yaw_gyro_angle    = gimbal_handle.yaw_motor.sensor.gyro_angle;
    info->pitch_rate        = gimbal_handle.pitch_motor.sensor.palstance;
    info->yaw_rate          = gimbal_handle.yaw_motor.sensor.palstance;
    Comm_TransmitData(&gimbal_tx_handle, USER_PROTOCOL_HEADER_SOF, GIMBAL_INFO_CMD_ID, (uint8_t*)info, sizeof(Comm_GimbalInfo_t));
    return 0;
}

static void DBUS_ReceiveCallback(uint8_t* data, uint16_t len)
{
    RC_DataParser(RC_GetDataPointer(), data, len);
    Comm_TransmitData(&gimbal_tx_handle, USER_PROTOCOL_HEADER_SOF, RC_DATA_CMD_ID, data, len);
    OfflineHandle_TimeUpdate(OFFLINE_DBUS);
}

static void COM1_ReceiveCallback(uint8_t* data, uint16_t len)
{

}

static void COM2_ReceiveCallback(uint8_t* data, uint16_t len)
{

}

static void CAN1_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc)
{
    switch (std_id)
    {
        case GIMBAL_MOTOR_YAW_MESSAGE_ID:
        {
            Motor_DataParse(gimbal_handle.yaw_motor.motor_info, data);
            OfflineHandle_TimeUpdate(OFFLINE_GIMBAL_YAW);
        }break;
        case GIMBAL_MOTOR_PITCH_MESSAGE_ID:
        {
            Motor_DataParse(gimbal_handle.pitch_motor.motor_info, data);
            OfflineHandle_TimeUpdate(OFFLINE_GIMBAL_PITCH);
        }break;
        case GIMBAL_RX_DATA_STD_ID:
        {
            Comm_ReceiveData(&gimbal_rx_handle, data, dlc);
            OfflineHandle_TimeUpdate(OFFLINE_CHASSIS_INFO);
        }break;
        case REFEREE_DATA_STD_ID:
        {
            Comm_ReceiveData(&referee_rx_handle, data, dlc);
            OfflineHandle_TimeUpdate(OFFLINE_REFEREE_SYSTEM);
        }break;
        default:
            break;
    }
}

static void CAN2_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc)
{
    switch (std_id)
    {
        case FRICTION_WHEEL_1_MESSAGE_ID:
        {
            Motor_DataParse(FrictionWheelMotor_1_Pointer(), data);
            OfflineHandle_TimeUpdate(OFFLINE_FRICTION_WHEEL_MOTOR1);
        }break;
        case FRICTION_WHEEL_2_MESSAGE_ID:
        {
            Motor_DataParse(FrictionWheelMotor_2_Pointer(), data);
            OfflineHandle_TimeUpdate(OFFLINE_FRICTION_WHEEL_MOTOR2);
        }break;
        case TRIGGER_MOTOR_MESSAGE_ID:
        {
            Motor_DataParse(TriggerMotor_Pointer(), data);
            OfflineHandle_TimeUpdate(OFFLINE_TRIGGER_MOTOR);
        }break;
        case MAGAZINE_MOTOR_MESSAGE_ID:
        {
            Motor_DataParse(MagazineMotor_Pointer(), data);
            OfflineHandle_TimeUpdate(OFFLINE_MAGAZINE_MOTOR);
        }break;
        default:
            break;
    }
}
