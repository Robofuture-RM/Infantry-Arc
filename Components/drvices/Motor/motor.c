/* 包含头文件 ----------------------------------------------------------------*/
#include "motor.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/
MotorInfo_t chassis_motor[4];
MotorInfo_t gimbal_motor_yaw;
MotorInfo_t gimbal_motor_pitch;
MotorInfo_t friction_wheel_motor[2];
MotorInfo_t trigger_motor;
MotorInfo_t Magazine_motor;
/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
static void Motor_EncoderData(MotorInfo_t* ptr, uint8_t data[])
{
    ptr->last_ecd = ptr->ecd;
    ptr->ecd = (uint16_t)(data[0] << 8 | data[1]);

    if (ptr->ecd - ptr->last_ecd > MOTOR_ENCODER_RANGE_HALF)
    {
        ptr->round_cnt--;
        ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - MOTOR_ENCODER_RANGE;
    }
    else if (ptr->ecd - ptr->last_ecd < -MOTOR_ENCODER_RANGE_HALF)
    {
        ptr->round_cnt++;
        ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + MOTOR_ENCODER_RANGE;
    }
    else
    {
        ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
    }

    ptr->total_ecd = ptr->round_cnt * MOTOR_ENCODER_RANGE + ptr->ecd - ptr->offset_ecd;
    /* total angle, unit is degree */
    ptr->total_angle = ptr->total_ecd / ENCODER_ANGLE_RATIO;

    ptr->speed_rpm = (int16_t)(data[2] << 8 | data[3]);
    ptr->given_current = (int16_t)(data[4] << 8 | data[5]);
    ptr->temperature = data[6];
}

static void Motor_EncoderOffset(MotorInfo_t* ptr, uint8_t data[])
{
    ptr->ecd        = (uint16_t)(data[0] << 8 | data[1]);
    ptr->offset_ecd = ptr->ecd;
}

/* 电调返回数据处理 */
void Motor_DataParse(MotorInfo_t *ptr, uint8_t data[])
{
    if (ptr == NULL)
        return;
    ptr->msg_cnt++;

    if (ptr->msg_cnt < 50)
    {
        Motor_EncoderOffset(ptr, data);
        return;
    }

    Motor_EncoderData(ptr, data);
}

int16_t Motor_RelativePosition(int16_t ecd, int16_t center_offset)
{
    int16_t tmp = 0;
    if (center_offset >= MOTOR_ENCODER_RANGE_HALF)
    {
        if (ecd > center_offset - MOTOR_ENCODER_RANGE_HALF)
            tmp = ecd - center_offset;
        else
            tmp = ecd + MOTOR_ENCODER_RANGE - center_offset;
    }
    else
    {
        if (ecd > center_offset + MOTOR_ENCODER_RANGE_HALF)
            tmp = ecd - MOTOR_ENCODER_RANGE - center_offset;
        else
            tmp = ecd - center_offset;
    }
    return tmp;
}

/* 发送电机数据 */
void Motor_SendMessage(CAN_Object_t *obj, uint32_t std_id, int16_t cur1, int16_t cur2, int16_t cur3, int16_t cur4)
{
    uint8_t TxData[8] = {0};
    TxData[0] = (uint8_t)(cur1 >> 8);
    TxData[1] = (uint8_t)cur1;
    TxData[2] = (uint8_t)(cur2 >> 8);
    TxData[3] = (uint8_t)cur2;
    TxData[4] = (uint8_t)(cur3 >> 8);
    TxData[5] = (uint8_t)cur3;
    TxData[6] = (uint8_t)(cur4 >> 8);
    TxData[7] = (uint8_t)cur4;
    BSP_CAN_TransmitData(obj, std_id, TxData, 8);
}

/* 电机快速设置ID */
void Motor_QuicklySetID(CAN_Object_t *obj)
{
    uint8_t TxData[8] = {0};
    BSP_CAN_TransmitData(obj, 0x700, TxData, 8);
}

MotorInfo_t* ChassisMotor_Pointer(uint8_t i)
{
    return &chassis_motor[i];
}

MotorInfo_t* GimbalMotorYaw_Pointer(void)
{
    return &gimbal_motor_yaw;
}

MotorInfo_t* GimbalMotorPitch_Pointer(void)
{
    return &gimbal_motor_pitch;
}

MotorInfo_t* FrictionWheelMotor_1_Pointer(void)
{
    return &friction_wheel_motor[0];
}

MotorInfo_t* FrictionWheelMotor_2_Pointer(void)
{
    return &friction_wheel_motor[1];
}

MotorInfo_t* TriggerMotor_Pointer(void)
{
    return &trigger_motor;
}

MotorInfo_t* MagazineMotor_Pointer(void)
{
    return &Magazine_motor;
}
