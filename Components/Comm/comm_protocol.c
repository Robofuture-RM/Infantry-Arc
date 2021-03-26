/* 包含头文件 ----------------------------------------------------------------*/
#include "comm_protocol.h"
#include "crc.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
static LIST_HEAD(receive_head);
static LIST_HEAD(transmit_head);
/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/
static void Comm_Unpack(ReceiveHandle_t* p_handle);
static void UnpackDataSolve(ReceiveHandle_t* p_handle, uint8_t *p_frame);

/* 函数体 --------------------------------------------------------------------*/
static void Comm_Unpack(ReceiveHandle_t* p_handle)
{
    uint8_t byte = 0;
    while (fifo_s_used(&p_handle->fifo))
    {
        byte = fifo_s_get(&p_handle->fifo);
        switch (p_handle->unpack_step)
        {
            case STEP_HEADER_SOF:
            {
                if (byte == p_handle->header_sof)
                {
                    p_handle->unpack_step = STEP_LENGTH_LOW;
                    p_handle->protocol_packet[p_handle->index++] = byte;
                }
                else
                {
                    p_handle->index = 0;
                }
            }
            break;
            case STEP_LENGTH_LOW:
            {
                p_handle->data_len = byte;
                p_handle->protocol_packet[p_handle->index++] = byte;
                p_handle->unpack_step = STEP_LENGTH_HIGH;
            }
            break;

            case STEP_LENGTH_HIGH:
            {
                p_handle->data_len |= (byte << 8);
                p_handle->protocol_packet[p_handle->index++] = byte;

                if (p_handle->data_len < (PROTOCOL_FRAME_MAX_SIZE - HEADER_CRC_CMDID_LEN))
                {
                    p_handle->unpack_step = STEP_FRAME_SEQ;
                }
                else
                {
                    p_handle->unpack_step = STEP_HEADER_SOF;
                    p_handle->index = 0;
                }
            }
            break;

            case STEP_FRAME_SEQ:
            {
                p_handle->protocol_packet[p_handle->index++] = byte;
                p_handle->unpack_step = STEP_HEADER_CRC8;
            }
            break;

            case STEP_HEADER_CRC8:
            {
                p_handle->protocol_packet[p_handle->index++] = byte;

                if (p_handle->index == PROTOCOL_HEADER_SIZE)
                {
                    if (verify_crc8_check_sum(p_handle->protocol_packet, PROTOCOL_HEADER_SIZE))
                    {
                        p_handle->unpack_step = STEP_DATA_CRC16;
                    }
                    else
                    {
                        p_handle->unpack_step = STEP_HEADER_SOF;
                        p_handle->index = 0;
                    }
                }
            }
            break;

            case STEP_DATA_CRC16:
            {
                if (p_handle->index < (HEADER_CRC_CMDID_LEN + p_handle->data_len))
                {
                    p_handle->protocol_packet[p_handle->index++] = byte;
                }
                if (p_handle->index >= (HEADER_CRC_CMDID_LEN + p_handle->data_len))
                {
                    p_handle->unpack_step = STEP_HEADER_SOF;
                    p_handle->index = 0;

                    if (verify_crc16_check_sum(p_handle->protocol_packet, HEADER_CRC_CMDID_LEN + p_handle->data_len))
                    {
                        UnpackDataSolve(p_handle, p_handle->protocol_packet);
                    }
                }
            }
            break;

            default:
            {
                p_handle->unpack_step = STEP_HEADER_SOF;
                p_handle->index = 0;
            }
            break;
        }
    }
}

static void UnpackDataSolve(ReceiveHandle_t* p_handle, uint8_t *p_frame)
{
    FrameHeader_t *p_header = (FrameHeader_t*)p_frame;

//    uint16_t sof         = p_header->sof;
    uint16_t data_length = p_header->data_length;
    uint16_t cmd_id      = *(uint16_t *)(p_frame + PROTOCOL_HEADER_SIZE);
    uint8_t *data_addr   = p_frame + PROTOCOL_HEADER_SIZE + PROTOCOL_CMD_SIZE;

    if(p_handle->func != NULL)
    {
        p_handle->func(cmd_id, data_addr, data_length);
    }
}

static uint16_t Comm_Pack(TransmitHandle_t* p_handle, uint8_t sof, uint16_t cmd_id, uint8_t *p_data, uint16_t len)
{
    uint16_t frame_length = PROTOCOL_HEADER_SIZE + PROTOCOL_CMD_SIZE + len + PROTOCOL_CRC16_SIZE;
    if (frame_length > PROTOCOL_FRAME_MAX_SIZE)
        return 0;
    FrameHeader_t *p_header = (FrameHeader_t*)(p_handle->protocol_packet);

    p_header->sof          = sof;
    p_header->data_length  = len;
    p_header->seq          = p_handle->seq++;
    memcpy(p_handle->protocol_packet + PROTOCOL_HEADER_SIZE, (uint8_t*)&cmd_id, PROTOCOL_CMD_SIZE);
    append_crc8_check_sum(p_handle->protocol_packet, PROTOCOL_HEADER_SIZE);
    memcpy(p_handle->protocol_packet + PROTOCOL_HEADER_SIZE + PROTOCOL_CMD_SIZE, p_data, len);
    append_crc16_check_sum(p_handle->protocol_packet, frame_length);

    return frame_length;
}

void Comm_ReceiveInit(ReceiveHandle_t* p_handle, uint8_t header_sof, uint8_t *rx_fifo_buffer, uint16_t rx_fifo_size, UnpackHookFunc_t func)
{
    if (p_handle == NULL)
        return;
    list_t *cur;
    list_for_each_prev(cur, &receive_head)
    {
        if (cur == &p_handle->list)
            return;
    }
    list_add(&p_handle->list, &receive_head);
    p_handle->header_sof = header_sof;
    fifo_s_init(&p_handle->fifo, rx_fifo_buffer, rx_fifo_size);
    p_handle->func = func;
}

void Comm_ReceiveData(ReceiveHandle_t* p_handle, uint8_t *p_data, uint16_t len)
{
    if (p_handle == NULL)
        return;

    if (fifo_s_free(&p_handle->fifo) > len)
        fifo_s_puts(&p_handle->fifo, (char *)p_data, len);
}

void Comm_ReceiveDataHandler(void)
{
    ReceiveHandle_t* p_handle;
    list_t *cur_pos;
    list_for_each_prev(cur_pos, &receive_head)
    {
        p_handle = (ReceiveHandle_t *)cur_pos;
        Comm_Unpack(p_handle);
    }
}

void Comm_TransmitInit(TransmitHandle_t* p_handle, uint8_t *tx_fifo_buffer, uint16_t tx_fifo_size, TransmitHookFunc_t func)
{
    if (p_handle == NULL)
        return;
    list_t *cur;
    list_for_each_prev(cur, &transmit_head)
    {
        if (cur == &p_handle->list)
            return;
    }
    list_add(&p_handle->list, &transmit_head);
    fifo_s_init(&p_handle->fifo, tx_fifo_buffer, tx_fifo_size);
    p_handle->func = func;
}

void Comm_TransmitData(TransmitHandle_t* p_handle, uint8_t header_sof, uint16_t cmd_id, uint8_t *p_data, uint16_t len)
{
    if (p_handle == NULL)
        return;

    if (fifo_s_free(&p_handle->fifo) > len)
    {
        uint16_t frame_length = Comm_Pack(p_handle, header_sof, cmd_id, p_data, len);
        fifo_s_puts(&p_handle->fifo, (char *)p_handle->protocol_packet, frame_length);
    }
}

void Comm_TransmitDataHandler(void)
{
    uint8_t buff[PROTOCOL_FRAME_MAX_SIZE] = {0};
    TransmitHandle_t* p_handle;
    list_t* cur_pos;
    uint16_t used_len = 0;
    uint16_t free_len = 0;
    uint16_t send_len = 0;

    list_for_each_prev(cur_pos, &transmit_head)
    {
        p_handle = (TransmitHandle_t *)cur_pos;
        used_len = fifo_s_used(&p_handle->fifo);
        if (used_len)
        {
            if (p_handle->get_free_func != NULL)
            {
                free_len = p_handle->get_free_func();
                send_len = (free_len < used_len) ? free_len : used_len;
            }
            else
            {
                send_len = used_len;
            }

            if (send_len > PROTOCOL_FRAME_MAX_SIZE)
            {
                send_len = PROTOCOL_FRAME_MAX_SIZE;
            }

            fifo_s_gets(&p_handle->fifo, (char *)buff, send_len);
            if(p_handle->func != NULL)
            {
                p_handle->func(buff, send_len);
            }
        }
    }
}



