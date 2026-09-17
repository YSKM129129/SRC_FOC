#include "foc_can.h"
#include "fdcan.h"
#include "foc_control.h"
#include <math.h>
#include <stdint.h>

#define FOC_CAN_COMMAND_ID \
    (FOC_CAN_COMMAND_BASE + FOC_CAN_NODE_ID)
#define FOC_CAN_TELEMETRY_ID \
    (FOC_CAN_TELEMETRY_BASE + FOC_CAN_NODE_ID)
#define FOC_CAN_BROADCAST_COMMAND_ID \
    (FOC_CAN_COMMAND_BASE + 0x0FU)

static int16_t foc_can_read_i16(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static void foc_can_write_i16(uint8_t *data, int16_t value)
{
    uint16_t raw = (uint16_t)value;
    data[0] = (uint8_t)(raw & 0xFFU);
    data[1] = (uint8_t)(raw >> 8);
}

HAL_StatusTypeDef FOC_Can_Init(void)
{
    FDCAN_FilterTypeDef filter = {0};

    /* Accept command IDs 0x200..0x20F; the callback selects this node or broadcast. */
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = FOC_CAN_COMMAND_BASE;
    filter.FilterID2 = 0x7F0U;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                       0U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_FDCAN_Start(&hfdcan1);
}

HAL_StatusTypeDef FOC_Can_SendTelemetry(void)
{
    FDCAN_TxHeaderTypeDef header = {0};
    uint8_t data[8] = {0};
    float angle = FOC_GetAngle();
    float angle_one_turn;
    int16_t angle_raw;
    int16_t speed_raw;
    int16_t iq_raw;
    int16_t id_raw;

    /* Encode one-turn angle in 0.001 rad and currents in 0.01 A. */
    angle_one_turn = fmodf(angle, FOC_2PI);
    if (angle_one_turn < 0.0f)
    {
        angle_one_turn += FOC_2PI;
    }

    angle_raw = (int16_t)(angle_one_turn * 1000.0f);
    speed_raw = (int16_t)(FOC_GetSpeed() * 10.0f);
    iq_raw = (int16_t)(FOC_GetIq() * 100.0f);
    id_raw = (int16_t)(FOC_GetId() * 100.0f);

    foc_can_write_i16(&data[0], angle_raw);
    foc_can_write_i16(&data[2], speed_raw);
    foc_can_write_i16(&data[4], iq_raw);
    foc_can_write_i16(&data[6], id_raw);

    header.Identifier = FOC_CAN_TELEMETRY_ID;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, data);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef header = {0};
    uint8_t data[8] = {0};

    if (hfdcan != &hfdcan1 ||
        (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
    {
        return;
    }

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
    {
        return;
    }

    if (header.IdType == FDCAN_STANDARD_ID &&
        (header.Identifier == FOC_CAN_COMMAND_ID ||
         header.Identifier == FOC_CAN_BROADCAST_COMMAND_ID) &&
        header.DataLength >= FDCAN_DLC_BYTES_4)
    {
        /* Bytes 2..3 are signed Iq in 10 mA/count, little-endian. */
        int16_t iq_command = foc_can_read_i16(&data[2]);
        FOC_SetTorque((float)iq_command * 0.01f);
    }
}
