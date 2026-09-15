#include "foc_spi2_link.h"
#include "foc_control.h"
#include "gpio.h"
#include "spi.h"
#include <math.h>
#include <stdint.h>

static uint16_t tx_frame[FOC_SPI2_FRAME_WORDS];
static uint16_t rx_frame[FOC_SPI2_FRAME_WORDS];
static uint16_t sequence;
static uint32_t rx_count;

static uint16_t frame_crc(const uint16_t *frame)
{
    uint16_t crc = 0U;
    uint32_t index;

    for (index = 0U; index < FOC_SPI2_FRAME_WORDS - 1U; index++)
    {
        crc ^= frame[index];
    }

    return crc;
}

static int16_t float_to_i16(float value, float scale)
{
    float scaled = value * scale;

    if (scaled > 32767.0f)
    {
        return 32767;
    }
    if (scaled < -32768.0f)
    {
        return -32768;
    }

    return (int16_t)scaled;
}

static void prepare_telemetry_frame(void)
{
    float angle = fmodf(FOC_GetAngle(), FOC_2PI);

    if (angle < 0.0f)
    {
        angle += FOC_2PI;
    }

    tx_frame[0] = FOC_SPI2_MAGIC;
    tx_frame[1] = sequence++;
    tx_frame[2] = (uint16_t)float_to_i16(angle, 1000.0f);
    tx_frame[3] = (uint16_t)float_to_i16(FOC_GetSpeed(), 10.0f);
    tx_frame[4] = (uint16_t)float_to_i16(FOC_GetIq(), 100.0f);
    tx_frame[5] = (uint16_t)float_to_i16(FOC_GetId(), 100.0f);
    tx_frame[6] = 0U; /* Reserved status/fault flags. */
    tx_frame[7] = frame_crc(tx_frame);
}

static void consume_command_frame(void)
{
    int16_t iq_command;

    if (rx_frame[0] != FOC_SPI2_MAGIC ||
        rx_frame[7] != frame_crc(rx_frame))
    {
        return;
    }

    /* Main MCU command: word 2, -1000..1000 means -1.0..1.0 normalized Iq. */
    iq_command = (int16_t)rx_frame[2];
    FOC_SetTorque((float)iq_command / 1000.0f);
    rx_count++;
}

HAL_StatusTypeDef FOC_Spi2_Exchange(void)
{
    HAL_StatusTypeDef status;

    prepare_telemetry_frame();

    /* PB12 is a manually controlled, active-low chip-select for the slave. */
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(&hspi2,
                                     (uint8_t *)tx_frame,
                                     (uint8_t *)rx_frame,
                                     FOC_SPI2_FRAME_WORDS,
                                     2U);
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

    if (status == HAL_OK)
    {
        consume_command_frame();
    }

    return status;
}

uint32_t FOC_Spi2_GetRxCount(void)
{
    return rx_count;
}
