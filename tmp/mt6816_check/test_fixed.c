#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#undef assert
#define assert(c) do { if (!(c)) { fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #c); exit(1); } } while (0)
#include "../../Core/Src/foc_control.c"

static int encoder_cs = 1, driver_cs = 1, gates, enable, dma_fail;
static uint16_t driver_regs[8];
static uint32_t compare_value[4];
static uint8_t encoder_command;

void Error_Handler(void) { assert(!"unexpected Error_Handler"); }
void test_set_compare(uint32_t channel, uint32_t compare)
{
    assert(channel >= 1 && channel <= 3);
    compare_value[channel] = compare;
}
void HAL_GPIO_WritePin(int port, int pin, int value)
{
    (void)port;
    if (pin == MT6816_CS_Pin) encoder_cs = value;
    if (pin == DRV_CS_Pin) driver_cs = value;
    if (pin == DRV_cotr_Pin) gates = value;
    if (pin == DRV_ENBLE_Pin) enable = value;
}
int HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h, uint8_t *tx, uint8_t *rx,
                           uint16_t count, uint32_t timeout)
{
    assert(h == &hspi3 && count == 1 && timeout == 2 && driver_cs == 0);
    uint16_t word = *(uint16_t *)tx;
    unsigned reg = (word >> 11) & 15U;
    assert(reg < 8);
    *(uint16_t *)rx = driver_regs[reg];
    if (!(word & 0x8000U)) driver_regs[reg] = word & 0x07ffU;
    return HAL_OK;
}
int HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *h, uint8_t *tx,
                               uint8_t *rx, uint16_t count)
{
    assert(h == &hspi1 && rx == enc_rx && count == 2 && encoder_cs == 0);
    assert(tx[1] == 0 && (tx[0] == 0x83 || tx[0] == 0x84));
    encoder_command = tx[0];
    return dma_fail;
}
void HAL_Delay(uint32_t ms)
{
    while (ms--)
    {
        ++test_ms;
        if (foc_state >= FOC_STATE_CALIBRATING && foc_state < FOC_STATE_FAULT)
        {
            enc_initialized = 1U;
            enc_status = 0U;
            enc_last_valid_ms = test_ms;
            if (foc_state == FOC_STATE_ALIGNING)
                angle = 1.0f + align_theta / FOC_MOTOR_POLE_PAIRS;
            HAL_ADCEx_InjectedConvCpltCallback(&hadc1);
        }
    }
}
static uint16_t mt6816_packet(unsigned raw, unsigned weak)
{
    uint16_t word = (uint16_t)((raw << 2) | (weak << 1));
    unsigned ones = 0;
    for (unsigned bit = 1; bit < 16; ++bit) ones += (word >> bit) & 1U;
    return word | (ones & 1U);
}
static void receive_encoder(uint16_t word)
{
    assert(encoder_command == 0x83);
    enc_rx[1] = (uint8_t)(word >> 8);
    HAL_SPI_TxRxCpltCallback(&hspi1);
    assert(encoder_command == 0x84);
    enc_rx[1] = (uint8_t)word;
    HAL_SPI_TxRxCpltCallback(&hspi1);
}
int main(void)
{
    adc_regs.JDR1 = 2040U;
    adc_regs.JDR2 = 2060U;
    FOC_Init();
    assert(enable == 1);
    assert(driver_regs[2] == 0x020U && driver_regs[6] == 0x283U);
    assert(offset_samples == 128U && offset_u == 2040.0f && offset_v == 2060.0f);
    assert(FOC_GetState() == FOC_STATE_RUNNING && FOC_GetFault() == 0U);
    assert(encoder_direction == 1.0f);
    assert(fabsf(wrap(angle * 8.0f - electrical_offset - FOC_PI)) < 0.0001f);

    FOC_SetTorque(0.03f);
    enc_last_valid_ms = test_ms;
    HAL_ADCEx_InjectedConvCpltCallback(&hadc1);
    assert(gates == 1);
    assert(compare_value[1] <= PWM_PERIOD && compare_value[2] <= PWM_PERIOD &&
           compare_value[3] <= PWM_PERIOD);
    pwm(0.2f, 0.5f, 0.8f);
    assert(compare_value[1] >= 1699U && compare_value[1] <= 1700U &&
           compare_value[2] == 1062U &&
           compare_value[3] >= 424U && compare_value[3] <= 425U);

    /* Decoder still covers the complete 14-bit range and rejects corruption. */
    for (unsigned raw = 0; raw < 16384; ++raw)
    {
        receive_encoder(mt6816_packet(raw, 0));
        assert(FOC_GetEncoderRawAngle() == raw && FOC_GetEncoderStatus() == 0U);
    }
    uint16_t previous = FOC_GetEncoderRawAngle();
    receive_encoder(mt6816_packet(previous, 0) ^ 0x20U);
    assert((FOC_GetEncoderStatus() & FOC_ENCODER_PARITY_ERROR) &&
           FOC_GetEncoderRawAngle() == previous);

    test_ms += 10U;
    HAL_ADCEx_InjectedConvCpltCallback(&hadc1);
    assert(FOC_GetState() == FOC_STATE_FAULT);
    assert(FOC_GetFault() & FOC_FAULT_ENCODER);
    assert(gates == 0);
    puts("PASS: DRV config, offset acquisition, 8-pole-pair alignment, torque run, PWM2, MT6816 and stale-angle shutdown.");
    return 0;
}
