#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#undef assert
#define assert(c) do { if (!(c)) { fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #c); exit(1); } } while (0)
#include "../../Core/Src/foc_control.c"
static int cs = 1, fail_dma, transfers;
static uint8_t command;
static int gates, driver_cs = 1, driver_fail;
static uint16_t driver_regs[8];
void Error_Handler(void) { assert(!"Unexpected Error_Handler"); }
int HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h, uint8_t *tx, uint8_t *rx, uint16_t n, uint32_t timeout)
{
    assert(h == &hspi3 && n == 1 && timeout == 2 && driver_cs == 0);
    uint16_t word = *(uint16_t *)tx;
    unsigned reg = (word >> 11) & 15U;
    assert(reg < 8);
    *(uint16_t *)rx = driver_fail ? 0U : driver_regs[reg];
    if (!(word & 0x8000)) { driver_regs[reg] = word & 0x7ffU; }
    return 0;
}
void HAL_GPIO_WritePin(int port, int pin, int value)
{
    (void)port;
    if (pin == MT6816_CS_Pin) { cs = value; }
    if (pin == DRV_cotr_Pin) { gates = value; }
    if (pin == DRV_CS_Pin) { driver_cs = value; }
}
int HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *h, uint8_t *tx, uint8_t *rx, uint16_t len)
{
    assert(h == &hspi1 && rx == enc_rx && len == 2 && cs == 0);
    assert(tx[1] == 0 && (tx[0] == 0x83 || tx[0] == 0x84));
    command = tx[0];
    ++transfers;
    return fail_dma;
}
static uint16_t packet(unsigned raw, unsigned weak)
{
    uint16_t w = (uint16_t)((raw << 2) | (weak << 1));
    unsigned ones = 0;
    for (unsigned i = 1; i < 16; ++i) { ones += (w >> i) & 1U; }
    return w | (ones & 1U);
}
static void receive(uint16_t w)
{
    assert(command == 0x83);
    enc_rx[0] = 0xff; enc_rx[1] = w >> 8;
    HAL_SPI_TxRxCpltCallback(&hspi1);
    assert(command == 0x84);
    enc_rx[0] = 0xff; enc_rx[1] = w & 0xff;
    HAL_SPI_TxRxCpltCallback(&hspi1);
    assert(command == 0x83);
}
int main(void)
{
    FOC_Init();
    assert(FOC_GetEncoderStatus() == FOC_ENCODER_NOT_READY);
    receive(packet(12000, 0));
    assert(FOC_GetAngle() > FOC_PI && enc_turns == 0);
    receive(packet(16380, 0)); receive(packet(3, 0));
    assert(enc_turns == FOC_2PI);
    receive(packet(16380, 0)); assert(enc_turns == 0);
    for (unsigned raw = 0; raw < 16384; ++raw)
    {
        receive(packet(raw, 0));
        assert(FOC_GetEncoderRawAngle() == raw && FOC_GetEncoderStatus() == 0);
        float last = FOC_GetAngle();
        for (unsigned bit = 0; bit < 16; ++bit)
        {
            receive(packet(raw, 0) ^ (1U << bit));
            assert(FOC_GetEncoderStatus() & FOC_ENCODER_PARITY_ERROR);
            assert(FOC_GetAngle() == last && FOC_GetEncoderRawAngle() == raw);
        }
        receive(packet(raw, 1));
        assert(FOC_GetEncoderStatus() == FOC_ENCODER_NO_MAG);
        assert(FOC_GetAngle() == last);
    }
    receive(packet(4096, 0));
    assert(FOC_GetEncoderStatus() == 0);
    assert(fabsf(angle - FOC_PI / 2) < 0.00001f);
    SPI_HandleTypeDef other = {2};
    int previous = transfers;
    HAL_SPI_TxRxCpltCallback(&other); HAL_SPI_ErrorCallback(&other);
    assert(transfers == previous && FOC_GetEncoderStatus() == 0);
    fail_dma = 1; enc_rx[1] = 0;
    HAL_SPI_TxRxCpltCallback(&hspi1);
    assert(cs == 1 && (FOC_GetEncoderStatus() & FOC_ENCODER_SPI_ERROR));
    enc_status = 0; cs = 0;
    HAL_SPI_ErrorCallback(&hspi1);
    assert(cs == 1 && FOC_GetEncoderStatus() == FOC_ENCODER_SPI_ERROR);
    puts("PASS: 16384 angles; 262144 single-bit errors; weak magnet; wrap; DMA frames and failures.");

    fail_dma = 0;
    enc_status = 0;
    test_ms = foc_started_ms;
    adc_regs.JDR1 = 2000; adc_regs.JDR2 = 2100;
    for (unsigned i = 0; i < 128; ++i) { HAL_ADCEx_InjectedConvCpltCallback(&hadc1); }
    assert(offset_samples == 128 && offset_u == 2000 && offset_v == 2100 && gates == 0);
    enc_last_valid_ms = test_ms;
    startup_tick(); assert(g_foc_debug.startup_state == FOC_STARTUP_WAIT);
    test_ms += 500; enc_last_valid_ms = test_ms;
    startup_tick(); assert(g_foc_debug.startup_state == FOC_STARTUP_ALIGN_HOLD);
    HAL_ADCEx_InjectedConvCpltCallback(&hadc1); assert(gates == 1);
    angle = 1.0f;
    test_ms += 500; enc_last_valid_ms = test_ms;
    startup_tick(); assert(g_foc_debug.startup_state == FOC_STARTUP_ALIGN_SWEEP);
    test_ms += 500; enc_last_valid_ms = test_ms;
    startup_tick(); assert(fabsf(alignment_theta - FOC_PI / 2) < 0.0001f);
    test_ms += 500; enc_last_valid_ms = test_ms;
    startup_tick(); assert(g_foc_debug.startup_state == FOC_STARTUP_ALIGN_SETTLE);
    angle = 1.0f - FOC_PI / 8.0f;
    test_ms += 500; enc_last_valid_ms = test_ms;
    startup_tick(); assert(g_foc_debug.startup_state == FOC_STARTUP_READY);
    assert(g_foc_debug.encoder_direction == -1.0f);
    assert(fabsf(wrap(-angle * 8 - g_foc_debug.electrical_offset - FOC_PI)) < 0.0001f);
#if FOC_DEBUG_ENABLE
    FOC_DebugSetTorque(0.3f); debug_tick();
    assert(g_foc_debug.running && fabsf(iq_ref - 0.03f) < 0.0001f);
    FOC_DebugSetTorque(5.0f); debug_tick(); assert(g_foc_debug.commanded_iq_a == 0.5f);
    FOC_DebugSetTorque(-5.0f); debug_tick(); assert(g_foc_debug.commanded_iq_a == -0.5f);
    FOC_DebugSetSpeed(300.0f); speed = 0; debug_tick();
    assert(g_foc_debug.commanded_iq_a > 0 && g_foc_debug.commanded_iq_a <= 0.5f);
    for (unsigned i = 0; i < 10000; ++i) { debug_tick(); }
    speed = -600 * FOC_2PI / 60; debug_tick();
    assert(g_foc_debug.commanded_iq_a < 0); /* Brake above target; no windup. */
    FOC_DebugSetTorque(NAN); debug_tick(); assert(!g_foc_debug.running && iq_ref == 0);
    FOC_DebugSetTorque(0.3f); debug_tick();
    FOC_DebugStop(); HAL_ADCEx_InjectedConvCpltCallback(&hadc1);
    assert(!g_foc_debug.running && gates == 0);
    FOC_DebugSetTorque(0.3f); debug_tick();
    test_ms += 10; startup_tick(); debug_tick();
    assert(g_foc_debug.startup_state == FOC_STARTUP_FAULT && !g_foc_debug.running && gates == 0);
#endif
    g_foc_debug.startup_state = FOC_STARTUP_ALIGN_SETTLE;
    enc_last_valid_ms = test_ms; startup_phase_ms = test_ms - 500;
    angle = alignment_start_angle; startup_tick();
    assert(g_foc_debug.startup_state == FOC_STARTUP_FAULT);
    assert(g_foc_debug.fault & FOC_FAULT_ALIGNMENT);
    driver_fail = 1; assert(!driver_init());
    assert(g_foc_debug.fault == FOC_FAULT_DRIVER);
    puts("PASS: driver registers; offsets; startup sequencing; direction/zero alignment; current limit; speed PI; stop/faults.");
}
