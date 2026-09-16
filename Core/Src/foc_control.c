#include "foc_control.h"
#include "gpio.h"
#include <math.h>

#define ADC_FS 4095.0f
#define SHUNT_OHM 0.001f
#define AMP_GAIN 20.0f
#define VREF 3.3f
#define PWM_PERIOD 2125U
#define MAX_MOD 0.90f
#define MAX_CURRENT 10.0f
#define ALIGN_CURRENT 0.3f

typedef struct { float kp, ki, integ, out; } pi_t;
static volatile float angle, angle_multi, speed, iq, id, iq_ref;
/* MT6816: two separate 16-clock frames, each sent as two 8-bit bytes. */
static uint8_t enc_rx[2];
static uint8_t enc_tx[2] = {0x83U, 0x00U};
static uint8_t enc_high, enc_read_low, enc_initialized;
static volatile uint16_t enc_raw;
static volatile uint32_t enc_status = FOC_ENCODER_NOT_READY;
static volatile uint32_t enc_last_valid_ms;
static volatile uint32_t foc_state = FOC_STATE_IDLE, foc_fault;
static volatile uint32_t offset_samples;
static float offset_u, offset_v;
static volatile float align_theta;
static float encoder_direction = 1.0f, electrical_offset;
static float enc_prev, enc_turns;
static pi_t pi_q = {0.08f, 0.0008f, 0, 0};
static pi_t pi_d = {0.08f, 0.0008f, 0, 0};

static void stop_fault(uint32_t fault)
{
    foc_fault |= fault;
    foc_state = FOC_STATE_FAULT;
    HAL_GPIO_WritePin(DRV_cotr_GPIO_Port, DRV_cotr_Pin, GPIO_PIN_RESET);
}

static uint32_t encoder_healthy(void)
{
    /* Discard individual corrupt frames, but never use a stale angle. */
    return !(enc_status & (FOC_ENCODER_NOT_READY | FOC_ENCODER_SPI_ERROR)) &&
           (HAL_GetTick() - enc_last_valid_ms < 10U);
}

static HAL_StatusTypeDef driver_transfer(uint16_t command, uint16_t *value)
{
    uint16_t received = 0U;
    /* Mode 1, 16 clocks. CS high >= 400 ns at 170 MHz. */
    for (uint32_t i = 0; i < 128U; ++i) { __NOP(); }
    HAL_GPIO_WritePin(DRV_CS_GPIO_Port, DRV_CS_Pin, GPIO_PIN_RESET);
    for (uint32_t i = 0; i < 32U; ++i) { __NOP(); }
    HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(&hspi3,
        (uint8_t *)&command, (uint8_t *)&received, 1U, 2U);
    for (uint32_t i = 0; i < 32U; ++i) { __NOP(); }
    HAL_GPIO_WritePin(DRV_CS_GPIO_Port, DRV_CS_Pin, GPIO_PIN_SET);
    *value = received & 0x07ffU;
    return result;
}

static uint32_t driver_init(void)
{
    uint16_t value;
    HAL_GPIO_WritePin(DRV_cotr_GPIO_Port, DRV_cotr_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DRV_CS_GPIO_Port, DRV_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DRV_ENBLE_GPIO_Port, DRV_ENBLE_Pin, GPIO_PIN_RESET);
    HAL_Delay(2U);
    HAL_GPIO_WritePin(DRV_ENBLE_GPIO_Port, DRV_ENBLE_Pin, GPIO_PIN_SET);
    HAL_Delay(2U); /* tWAKE/tREADY >= 1 ms. PWM has not started. */
    if (driver_transfer(0x1020U, &value) != HAL_OK || /* reg 2: 3PWM */
        driver_transfer(0x3283U, &value) != HAL_OK || /* reg 6: 20 V/V, VREF/2 */
        driver_transfer(0x9000U, &value) != HAL_OK || value != 0x0020U ||
        driver_transfer(0xb000U, &value) != HAL_OK || value != 0x0283U)
    {
        stop_fault(FOC_FAULT_DRIVER);
        HAL_GPIO_WritePin(DRV_ENBLE_GPIO_Port, DRV_ENBLE_Pin, GPIO_PIN_RESET);
        return 0U;
    }
    return 1U;
}

static void encoder_cs_delay(void)
{
    /* At 170 MHz, 64 NOPs alone exceed 376 ns. Covers TL >= 100 ns,
       TH >= 0.5 SCK (94 ns at /32), and provides a CS-high gap. */
    for (uint32_t i = 0; i < 64U; ++i) { __NOP(); }
}

static void encoder_start_frame(uint8_t command)
{
    enc_tx[0] = command;
    encoder_cs_delay();
    HAL_GPIO_WritePin(MT6816_CS_GPIO_Port, MT6816_CS_Pin, GPIO_PIN_RESET);
    encoder_cs_delay();
    if (HAL_SPI_TransmitReceive_DMA(&hspi1, enc_tx, enc_rx, 2U) != HAL_OK)
    {
        HAL_GPIO_WritePin(MT6816_CS_GPIO_Port, MT6816_CS_Pin, GPIO_PIN_SET);
        enc_status |= FOC_ENCODER_SPI_ERROR;
    }
}

static void encoder_update(uint8_t high, uint8_t low)
{
    uint16_t word = ((uint16_t)high << 8) | low;
    uint16_t parity = word;
    uint32_t status = enc_initialized ? 0U : FOC_ENCODER_NOT_READY;
    /* Even parity covers all 16 bits, including No_Mag_Warning and PC. */
    parity ^= parity >> 8;
    parity ^= parity >> 4;
    parity ^= parity >> 2;
    parity ^= parity >> 1;
    if (parity & 1U) { status |= FOC_ENCODER_PARITY_ERROR; }
    if (low & 2U) { status |= FOC_ENCODER_NO_MAG; }
    if (status & (FOC_ENCODER_PARITY_ERROR | FOC_ENCODER_NO_MAG))
    {
        enc_status = status;
        return; /* Keep the last valid angle; do not feed corrupt data to FOC. */
    }

    uint16_t raw = word >> 2;
    float a = FOC_2PI * (float)raw / 16384.0f;
    if (enc_initialized)
    {
        float d = (float)raw - enc_prev;
        if (d > 8192.0f) { enc_turns -= FOC_2PI; }
        else if (d < -8192.0f) { enc_turns += FOC_2PI; }
    }
    enc_prev = (float)raw;
    enc_raw = raw;
    angle = a;
    angle_multi = enc_turns + a;
    enc_initialized = 1U;
    enc_last_valid_ms = HAL_GetTick();
    enc_status = 0U;
}

static float clamp(float x,float lo,float hi){return x<lo?lo:(x>hi?hi:x);}
static float wrap(float x){ while(x>FOC_PI)x-=FOC_2PI; while(x<-FOC_PI)x+=FOC_2PI; return x; }

static void pwm(float a,float b,float c){
 a=clamp(a,0,MAX_MOD); b=clamp(b,0,MAX_MOD); c=clamp(c,0,MAX_MOD);
 /* PWM2: high-side duty = 1 - CCR/ARR. All low sides conduct at CNT=0. */
 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,(uint32_t)((1.0f-a)*PWM_PERIOD));
 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,(uint32_t)((1.0f-b)*PWM_PERIOD));
 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,(uint32_t)((1.0f-c)*PWM_PERIOD));
}
static void svpwm(float theta,float vd,float vq)
{
    float al=vd*cosf(theta)-vq*sinf(theta),
            be=vd*sinf(theta)+vq*cosf(theta);
    float u=0.5f+0.5f*al,
            v=0.5f+0.5f*(-0.5f*al+0.8660254f*be),
            w=0.5f+0.5f*(-0.5f*al-0.8660254f*be);
    pwm(u,v,w);
}
void FOC_SetTorque(float iq_norm){iq_ref=isfinite(iq_norm)?clamp(iq_norm,-1,1):0.0f;}

/* Initialization runs once in main; interrupts continue during these waits. */
static uint32_t align_wait(uint32_t ms)
{
    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < ms)
    {
        if (foc_state == FOC_STATE_FAULT) { return 0U; }
        if (!encoder_healthy()) { stop_fault(FOC_FAULT_ENCODER); return 0U; }
        HAL_Delay(1U);
    }
    return 1U;
}

void FOC_Init(void)
{
    if (!driver_init()) { return; }
    pwm(0.5f,0.5f,0.5f);
    /* Load all CCR preloads and RCR=19 before starting from CNT=0.
       Update/TRGO then occurs at underflow, once per 10 PWM periods. */
    htim1.Instance->EGR = TIM_EGR_UG;
    foc_state = FOC_STATE_CALIBRATING;
    if (HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED) != HAL_OK ||
        HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_Base_Start_IT(&htim5) != HAL_OK)
    {
        stop_fault(FOC_FAULT_STARTUP);
        return;
    }
    HAL_GPIO_WritePin(MT6816_CS_GPIO_Port,MT6816_CS_Pin,GPIO_PIN_SET);
    enc_read_low = 0U;
    encoder_start_frame(0x83U);
    /* Gather real CSA offsets with INL held low, not an assumed mid-scale. */
    uint32_t start = HAL_GetTick();
    while (offset_samples < 128U || !encoder_healthy())
    {
        if (HAL_GetTick() - start >= 500U) { stop_fault(FOC_FAULT_STARTUP); return; }
        HAL_Delay(1U);
    }
    align_theta = 0.0f;
    foc_state = FOC_STATE_ALIGNING;
    if (!align_wait(500U)) { return; }
    float initial_angle = angle;
    /* Half an electrical turn is 22.5 mechanical degrees for this motor. */
    for (uint32_t step = 1U; step <= 1000U; ++step)
    {
        align_theta = FOC_PI * (float)step / 1000.0f;
        if (!align_wait(1U)) { return; }
    }
    if (!align_wait(500U)) { return; }
    float delta = wrap(angle - initial_angle);
    float expected = FOC_PI / FOC_MOTOR_POLE_PAIRS;
    if (fabsf(delta) < 0.7f*expected || fabsf(delta) > 1.3f*expected)
    {
        stop_fault(FOC_FAULT_ALIGNMENT);
        return;
    }
    encoder_direction = delta > 0.0f ? 1.0f : -1.0f;
    electrical_offset = wrap(encoder_direction * angle * FOC_MOTOR_POLE_PAIRS - FOC_PI);
    /* Drop INL while changing frames/resetting PI, so an ISR cannot re-enable
       the bridge with partially published calibration data. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    HAL_GPIO_WritePin(DRV_cotr_GPIO_Port, DRV_cotr_Pin, GPIO_PIN_RESET);
    pi_q.integ = 0.0f;
    pi_d.integ = 0.0f;
    if (foc_state != FOC_STATE_FAULT) { foc_state = FOC_STATE_RUNNING; }
    __set_PRIMASK(primask);
}
float FOC_GetAngle(void){return angle_multi;}
float FOC_GetSpeed(void){return encoder_direction*speed;}
float FOC_GetIq(void){return iq;}
float FOC_GetId(void){return id;}
uint16_t FOC_GetEncoderRawAngle(void){return enc_raw;}
uint32_t FOC_GetEncoderStatus(void){return enc_status;}
uint32_t FOC_GetState(void){return foc_state;}
uint32_t FOC_GetFault(void){return foc_fault;}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *h)
{
    if(h->Instance!=SPI1) return;
    /* HAL has waited for SPI BSY to clear before this callback. */
    encoder_cs_delay();
    HAL_GPIO_WritePin(MT6816_CS_GPIO_Port,MT6816_CS_Pin,GPIO_PIN_SET);
    if (!enc_read_low)
    {
        enc_high = enc_rx[1]; /* First RX byte is not register data. */
        enc_read_low = 1U;
        encoder_start_frame(0x84U);
    }
    else
    {
        encoder_update(enc_high, enc_rx[1]);
        enc_read_low = 0U;
        encoder_start_frame(0x83U);
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *h)
{
    if (h->Instance != SPI1) { return; }
    encoder_cs_delay();
    HAL_GPIO_WritePin(MT6816_CS_GPIO_Port, MT6816_CS_Pin, GPIO_PIN_SET);
    enc_status |= FOC_ENCODER_SPI_ERROR;
    /* Stop the chain on a transport error; retain the last valid sample. */
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *h)
{
    if(h->Instance==TIM5)
        {
            static float last;
            static uint32_t have_last;
            float current = angle;
            if (!encoder_healthy()) { have_last=0U; speed=0.0f; return; }
            if (have_last) { speed=0.15f*wrap(current-last)/0.001f+0.85f*speed; }
            last=current;
            have_last=1U;
        }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *h)
{
    if(h->Instance!=ADC1)  return;

    if (foc_state == FOC_STATE_CALIBRATING)
    {
        if (offset_samples < 128U)
        {
            offset_u += (float)h->Instance->JDR1;
            offset_v += (float)h->Instance->JDR2;
            if (offset_samples == 127U) { offset_u /= 128.0f; offset_v /= 128.0f; }
            ++offset_samples;
        }
        return;
    }
    if (foc_state != FOC_STATE_ALIGNING && foc_state != FOC_STATE_RUNNING) { return; }
    if (!encoder_healthy()) { stop_fault(FOC_FAULT_ENCODER); return; }

    float iu=VREF*((float)h->Instance->JDR1-offset_u)/(ADC_FS*SHUNT_OHM*AMP_GAIN);
    float iv=VREF*((float)h->Instance->JDR2-offset_v)/(ADC_FS*SHUNT_OHM*AMP_GAIN);
    float al=iu, be=(iu+2*iv)*0.577350269f;

    uint32_t aligning = foc_state == FOC_STATE_ALIGNING;
    float theta = aligning ? align_theta :
        wrap(encoder_direction*angle*FOC_MOTOR_POLE_PAIRS-electrical_offset);
    float c=cosf(theta), s=sinf(theta);
    id=al*c+be*s;
    iq=-al*s+be*c;

    float e=(aligning ? 0.0f : iq_ref*MAX_CURRENT)-iq;
    float ed=(aligning ? ALIGN_CURRENT : 0.0f)-id;

    pi_q.integ=clamp(pi_q.integ+pi_q.ki*e,-0.8f,0.8f);
    pi_q.out=clamp(pi_q.kp*e+pi_q.integ,-0.8f,0.8f);
    pi_d.integ=clamp(pi_d.integ+pi_d.ki*ed,-0.8f,0.8f);
    pi_d.out=clamp(pi_d.kp*ed+pi_d.integ,-0.8f,0.8f);
    svpwm(theta,pi_d.out,pi_q.out);
    HAL_GPIO_WritePin(DRV_cotr_GPIO_Port, DRV_cotr_Pin, GPIO_PIN_SET);
}

