#include "foc_control.h"
#include "gpio.h"
#include "DRV8353.h"
#include <math.h>

#define ADC_FS 4095.0f
#define SHUNT_OHM 0.001f
/* Uses DRV8353 SOA/SOB only. U15/U16 OUT pins must be disconnected because
   the schematic ties those INA240 outputs to the same nets. */
#define AMP_GAIN 40.0f
#define VREF 3.3f
#define PWM_PERIOD 2125U
#define MAX_MOD 0.90f
#define MAX_PHASE_CURRENT 3.0f
#define CURRENT_OUTPUT_LIMIT 0.05f
#define ALIGN_VOLTAGE 0.04f
#define FOC_CALIBRATION_ADDRESS 0x0801F000UL
#define FOC_CALIBRATION_MAGIC 0x464F4342UL
#define FOC_CALIBRATION_CHECK 0xA53C91E7UL
/* Set to 1 for one firmware run to replace the stored encoder calibration. */
#define FOC_FORCE_ENCODER_CALIBRATION 0U

typedef struct { float kp, ki, integ, out; } pi_t;
static volatile float angle, angle_multi, speed, iq, id, iq_ref_amp;
/* MT6816: two separate 16-clock frames, each sent as two 8-bit bytes. */
static uint8_t enc_rx[2];
static uint8_t enc_tx[2] = {0x83U, 0x00U};
static uint8_t enc_high, enc_read_low, enc_initialized;
static volatile uint8_t enc_sampling_enabled, enc_busy;
static volatile uint16_t enc_raw;
static volatile uint32_t enc_status = FOC_ENCODER_NOT_READY;
static volatile uint32_t enc_last_valid_ms;
static volatile uint32_t foc_state = FOC_STATE_IDLE, foc_fault;
static volatile uint32_t overcurrent_detail;
static volatile uint32_t offset_samples;
static float offset_u, offset_v;
static volatile float align_theta;
static float encoder_direction = 1.0f, electrical_offset;
static float enc_prev, enc_turns;
/* 339285: Rll=0.464 ohm, Lll=0.322 mH. Conservative current-loop
   tuning for a 20 kHz update rate and approximately 16 V DC bus. */
static pi_t pi_q = {1.2f, 0.06f, 0, 0};
static pi_t pi_d = {1.2f, 0.06f, 0, 0};

static void led_set(int led, uint32_t on)
{
    GPIO_TypeDef *port = led == LED_5V ? LED1_GPIO_Port : LED2_GPIO_Port;
    uint16_t pin = led == LED_5V ? LED1_Pin : LED2_Pin;
    /* Both LEDs are fed from their rail through a resistor; the MCU sinks
       current, so GPIO low turns the LED on. */
    HAL_GPIO_WritePin(port, pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Blink_LED(int led)
{
    if (led == LED_5V)
    {
        for(int i=0; i<5; i++)
        {
            led_set(LED_5V, 1U);
            HAL_Delay(200);
            led_set(LED_5V, 0U);
            HAL_Delay(200);
        }
    }
    else if (led == LED_3V3)
    {
        for(int i=0; i<5; i++)
        {
            led_set(LED_3V3, 1U);
            HAL_Delay(200);
            led_set(LED_3V3, 0U);
            HAL_Delay(200);
        }    
    }
}

static void Blink_DRV_Error(void)
{
    uint32_t reason = drv835x_debug_error;
    uint32_t stage = drv835x_debug_stage;
    if (reason < 1U || reason > 2U) { reason = 3U; }
    if (stage < 1U || stage > 13U) { stage = 14U; }

    /* Repeating code on the 3V3 LED:
       long flashes = error reason, then short flashes = failing stage. */
    while (1)
    {
        for (uint32_t i = 0U; i < reason; ++i)
        {
            led_set(LED_3V3, 1U);
            HAL_Delay(500U);
            led_set(LED_3V3, 0U);
            HAL_Delay(300U);
        }
        HAL_Delay(700U);
        for (uint32_t i = 0U; i < stage; ++i)
        {
            led_set(LED_3V3, 1U);
            HAL_Delay(120U);
            led_set(LED_3V3, 0U);
            HAL_Delay(180U);
        }
        HAL_Delay(1500U);
    }
}

static uint32_t Blink_GDF_Detail(void)
{
    uint32_t detail = 0U;
    DRV835X_read_FaultStatusReg1();
    DRV835X_read_FaultStatusReg2();

    if (stru_DRV8353Obj.faultStatusReg1_obj.data & (1U << 8))
    {
        uint16_t vgs = stru_DRV8353Obj.faultStatusReg2_obj.data & 0x003fU;
        if (vgs != 0U && (vgs & (vgs - 1U)) == 0U)
        {
            while ((vgs & 1U) == 0U) { ++detail; vgs >>= 1; }
            ++detail;
        }
        else if (vgs != 0U) { detail = 7U; }
        else { detail = 8U; }
    }

    for (uint32_t i = 0U; i < detail; ++i)
    {
        led_set(LED_3V3, 1U);
        HAL_Delay(180U);
        led_set(LED_3V3, 0U);
        HAL_Delay(220U);
    }
    return detail;
}

static void Blink_Alignment_Detail(float delta, float expected)
{
    uint32_t detail;
    float travel = fabsf(delta);

    /* Long 3V3 flashes after the five 5V flashes:
       1 = essentially no net movement, 2 = movement too small,
       3 = movement too large. A GDF code takes priority if present. */
    if (travel < 0.10f * expected) { detail = 1U; }
    else if (travel < 0.70f * expected) { detail = 2U; }
    else { detail = 3U; }

    for (uint32_t i = 0U; i < detail; ++i)
    {
        led_set(LED_3V3, 1U);
        HAL_Delay(500U);
        led_set(LED_3V3, 0U);
        HAL_Delay(350U);
    }
}

static void stop_fault(uint32_t fault)
{
    foc_fault |= fault;
    foc_state = FOC_STATE_FAULT;
    enc_sampling_enabled = 0U;
    HAL_GPIO_WritePin(DRV_cotr_GPIO_Port, DRV_cotr_Pin, GPIO_PIN_RESET);
}

static uint32_t encoder_healthy(void)
{
    /* Discard individual corrupt frames, but never use a stale angle. */
    return !(enc_status & (FOC_ENCODER_NOT_READY | FOC_ENCODER_SPI_ERROR)) &&
           (HAL_GetTick() - enc_last_valid_ms < 10U);
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
    enc_busy = 1U;
    encoder_cs_delay();
    HAL_GPIO_WritePin(MT6816_CS_GPIO_Port, MT6816_CS_Pin, GPIO_PIN_RESET);
    encoder_cs_delay();
    if (HAL_SPI_TransmitReceive_DMA(&hspi1, enc_tx, enc_rx, 2U) != HAL_OK)
    {
        enc_busy = 0U;
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

static uint32_t calibration_checksum(uint32_t direction, uint32_t offset)
{
    return FOC_CALIBRATION_MAGIC ^ direction ^ offset ^ FOC_CALIBRATION_CHECK;
}

static uint32_t calibration_load(void)
{
    const volatile uint32_t *words=(const volatile uint32_t *)FOC_CALIBRATION_ADDRESS;
    union { uint32_t bits; float value; } offset;
    uint32_t direction=words[1];
    offset.bits=words[2];

    if (words[0] != FOC_CALIBRATION_MAGIC ||
        words[3] != calibration_checksum(direction,offset.bits) ||
        (direction != 1U && direction != 0xffffffffU) ||
        !isfinite(offset.value) || fabsf(offset.value) > FOC_PI)
    {
        return 0U;
    }

    encoder_direction=direction == 1U ? 1.0f : -1.0f;
    electrical_offset=offset.value;
    return 1U;
}

static uint32_t calibration_save(void)
{
    union { uint32_t bits; float value; } offset;
    FLASH_EraseInitTypeDef erase={0};
    uint32_t page_error=0xffffffffU;
    uint32_t direction=encoder_direction > 0.0f ? 1U : 0xffffffffU;
    uint32_t check;
    uint64_t first,second;
    HAL_StatusTypeDef status;

    offset.value=electrical_offset;
    check=calibration_checksum(direction,offset.bits);
    first=(uint64_t)FOC_CALIBRATION_MAGIC | ((uint64_t)direction << 32);
    second=(uint64_t)offset.bits | ((uint64_t)check << 32);

    erase.TypeErase=FLASH_TYPEERASE_PAGES;
    erase.NbPages=1U;
#if defined(FLASH_OPTR_DBANK)
    if ((FLASH->OPTR & FLASH_OPTR_DBANK) != 0U)
    {
        erase.Banks=FLASH_BANK_2;
        erase.Page=(FOC_CALIBRATION_ADDRESS-FLASH_BASE-FLASH_BANK_SIZE)/FLASH_PAGE_SIZE;
    }
    else
    {
        erase.Banks=FLASH_BANK_1;
        erase.Page=(FOC_CALIBRATION_ADDRESS-FLASH_BASE)/FLASH_PAGE_SIZE_128_BITS;
    }
#else
    erase.Banks=FLASH_BANK_1;
    erase.Page=(FOC_CALIBRATION_ADDRESS-FLASH_BASE)/FLASH_PAGE_SIZE;
#endif

    if (HAL_FLASH_Unlock() != HAL_OK) { return 0U; }
    status=HAL_FLASHEx_Erase(&erase,&page_error);
    if (status == HAL_OK)
    {
        status=HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                 FOC_CALIBRATION_ADDRESS,first);
    }
    if (status == HAL_OK)
    {
        status=HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                 FOC_CALIBRATION_ADDRESS+8U,second);
    }
    HAL_FLASH_Lock();
    return status == HAL_OK && calibration_load();
}

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
    float u_phase=al,
          v_phase=-0.5f*al+0.8660254f*be,
          w_phase=-0.5f*al-0.8660254f*be;

    /* Continuous, symmetric SVPWM. Injecting the same zero-sequence voltage
       into all three phases is equivalent to centering T0 in each PWM cycle. */
    float phase_max=fmaxf(u_phase,fmaxf(v_phase,w_phase));
    float phase_min=fminf(u_phase,fminf(v_phase,w_phase));
    float zero_sequence=-0.5f*(phase_max+phase_min);

    float u=0.5f+0.5f*(u_phase+zero_sequence),
          v=0.5f+0.5f*(v_phase+zero_sequence),
          w=0.5f+0.5f*(w_phase+zero_sequence);
    pwm(u,v,w);
}
void FOC_SetTorque(float iq_amp)
{
    iq_ref_amp=isfinite(iq_amp)?clamp(iq_amp,-MAX_PHASE_CURRENT,MAX_PHASE_CURRENT):0.0f;
}

void FOC_PollDriverFault(void)
{
    static uint32_t fault_reported;
    uint32_t detail = 0U;
    if (fault_reported) { return; }

    if (foc_state == FOC_STATE_FAULT)
    {
        fault_reported = 1U;
        /* Four quick flashes identify a controller-state fault, followed by:
           1=startup, 2=encoder, 3=alignment,
           4=IU positive, 5=IU negative, 6=IV positive, 7=IV negative,
           8=IW positive, 9=IW negative overcurrent, 10=unknown overcurrent,
           11=driver initialization/other driver fault. */
        if (foc_fault & FOC_FAULT_STARTUP) { detail = 1U; }
        else if (foc_fault & FOC_FAULT_ENCODER) { detail = 2U; }
        else if (foc_fault & FOC_FAULT_ALIGNMENT) { detail = 3U; }
        else if (foc_fault & FOC_FAULT_OVERCURRENT)
        {
            detail = overcurrent_detail ? overcurrent_detail : 10U;
        }
        else { detail = 11U; }

        while (1)
        {
            led_set(LED_3V3, 0U);
            HAL_Delay(1200U);
            for (uint32_t preamble = 0U; preamble < 4U; ++preamble)
            {
                led_set(LED_3V3, 1U);
                HAL_Delay(100U);
                led_set(LED_3V3, 0U);
                HAL_Delay(150U);
            }
            HAL_Delay(700U);
            for (uint32_t i = 0U; i < detail; ++i)
            {
                led_set(LED_3V3, 1U);
                HAL_Delay(400U);
                led_set(LED_3V3, 0U);
                HAL_Delay(300U);
            }
            HAL_Delay(1800U);
        }
    }

    if (foc_state != FOC_STATE_RUNNING) { return; }

    DRV835X_read_FaultStatusReg1();
    DRV835X_read_FaultStatusReg2();
    if (stru_DRV8353Obj.faultStatusReg1_obj.data & (1U << 10))
    {
        fault_reported = 1U;
        stop_fault(FOC_FAULT_DRIVER);
        /* Solid 5V LED means a runtime DRV8353 fault latched the bridge off. */
        led_set(LED_5V, 1U);

        /* For VDS overcurrent, slow 3V3 flashes identify the MOSFET:
           1=LC, 2=HC, 3=LB, 4=HB, 5=LA, 6=HA, 7=multiple/aggregate.
           Other fault classes use 8=GDF, 9=UVLO, 10=OTSD,
           11=shunt-sense OCP, 12=gate-drive UV, 13=other. */
        if (stru_DRV8353Obj.faultStatusReg1_obj.data & (1U << 9))
        {
            uint16_t vds = stru_DRV8353Obj.faultStatusReg1_obj.data & 0x003fU;
            if (vds != 0U && (vds & (vds - 1U)) == 0U)
            {
                while ((vds & 1U) == 0U) { ++detail; vds >>= 1; }
                ++detail;
            }
            else { detail = 7U; }
        }
        else if (stru_DRV8353Obj.faultStatusReg1_obj.data & (1U << 8)) { detail = 8U; }
        else if (stru_DRV8353Obj.faultStatusReg1_obj.data & (1U << 7)) { detail = 9U; }
        else if (stru_DRV8353Obj.faultStatusReg1_obj.data & (1U << 6)) { detail = 10U; }
        else if (stru_DRV8353Obj.faultStatusReg2_obj.data & 0x0700U) { detail = 11U; }
        else if (stru_DRV8353Obj.faultStatusReg2_obj.data & (1U << 6)) { detail = 12U; }
        else { detail = 13U; }

        /* Repeat forever so a power-on LED transient cannot be mistaken for
           the diagnostic. Three quick flashes mark the start of every code. */
        while (1)
        {
            led_set(LED_3V3, 0U);
            HAL_Delay(1200U);
            for (uint32_t preamble = 0U; preamble < 3U; ++preamble)
            {
                led_set(LED_3V3, 1U);
                HAL_Delay(100U);
                led_set(LED_3V3, 0U);
                HAL_Delay(150U);
            }
            HAL_Delay(700U);
            for (uint32_t i = 0U; i < detail; ++i)
            {
                led_set(LED_3V3, 1U);
                HAL_Delay(400U);
                led_set(LED_3V3, 0U);
                HAL_Delay(300U);
            }
            HAL_Delay(1800U);
        }
    }
}

/* Initialization runs once in main; interrupts continue during these waits. */
static uint32_t align_wait(uint32_t ms)
{
    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < ms)
    {
        if (foc_state == FOC_STATE_FAULT) { return 0U; }
        if (!encoder_healthy())
        {
            stop_fault(FOC_FAULT_ENCODER);
            return 0U;
        }
        HAL_Delay(1U);
    }
    return 1U;
}

void FOC_Init(void)
{
    if (DRV835X_Init() != HAL_OK)
    { 
        stop_fault(FOC_FAULT_DRIVER);
        Blink_DRV_Error();
    }
    pwm(0.5f,0.5f,0.5f);
    /* Load all CCR preloads and RCR=3 before starting from CNT=0.
       With 40 kHz center-aligned PWM, update/TRGO runs at 20 kHz. */
    htim1.Instance->EGR = TIM_EGR_UG;
    foc_state = FOC_STATE_CALIBRATING;
    if (HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED) != HAL_OK ||
        HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4) != HAL_OK ||
        HAL_TIM_Base_Start_IT(&htim5) != HAL_OK)
    {
        stop_fault(FOC_FAULT_STARTUP);
        return;
    }
    HAL_GPIO_WritePin(MT6816_CS_GPIO_Port,MT6816_CS_Pin,GPIO_PIN_SET);
    enc_read_low = 0U;
    enc_sampling_enabled = 1U;
    encoder_start_frame(0x83U);
    /* Gather real CSA offsets with INL held low, not an assumed mid-scale. */
    uint32_t start = HAL_GetTick();
    while (offset_samples < 128U || !encoder_healthy())
    {
        if (HAL_GetTick() - start >= 500U)
        {
            stop_fault(FOC_FAULT_STARTUP);
            return;
        }
        HAL_Delay(1U);
    }
    if (FOC_FORCE_ENCODER_CALIBRATION || !calibration_load())
    {
        align_theta = 0.0f;
        foc_state = FOC_STATE_ALIGNING;
        if (!align_wait(800U)) { return; }
        float initial_angle = angle;
        /* Determine direction once by sweeping an open-loop stator field over
           half an electrical turn. The alignment path deliberately bypasses
           the current PI and its as-yet-uncalibrated Park angle. */
        for (uint32_t step = 1U; step <= 1500U; ++step)
        {
            align_theta = FOC_PI * (float)step / 1500.0f;
            if (!align_wait(1U)) { return; }
        }
        if (!align_wait(500U)) { return; }
        float delta = wrap(angle - initial_angle);
        float expected = FOC_PI / FOC_MOTOR_POLE_PAIRS;
        /* Reject a sweep too small to establish encoder direction reliably. */
        if (fabsf(delta) < 0.25f*expected)
        {
            stop_fault(FOC_FAULT_ALIGNMENT);
            Blink_LED(LED_5V);
            HAL_Delay(700U);
            if (Blink_GDF_Detail() == 0U)
            {
                HAL_Delay(700U);
                Blink_Alignment_Detail(delta, expected);
            }
            return;
        }
        encoder_direction = delta > 0.0f ? 1.0f : -1.0f;
        /* The rotor is held at the final commanded electrical angle before
           this sample, so capture the offset at the same operating point. */
        electrical_offset = wrap(encoder_direction * angle * FOC_MOTOR_POLE_PAIRS - FOC_PI);

        /* Stop the bridge before erasing/programming the calibration page. */
        HAL_GPIO_WritePin(DRV_cotr_GPIO_Port, DRV_cotr_Pin, GPIO_PIN_RESET);
        foc_state = FOC_STATE_CALIBRATING;
        (void)calibration_save();
    }
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
    if (!enc_sampling_enabled)
    {
        enc_busy = 0U;
        return;
    }
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
        enc_busy = 0U;
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *h)
{
    if (h->Instance != SPI1) { return; }
    encoder_cs_delay();
    HAL_GPIO_WritePin(MT6816_CS_GPIO_Port, MT6816_CS_Pin, GPIO_PIN_SET);
    enc_busy = 0U;
    enc_sampling_enabled = 0U;
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
    static uint32_t encoder_divider;
    static uint32_t control_divider;
    static uint32_t overcurrent_count;
    if(h->Instance!=ADC1)  return;

    /* CH4 triggers ADC once per 40 kHz PWM period. Keep MT6816 at 4 kHz. */
    if (++encoder_divider >= 10U)
    {
        encoder_divider = 0U;
        if (enc_sampling_enabled && !enc_busy && !enc_read_low)
        {
            encoder_start_frame(0x83U);
        }
    }

    // calibrate ADC
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

    /* Run Clarke/Park and the PI controllers at 20 kHz. */
    if (++control_divider < 2U) { return; }
    control_divider = 0U;

    if (foc_state != FOC_STATE_ALIGNING && foc_state != FOC_STATE_RUNNING) return;

    if (!encoder_healthy()) 
    { 
        stop_fault(FOC_FAULT_ENCODER); 
        return; 
    }

    /* With U15/U16 removed, PA0/PA1 are driven only by the DRV8353 internal
       CSAs. The measured response under sufficient voltage shows that the
       direct SPA-SNA polarity closes the current loop as negative feedback. */

    float iu=VREF*((float)h->Instance->JDR1-offset_u)/(ADC_FS*SHUNT_OHM*AMP_GAIN);
    float iv=VREF*((float)h->Instance->JDR2-offset_v)/(ADC_FS*SHUNT_OHM*AMP_GAIN);

    float iw=-iu-iv;
    if (fabsf(iu) > MAX_PHASE_CURRENT || fabsf(iv) > MAX_PHASE_CURRENT || fabsf(iw) > MAX_PHASE_CURRENT)
    {
        if (iu > MAX_PHASE_CURRENT) { overcurrent_detail = 4U; }
        else if (iu < -MAX_PHASE_CURRENT) { overcurrent_detail = 5U; }
        else if (iv > MAX_PHASE_CURRENT) { overcurrent_detail = 6U; }
        else if (iv < -MAX_PHASE_CURRENT) { overcurrent_detail = 7U; }
        else if (iw > MAX_PHASE_CURRENT) { overcurrent_detail = 8U; }
        else { overcurrent_detail = 9U; }
        /* Reject an isolated PWM-edge sample; three consecutive samples are
           only 150 us at the 20 kHz loop rate. Hardware OCP remains immediate. */
        if (++overcurrent_count >= 3U)
        {
            stop_fault(FOC_FAULT_OVERCURRENT);
            led_set(LED_3V3, 1U);
            return;
        }
    }
    else { overcurrent_count = 0U; }

    /* Amplitude-invariant Clarke transform for two-shunt sampling.
       With iw=-iu-iv, this is the reduced form of the full 2/3 transform. */
    float i_alpha=iu;
    float i_beta=0.5773502692f*(iu+2.0f*iv);

    uint32_t aligning = foc_state == FOC_STATE_ALIGNING;
    float theta = aligning ? align_theta :
        wrap(encoder_direction*angle*FOC_MOTOR_POLE_PAIRS-electrical_offset);
    float c=cosf(theta), s=sinf(theta);

    /* Park transform. This sign convention is the inverse of svpwm() above. */
    id=i_alpha*c+i_beta*s;
    iq=-i_alpha*s+i_beta*c;

    if (aligning)
    {
        svpwm(theta,ALIGN_VOLTAGE,0.0f);
        HAL_GPIO_WritePin(DRV_cotr_GPIO_Port, DRV_cotr_Pin, GPIO_PIN_SET);
        return;
    }

    float eq=iq_ref_amp-iq;
    float ed=-id;

    // PI control
    pi_q.integ=clamp(pi_q.integ+pi_q.ki*eq,-CURRENT_OUTPUT_LIMIT,CURRENT_OUTPUT_LIMIT);
    pi_q.out=clamp(pi_q.kp*eq+pi_q.integ,-CURRENT_OUTPUT_LIMIT,CURRENT_OUTPUT_LIMIT);

    pi_d.integ=clamp(pi_d.integ+pi_d.ki*ed,-CURRENT_OUTPUT_LIMIT,CURRENT_OUTPUT_LIMIT);
    pi_d.out=clamp(pi_d.kp*ed+pi_d.integ,-CURRENT_OUTPUT_LIMIT,CURRENT_OUTPUT_LIMIT);

    svpwm(theta,pi_d.out,pi_q.out);
    HAL_GPIO_WritePin(DRV_cotr_GPIO_Port, DRV_cotr_Pin, GPIO_PIN_SET);
}
