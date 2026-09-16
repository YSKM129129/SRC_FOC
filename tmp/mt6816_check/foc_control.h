#include <stdint.h>
#define FOC_PI 3.14159265358979323846f
#define FOC_2PI 6.2831853071795864769f
#define FOC_ENCODER_NOT_READY 1U
#define FOC_ENCODER_PARITY_ERROR 2U
#define FOC_ENCODER_NO_MAG 4U
#define FOC_ENCODER_SPI_ERROR 8U
#define FOC_MOTOR_POLE_PAIRS 8.0f
#define FOC_STATE_IDLE 0U
#define FOC_STATE_CALIBRATING 1U
#define FOC_STATE_ALIGNING 2U
#define FOC_STATE_RUNNING 3U
#define FOC_STATE_FAULT 4U
#define FOC_FAULT_DRIVER 1U
#define FOC_FAULT_STARTUP 2U
#define FOC_FAULT_ENCODER 4U
#define FOC_FAULT_ALIGNMENT 8U
typedef struct { int Instance; } SPI_HandleTypeDef;
typedef struct { uint32_t EGR; } TIM_Regs;
typedef struct { TIM_Regs *Instance; } TIM_HandleTypeDef;
typedef struct { uint32_t JDR1, JDR2; } ADC_Regs;
typedef struct { ADC_Regs *Instance; } ADC_HandleTypeDef;
static SPI_HandleTypeDef hspi1 = {1};
static SPI_HandleTypeDef hspi3 = {3};
typedef int HAL_StatusTypeDef;
static uint32_t test_ms;
static uint32_t HAL_GetTick(void) { return test_ms; }
void HAL_Delay(uint32_t ms);
void Error_Handler(void);
void FOC_SetTorque(float value);
int HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t, uint32_t);
static TIM_Regs tim1_regs, tim5_regs;
static TIM_HandleTypeDef htim1 = {&tim1_regs}, htim5 = {&tim5_regs};
static ADC_Regs adc_regs;
static ADC_HandleTypeDef hadc1 = {&adc_regs};
#define SPI1 1
#define TIM5 (&tim5_regs)
#define ADC1 (&adc_regs)
#define MT6816_CS_GPIO_Port 0
#define MT6816_CS_Pin 1
#define DRV_ENBLE_GPIO_Port 0
#define DRV_ENBLE_Pin 2
#define DRV_cotr_GPIO_Port 0
#define DRV_cotr_Pin 3
#define DRV_CS_GPIO_Port 0
#define DRV_CS_Pin 4
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
#define TIM_CHANNEL_1 1
#define TIM_CHANNEL_2 2
#define TIM_CHANNEL_3 3
#define ADC_SINGLE_ENDED 0
#define HAL_OK 0
#define TIM_EGR_UG 1U
#define __NOP() ((void)0)
void test_set_compare(uint32_t channel, uint32_t compare);
#define __HAL_TIM_SET_COMPARE(a,b,c) test_set_compare((b),(c))
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __set_PRIMASK(x) ((void)(x))
#define HAL_TIM_PWM_Start(a,b) 0
#define HAL_TIM_Base_Start(a) ((void)0)
#define HAL_TIM_Base_Start_IT(a) 0
#define HAL_ADCEx_Calibration_Start(a,b) 0
#define HAL_ADCEx_InjectedStart_IT(a) 0
void HAL_GPIO_WritePin(int port, int pin, int value);
int HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t);
