#ifndef FOC_CONTROL_H
#define FOC_CONTROL_H
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include <stdint.h>
#define FOC_PI 3.14159265358979323846f
#define FOC_2PI 6.2831853071795864769f
#define FOC_ENCODER_NOT_READY    (1UL << 0)
#define FOC_ENCODER_PARITY_ERROR (1UL << 1)
#define FOC_ENCODER_NO_MAG       (1UL << 2)
#define FOC_ENCODER_SPI_ERROR    (1UL << 3)
#define FOC_MOTOR_POLE_PAIRS     8.0f
#define FOC_STATE_IDLE          0U
#define FOC_STATE_CALIBRATING   1U
#define FOC_STATE_ALIGNING      2U
#define FOC_STATE_RUNNING       3U
#define FOC_STATE_FAULT         4U
#define FOC_FAULT_DRIVER        (1UL << 0)
#define FOC_FAULT_STARTUP       (1UL << 1)
#define FOC_FAULT_ENCODER       (1UL << 2)
#define FOC_FAULT_ALIGNMENT     (1UL << 3)
#define LED_5V      1
#define LED_3V3     2

void FOC_Init(void);
void FOC_SetTorque(float iq_norm);
void Blink_LED(int led);
float FOC_GetAngle(void);
float FOC_GetSpeed(void);
float FOC_GetIq(void);
float FOC_GetId(void);
/* Last valid single-turn mechanical angle: 0..16383. */
uint16_t FOC_GetEncoderRawAngle(void);
/* Zero means the latest completed angle pair is valid. SPI error stops DMA. */
uint32_t FOC_GetEncoderStatus(void);
uint32_t FOC_GetState(void);
uint32_t FOC_GetFault(void);
#endif

