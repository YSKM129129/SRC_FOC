#ifndef FOC_CONTROL_H
#define FOC_CONTROL_H
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include <stdint.h>
#define FOC_PI 3.14159265358979323846f
#define FOC_2PI 6.2831853071795864769f
void FOC_Init(void);
void FOC_SetTorque(float iq_norm);
float FOC_GetAngle(void);
float FOC_GetSpeed(void);
float FOC_GetIq(void);
float FOC_GetId(void);
#endif

