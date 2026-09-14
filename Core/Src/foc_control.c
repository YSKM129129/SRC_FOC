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

typedef struct { float kp, ki, integ, out; } pi_t;
static volatile float angle, angle_multi, speed, iq, id, iq_ref;
static uint16_t enc_rx;
static uint16_t enc_tx = 0xffffU;
static float enc_prev, enc_turns;
static pi_t pi_q = {0.08f, 0.0008f, 0, 0};

static float clamp(float x,float lo,float hi){return x<lo?lo:(x>hi?hi:x);}
static float wrap(float x){ while(x>FOC_PI)x-=FOC_2PI; while(x<-FOC_PI)x+=FOC_2PI; return x; }
static void pwm(float a,float b,float c){
 a=clamp(a,0,MAX_MOD); b=clamp(b,0,MAX_MOD); c=clamp(c,0,MAX_MOD);
 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,(uint32_t)(a*PWM_PERIOD));
 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,(uint32_t)(b*PWM_PERIOD));
 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,(uint32_t)(c*PWM_PERIOD));
}
static void svpwm(float theta,float vd,float vq){
 float al=vd*cosf(theta)-vq*sinf(theta), be=vd*sinf(theta)+vq*cosf(theta);
 float u=0.5f+0.5f*al, v=0.5f+0.5f*(-0.5f*al+0.8660254f*be), w=0.5f+0.5f*(-0.5f*al-0.8660254f*be);
 pwm(u,v,w);
}
void FOC_SetTorque(float iq_norm){iq_ref=clamp(iq_norm,-1,1);}
void FOC_Init(void){
 HAL_GPIO_WritePin(DRV_ENBLE_GPIO_Port,DRV_ENBLE_Pin,GPIO_PIN_RESET);
 HAL_GPIO_WritePin(DRV_cotr_GPIO_Port,DRV_cotr_Pin,GPIO_PIN_RESET);
 HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1); HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2); HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
 HAL_TIM_Base_Start(&htim1); HAL_TIM_Base_Start_IT(&htim5);
 HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED); HAL_ADCEx_InjectedStart_IT(&hadc1);
 HAL_GPIO_WritePin(AS5047P_CS_GPIO_Port,AS5047P_CS_Pin,GPIO_PIN_SET);
 HAL_SPI_TransmitReceive_DMA(&hspi1,(uint8_t*)&enc_tx,(uint8_t*)&enc_rx,1);
 pwm(0.5f,0.5f,0.5f);
}
float FOC_GetAngle(void){return angle_multi;} float FOC_GetSpeed(void){return speed;} float FOC_GetIq(void){return iq;} float FOC_GetId(void){return id;}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *h){ if(h->Instance!=SPI1)return; HAL_GPIO_WritePin(AS5047P_CS_GPIO_Port,AS5047P_CS_Pin,GPIO_PIN_SET); uint16_t raw=enc_rx&0x3fff; float a=FOC_2PI*(float)raw/16384.0f; float d=(float)raw-enc_prev; if(d>8192)enc_turns-=FOC_2PI; else if(d<-8192)enc_turns+=FOC_2PI; enc_prev=(float)raw; angle=a; angle_multi=enc_turns+a; HAL_GPIO_WritePin(AS5047P_CS_GPIO_Port,AS5047P_CS_Pin,GPIO_PIN_RESET); HAL_SPI_TransmitReceive_DMA(&hspi1,(uint8_t*)&enc_tx,(uint8_t*)&enc_rx,1); }
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *h){ if(h->Instance==TIM5){ static float last; speed=0.15f*wrap(angle-last)/0.001f+0.85f*speed; last=angle; } }
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *h){ if(h->Instance!=ADC1)return; float iu=(VREF*((float)h->Instance->JDR1/ADC_FS-0.5f))/(SHUNT_OHM*AMP_GAIN); float iv=(VREF*((float)h->Instance->JDR2/ADC_FS-0.5f))/(SHUNT_OHM*AMP_GAIN); float al=iu,be=(iu+2*iv)*0.577350269f; id=al*cosf(angle)+be*sinf(angle); iq=-al*sinf(angle)+be*cosf(angle); float e=iq_ref*MAX_CURRENT-iq; pi_q.integ=clamp(pi_q.integ+pi_q.ki*e,-0.8f,0.8f); pi_q.out=clamp(pi_q.kp*e+pi_q.integ,-0.8f,0.8f); svpwm(angle,0,pi_q.out); }


