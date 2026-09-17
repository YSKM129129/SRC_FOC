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
#define FOC_FAULT_OVERCURRENT   (1UL << 4)
#define FOC_RECORD_LENGTH       256U

/* One current-loop snapshot. Integer fields use the same units as the
   public telemetry: current is mA and voltage commands are 1/1000 of the
   normalized DC-bus voltage. The buffer is intentionally debugger-friendly
   and is written from the control ISR without printf or blocking I/O. */
typedef struct
{
    uint32_t timestamp_us;
    uint16_t adc_u;
    uint16_t adc_v;
    int16_t id_ma;
    int16_t iq_ma;
    int16_t iq_ref_ma;
    int16_t vd_milli;
    int16_t vq_milli;
    uint16_t ccr1;
    uint16_t ccr2;
    uint16_t ccr3;
    uint16_t angle_age_us;
    uint16_t flags;
} FOC_RecordSample;

#define FOC_RECORD_FLAG_ALIGNING       (1U << 0)
#define FOC_RECORD_FLAG_D_SATURATED    (1U << 1)
#define FOC_RECORD_FLAG_Q_SATURATED    (1U << 2)
#define FOC_RECORD_FLAG_VECTOR_LIMIT   (1U << 3)
#define FOC_RECORD_FLAG_ENCODER_STALE  (1U << 4)
#define FOC_RECORD_FLAG_BLANKING       (1U << 5)
#define FOC_RECORD_FLAG_FAULT          (1U << 6)
#define LED_5V      1
#define LED_3V3     2

typedef enum
{
    FOC_MODE_TORQUE = 0U,
    FOC_MODE_SPEED = 1U
} FOC_ControlMode;

/* Unified command payload:
   - FOC_MODE_TORQUE: signed value in 1 mN*m/count.
   - FOC_MODE_SPEED:  signed value in 1 rpm/count. */
typedef struct
{
    FOC_ControlMode mode;
    int16_t value;
} FOC_CommandFrame;

void FOC_Init(void);
/* Apply a mode-dependent command payload. Returns 1 when MODE is valid. */
uint32_t FOC_ApplyCommandFrame(const FOC_CommandFrame *command);
void FOC_SetTorqueMilliNewtonMeter(float torque_mnm);
void FOC_SetSpeedRPM(float speed_rpm);
void FOC_PollDriverFault(void);
void Blink_LED(int led);
float FOC_GetAngle(void);
float FOC_GetSpeed(void);
float FOC_GetIq(void);
float FOC_GetId(void);
float FOC_GetBusVoltage(void);
FOC_ControlMode FOC_GetControlMode(void);
/* Last valid single-turn mechanical angle: 0..16383. */
uint16_t FOC_GetEncoderRawAngle(void);
/* Zero means the latest completed angle pair is valid. Transient errors are
   retried; control stops only when no valid angle arrives for 20 ms. */
uint32_t FOC_GetEncoderStatus(void);
uint32_t FOC_GetState(void);
uint32_t FOC_GetFault(void);
float FOC_GetIqReference(void);
uint32_t FOC_GetAngleAgeUs(void);
uint16_t FOC_GetControlFlags(void);

/* Continuous last-samples recorder. Stop/fault freezes the buffer so it can
   be inspected with the debugger or sent later over a low-rate link. */
extern volatile FOC_RecordSample foc_record_buffer[FOC_RECORD_LENGTH];
extern volatile uint16_t foc_record_write_index;
extern volatile uint16_t foc_record_count;
extern volatile uint8_t foc_record_frozen;
void FOC_RecordClear(void);
void FOC_RecordFreeze(void);
#endif

