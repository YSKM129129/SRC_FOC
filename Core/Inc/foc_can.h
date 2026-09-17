#ifndef FOC_CAN_H
#define FOC_CAN_H

#include "main.h"

/* Standard 11-bit identifiers used by the FOC node protocol. */
#define FOC_CAN_TELEMETRY_BASE 0x100U
#define FOC_CAN_COMMAND_BASE   0x200U
#define FOC_CAN_STATUS_BASE    0x300U

/* Four-byte command payload:
   byte 0: MODE (0=torque, 1=speed), byte 1: reserved,
   bytes 2..3: signed little-endian VALUE.
   Torque VALUE uses 1 mN*m/count; speed VALUE uses 1 rpm/count. */

/* Change this value when several FOC boards share one CAN bus. */
#ifndef FOC_CAN_NODE_ID
#define FOC_CAN_NODE_ID        1U
#endif

HAL_StatusTypeDef FOC_Can_Init(void);
HAL_StatusTypeDef FOC_Can_SendTelemetry(void);

#endif /* FOC_CAN_H */
