#ifndef FOC_SPI2_LINK_H
#define FOC_SPI2_LINK_H

#include "main.h"

#define FOC_SPI2_FRAME_WORDS 8U
#define FOC_SPI2_MAGIC        0xA55AU

/* RX command frame: word 0 magic, word 1 MODE (0=torque, 1=speed),
   word 2 signed VALUE. Torque uses 1 mN*m/count; speed uses 1 rpm/count.
   Word 7 is the XOR checksum of words 0..6. */

/* Exchange one telemetry frame and one command response with the master MCU. */
HAL_StatusTypeDef FOC_Spi2_Exchange(void);

/* Number of valid command frames received from the main MCU. */
uint32_t FOC_Spi2_GetRxCount(void);

#endif /* FOC_SPI2_LINK_H */
