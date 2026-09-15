#ifndef FOC_SPI2_LINK_H
#define FOC_SPI2_LINK_H

#include "main.h"

#define FOC_SPI2_FRAME_WORDS 8U
#define FOC_SPI2_MAGIC        0xA55AU

/* Exchange one telemetry frame and one command response with the master MCU. */
HAL_StatusTypeDef FOC_Spi2_Exchange(void);

/* Number of valid command frames received from the main MCU. */
uint32_t FOC_Spi2_GetRxCount(void);

#endif /* FOC_SPI2_LINK_H */
