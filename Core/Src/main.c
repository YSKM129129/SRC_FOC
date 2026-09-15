#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "foc_control.h"
#include "foc_can.h"
#include "foc_spi2_link.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

/* System clock configuration generated for the STM32G474. */
void SystemClock_Config(void);

int main(void)
{
    /* Initialize the HAL and configure the MCU system clock first. */
    HAL_Init();
    SystemClock_Config();

    /* Initialize GPIO, DMA, ADC, communication peripherals and timers. */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_FDCAN1_Init();
    MX_SPI1_Init();
    MX_SPI3_Init();
    MX_TIM1_Init();
    MX_TIM5_Init();
    MX_USART1_UART_Init();
    MX_ADC2_Init();
    MX_SPI2_Init();

    /* Start the CAN node before accepting commands from the master. */
    if (FOC_Can_Init() != HAL_OK)
    {
        Error_Handler();
    }

    /* Start PWM, current sampling, encoder feedback and FOC control. */
    FOC_Init();

    /* Main-loop work is handled by interrupts and low-rate telemetry links. */
    uint32_t last_telemetry_ms = HAL_GetTick();

    while (1)
    {
        if ((HAL_GetTick() - last_telemetry_ms) >= 10U)
        {
            last_telemetry_ms = HAL_GetTick();
            (void)FOC_Can_SendTelemetry();
            (void)FOC_Spi2_Exchange();
        }

        HAL_Delay(1);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc_config = {0};
    RCC_ClkInitTypeDef clk_config = {0};

    /* Use the external oscillator and configure the PLL for the CPU clock. */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    osc_config.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc_config.HSEState = RCC_HSE_ON;
    osc_config.PLL.PLLState = RCC_PLL_ON;
    osc_config.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc_config.PLL.PLLM = RCC_PLLM_DIV2;
    osc_config.PLL.PLLN = 85;
    osc_config.PLL.PLLP = RCC_PLLP_DIV2;
    osc_config.PLL.PLLQ = RCC_PLLQ_DIV2;
    osc_config.PLL.PLLR = RCC_PLLR_DIV2;

    if (HAL_RCC_OscConfig(&osc_config) != HAL_OK)
    {
        /* Stop here if the oscillator or PLL cannot be configured. */
        Error_Handler();
    }

    clk_config.ClockType = RCC_CLOCKTYPE_HCLK |
                           RCC_CLOCKTYPE_SYSCLK |
                           RCC_CLOCKTYPE_PCLK1 |
                           RCC_CLOCKTYPE_PCLK2;
    clk_config.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk_config.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk_config.APB1CLKDivider = RCC_HCLK_DIV1;
    clk_config.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&clk_config, FLASH_LATENCY_4) != HAL_OK)
    {
        /* Stop here if the bus clocks or flash wait states fail to configure. */
        Error_Handler();
    }
}

void Error_Handler(void)
{
    /* Disable interrupts and remain here for safe fault handling. */
    __disable_irq();

    while (1)
    {
    }
}
