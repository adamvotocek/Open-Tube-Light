/**
 * @file ssd1306_conf.h
 * @brief Project configuration for the SSD1306 library.
 *
 * Change this file when you port the display driver to a different MCU, transport, or board.
 * The current project is configured for STM32H7, I2C1, FreeRTOS, and I2C TX DMA.
 *
 * Configuration checklist:
 * - Select exactly one STM32 family macro below.
 * - Select exactly one transport macro: SSD1306_USE_I2C or SSD1306_USE_SPI.
 * - If SSD1306_USE_DMA is enabled on I2C, make sure the project provides:
 *   - an I2C TX DMA channel linked to the chosen I2C instance,
 *   - HAL_I2C_MasterTxCpltCallback() and HAL_I2C_ErrorCallback() forwarding,
 *   - DMA-safe memory for SSD1306_DMA_BUFFER_ATTRIBUTE.
 * - If SSD1306_USE_RTOS is enabled, the library uses osDelay() and a semaphore wait path once the
 *   kernel is running, but still remains usable before the scheduler starts.
 * - Include only the fonts your UI actually uses to keep flash usage under control.
 */

#ifndef __SSD1306_CONF_H__
#define __SSD1306_CONF_H__

/* Select the MCU family so ssd1306.h pulls in the matching HAL header. */
//#define STM32F0
//#define STM32F1
//#define STM32F4
//#define STM32L0
//#define STM32L1
//#define STM32L4
//#define STM32F3
#define STM32H7
//#define STM32F7
//#define STM32G0
//#define STM32C0
//#define STM32U5

/* Select exactly one transport implementation. */
#define SSD1306_USE_I2C
//#define SSD1306_USE_SPI

/* I2C instance and 7-bit display address used by the OLED module. */
#define SSD1306_I2C_PORT        hi2c1
#define SSD1306_I2C_ADDR        (0x3C << 1)

/* Enable the DMA-backed full-frame flush path. */
#define SSD1306_USE_DMA

/* Enable CMSIS-RTOS2-aware delays and DMA completion waits once the kernel is running. */
#define SSD1306_USE_RTOS

/* Timeout used by blocking command writes and by the pre-kernel fallback transfer path. */
#define SSD1306_I2C_TIMEOUT_MS  250U

/* Place the DMA staging buffer in DMA-safe memory and keep it cache-line aligned. */
#define SSD1306_DMA_BUFFER_ATTRIBUTE __attribute__((section(".i2c_buffers"), aligned(32)))

/* Optional SPI board wiring if SSD1306_USE_SPI is selected instead of I2C. */
//#define SSD1306_SPI_PORT        hspi1
//#define SSD1306_CS_Port         OLED_CS_GPIO_Port
//#define SSD1306_CS_Pin          OLED_CS_Pin
//#define SSD1306_DC_Port         OLED_DC_GPIO_Port
//#define SSD1306_DC_Pin          OLED_DC_Pin
//#define SSD1306_Reset_Port      OLED_Res_GPIO_Port
//#define SSD1306_Reset_Pin       OLED_Res_Pin

/* Optional controller orientation flags. */
// #define SSD1306_MIRROR_VERT
// #define SSD1306_MIRROR_HORIZ

/* Optional inversion flag. */
// # define SSD1306_INVERSE_COLOR

/* Include only the fonts your UI actually uses. */
#define SSD1306_INCLUDE_FONT_6x8
#define SSD1306_INCLUDE_FONT_7x10
#define SSD1306_INCLUDE_FONT_11x18
#define SSD1306_INCLUDE_FONT_16x26

#define SSD1306_INCLUDE_FONT_16x24

#define SSD1306_INCLUDE_FONT_16x15

/* Override the panel width if your module is not 128 pixels wide. */
// #define SSD1306_WIDTH           64

/* Set a controller RAM column offset if the visible panel does not start at column 0. */
// #define SSD1306_X_OFFSET

/* Override the panel height when using 32, 64, or 128 pixel high modules. */
// #define SSD1306_HEIGHT          64

#endif /* __SSD1306_CONF_H__ */
