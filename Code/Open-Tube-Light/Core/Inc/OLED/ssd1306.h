/**
 * @file ssd1306.h
 * @brief Framebuffer-based SSD1306/SSD1315 OLED driver adapted for HAL, FreeRTOS, and optional I2C DMA.
 *
 * Original base library:
 * - Olivier Van den Eede (4ilo), 2016
 * - SPI refactor by Aleksander Alekseev (afiskon), 2018
 * - Project integration for STM32H7, FreeRTOS, and I2C DMA in this repository
 *
 * Usage model:
 * 1. Configure the transport and feature macros in ssd1306_conf.h.
 * 2. Initialize the underlying peripheral first, then call ssd1306_Init().
 * 3. Draw into the local framebuffer with the draw/text helpers in this header.
 * 4. Push the framebuffer to the OLED with ssd1306_UpdateScreen() or ssd1306_UpdateScreenAsync().
 *
 * Important behavior notes:
 * - Drawing functions only modify the local RAM framebuffer. Nothing appears on the display until
 *   an update function is called.
 * - ssd1306_UpdateScreen() is the compatibility API. Before the kernel starts it uses a blocking
 *   transfer path. After the kernel starts, and when DMA is enabled, it starts a DMA flush and then
 *   waits for completion.
 * - ssd1306_UpdateScreenAsync() is the explicit non-blocking API. It starts the DMA flush and returns
 *   immediately when the runtime transport supports it.
 * - The library is not multi-thread safe. One task should own all OLED access.
 * - When SSD1306_USE_DMA is enabled for I2C, HAL_I2C_MasterTxCpltCallback() and HAL_I2C_ErrorCallback()
 *   must forward to the integration callbacks declared at the bottom of this header.
 */

#ifndef __SSD1306_H__
#define __SSD1306_H__

#include <stddef.h>
#include <stdint.h>
#include <_ansi.h>

_BEGIN_STD_C

#include "ssd1306_conf.h"


#if defined(STM32WB)
#include "stm32wbxx_hal.h"
#elif defined(STM32F0)
#include "stm32f0xx_hal.h"
#elif defined(STM32F1)
#include "stm32f1xx_hal.h"
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#elif defined(STM32L0)
#include "stm32l0xx_hal.h"
#elif defined(STM32L1)
#include "stm32l1xx_hal.h"
#elif defined(STM32L4)
#include "stm32l4xx_hal.h"
#elif defined(STM32L5)
#include "stm32l5xx_hal.h"
#elif defined(STM32F3)
#include "stm32f3xx_hal.h"
#elif defined(STM32H7)
#include "stm32h7xx_hal.h"
#elif defined(STM32F7)
#include "stm32f7xx_hal.h"
#elif defined(STM32G0)
#include "stm32g0xx_hal.h"
#elif defined(STM32G4)
#include "stm32g4xx_hal.h"
#elif defined(STM32C0)
#include "stm32c0xx_hal.h"
#elif defined(STM32U5)
#include "stm32u5xx_hal.h"
#else
#error "SSD1306 library was tested only on STM32F0, STM32F1, STM32F3, STM32F4, STM32F7, STM32L0, STM32L1, STM32L4, STM32H7, STM32G0, STM32G4, STM32WB, STM32C0, STM32U5 MCU families. Please modify ssd1306.h if you know what you are doing. Also please send a pull request if it turns out the library works on other MCU's as well!"
#endif


#ifdef SSD1306_X_OFFSET
#define SSD1306_X_OFFSET_LOWER (SSD1306_X_OFFSET & 0x0F)
#define SSD1306_X_OFFSET_UPPER ((SSD1306_X_OFFSET >> 4) & 0x07)
#else
#define SSD1306_X_OFFSET_LOWER 0
#define SSD1306_X_OFFSET_UPPER 0
#endif

/* vvv I2C config vvv */

#ifndef SSD1306_I2C_PORT
#define SSD1306_I2C_PORT        hi2c1
#endif

#ifndef SSD1306_I2C_ADDR
#define SSD1306_I2C_ADDR        (0x3C << 1)
#endif

/* ^^^ I2C config ^^^ */

/* vvv SPI config vvv */

#ifndef SSD1306_SPI_PORT
#define SSD1306_SPI_PORT        hspi2
#endif

#ifndef SSD1306_CS_Port
#define SSD1306_CS_Port         GPIOB
#endif
#ifndef SSD1306_CS_Pin
#define SSD1306_CS_Pin          GPIO_PIN_12
#endif

#ifndef SSD1306_DC_Port
#define SSD1306_DC_Port         GPIOB
#endif
#ifndef SSD1306_DC_Pin
#define SSD1306_DC_Pin          GPIO_PIN_14
#endif

#ifndef SSD1306_Reset_Port
#define SSD1306_Reset_Port      GPIOA
#endif
#ifndef SSD1306_Reset_Pin
#define SSD1306_Reset_Pin       GPIO_PIN_8
#endif

/* ^^^ SPI config ^^^ */

#if defined(SSD1306_USE_I2C)
extern I2C_HandleTypeDef SSD1306_I2C_PORT;
#elif defined(SSD1306_USE_SPI)
extern SPI_HandleTypeDef SSD1306_SPI_PORT;
#else
#error "You should define SSD1306_USE_SPI or SSD1306_USE_I2C macro!"
#endif

// SSD1306 OLED height in pixels
#ifndef SSD1306_HEIGHT
#define SSD1306_HEIGHT          64
#endif

// SSD1306 width in pixels
#ifndef SSD1306_WIDTH
#define SSD1306_WIDTH           128
#endif

#ifndef SSD1306_BUFFER_SIZE
#define SSD1306_BUFFER_SIZE   SSD1306_WIDTH * SSD1306_HEIGHT / 8
#endif

/** Framebuffer pixel values used by drawing functions. */
typedef enum {
    Black = 0x00, // Black color, no pixel
    White = 0x01  // Pixel is set. Color depends on OLED
} SSD1306_COLOR;

/** Result codes returned by transport-aware API calls. */
typedef enum {
    SSD1306_OK = 0x00,
    SSD1306_ERR = 0x01,
    SSD1306_BUSY = 0x02,
    SSD1306_TIMEOUT = 0x03
} SSD1306_Error_t;

/** Internal cursor and display state. Applications usually do not access this directly. */
typedef struct {
    uint16_t CurrentX;
    uint16_t CurrentY;
    uint8_t Initialized;
    uint8_t DisplayOn;
} SSD1306_t;

/** One point used by ssd1306_Polyline(). */
typedef struct {
    uint8_t x;
    uint8_t y;
} SSD1306_VERTEX;

/** Font descriptor used by the text drawing helpers. */
typedef struct {
	const uint8_t width;                /**< Font width in pixels */
	const uint8_t height;               /**< Font height in pixels */
	const uint16_t *const data;         /**< Pointer to font data array */
    const uint8_t *const char_width;    /**< Proportional character width in pixels (NULL for monospaced) */
} SSD1306_Font_t;

/**
 * @brief Initialize the OLED controller and clear the framebuffer.
 * @note Call this after the selected I2C or SPI peripheral is initialized.
 * @note The function is safe before the scheduler starts.
 */
void ssd1306_Init(void);

/**
 * @brief Fill the entire local framebuffer with one color.
 * @note This does not update the physical display by itself.
 */
void ssd1306_Fill(SSD1306_COLOR color);

/**
 * @brief Push the local framebuffer to the OLED and wait until the update is complete.
 * @note With I2C DMA enabled, this uses DMA after the kernel starts and falls back to blocking I2C
 *       before the kernel starts.
 */
void ssd1306_UpdateScreen(void);

/**
 * @brief Start a framebuffer flush without waiting for completion.
 * @return SSD1306_OK if the transfer was started or completed synchronously.
 * @return SSD1306_BUSY if a previous DMA flush is still active.
 * @return SSD1306_ERR on transport setup failure.
 * @note This is the preferred API inside a task that wants to avoid blocking on the OLED transfer.
 */
SSD1306_Error_t ssd1306_UpdateScreenAsync(void);

/**
 * @brief Wait for the currently active asynchronous flush to finish.
 * @param timeout_ms Wait timeout in milliseconds. Use osWaitForever when running under CMSIS-RTOS2.
 * @return SSD1306_OK on successful completion.
 * @return SSD1306_TIMEOUT if the transfer did not finish before the timeout.
 * @return SSD1306_ERR if no synchronization object exists or the transfer failed.
 */
SSD1306_Error_t ssd1306_WaitForUpdate(uint32_t timeout_ms);

/**
 * @brief Report whether the transport path is idle.
 * @return 1 if no OLED update transfer is active, otherwise 0.
 */
uint8_t ssd1306_IsReady(void);

/** @brief Set or clear one framebuffer pixel. */
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);

/** @brief Draw one character at the current cursor position and advance the cursor. */
char ssd1306_WriteChar(char ch, SSD1306_Font_t Font, SSD1306_COLOR color);

/** @brief Draw a null-terminated string at the current cursor position. */
char ssd1306_WriteString(char* str, SSD1306_Font_t Font, SSD1306_COLOR color);

/** @brief Move the text cursor used by ssd1306_WriteChar() and ssd1306_WriteString(). */
void ssd1306_SetCursor(uint8_t x, uint8_t y);

/** @brief Draw a line segment into the framebuffer. */
void ssd1306_Line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);

/** @brief Draw an arc into the framebuffer. */
void ssd1306_DrawArc(uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SSD1306_COLOR color);

/** @brief Draw an arc plus radius lines from the center to both arc endpoints. */
void ssd1306_DrawArcWithRadiusLine(uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SSD1306_COLOR color);

/** @brief Draw a circle outline into the framebuffer. */
void ssd1306_DrawCircle(uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1306_COLOR color);

/** @brief Draw a filled circle into the framebuffer. */
void ssd1306_FillCircle(uint8_t par_x,uint8_t par_y,uint8_t par_r,SSD1306_COLOR par_color);

/** @brief Draw connected line segments from a vertex array. */
void ssd1306_Polyline(const SSD1306_VERTEX *par_vertex, uint16_t par_size, SSD1306_COLOR color);

/** @brief Draw a rectangle outline into the framebuffer. */
void ssd1306_DrawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);

/** @brief Draw a filled rectangle into the framebuffer. */
void ssd1306_FillRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);

/**
 * @brief Invert color of pixels in rectangle (include border)
 * 
 * @param x1 X Coordinate of top left corner
 * @param y1 Y Coordinate of top left corner
 * @param x2 X Coordinate of bottom right corner
 * @param y2 Y Coordinate of bottom right corner
 * @return SSD1306_Error_t status
 */
SSD1306_Error_t ssd1306_InvertRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);

/** @brief Draw a 1-bit bitmap into the framebuffer. */
void ssd1306_DrawBitmap(uint8_t x, uint8_t y, const unsigned char* bitmap, uint8_t w, uint8_t h, SSD1306_COLOR color);

/**
 * @brief Sets the contrast of the display.
 * @param[in] value contrast to set.
 * @note Contrast increases as the value increases.
 * @note RESET = 7Fh.
 */
void ssd1306_SetContrast(const uint8_t value);

/**
 * @brief Set Display ON/OFF.
 * @param[in] on 0 for OFF, any for ON.
 */
void ssd1306_SetDisplayOn(const uint8_t on);

/**
 * @brief Reads DisplayOn state.
 * @return  0: OFF.
 *          1: ON.
 */
uint8_t ssd1306_GetDisplayOn();

/*
 * Advanced integration API.
 *
 * Most application code should use the framebuffer helpers above and never call these directly.
 * They are exposed so board-specific startup and HAL callback glue can be kept outside the library.
 */

/** @brief Reset the OLED hardware when the selected transport needs it. */
void ssd1306_Reset(void);

/** @brief Send one raw command byte to the controller. */
void ssd1306_WriteCommand(uint8_t byte);

/** @brief Send a raw data block to the controller. */
void ssd1306_WriteData(uint8_t* buffer, size_t buff_size);

/** @brief Replace the local framebuffer with caller-provided bytes. */
SSD1306_Error_t ssd1306_FillBuffer(uint8_t* buf, uint32_t len);

#if defined(SSD1306_USE_I2C) && defined(SSD1306_USE_DMA)
/** @brief Forward HAL_I2C_MasterTxCpltCallback() here when I2C DMA is enabled. */
void ssd1306_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);

/** @brief Forward HAL_I2C_ErrorCallback() here when I2C DMA is enabled. */
void ssd1306_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);
#endif

_END_STD_C

#endif // __SSD1306_H__
