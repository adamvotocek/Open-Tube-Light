/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SK9822/sk9822.h"
#include "pixel_render.h"
#include "device_config.h"
#include "dmx_input.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_uart.h"
#include "OLED/ssd1306.h"
#include "OLED/ssd1306_fonts.h"

// Included for OLED status page
#include <stdio.h>
#include "lwip/ip4_addr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONTROL_TASK_FLAG_INIT_READY  0x02U

// OLED status page
#define OLED_STATUS_UPDATE_PERIOD_MS   1000U
#define OLED_STATUS_FLUSH_TIMEOUT_MS   100U
#define OLED_STATUS_LINE_HEIGHT        12U
#define OLED_STATUS_IPV4_TEXT_LEN      IP4ADDR_STRLEN_MAX
#define OLED_STATUS_LINE_BUFFER_LEN    24U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart5;
DMA_HandleTypeDef hdma_uart5_rx;

/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for EffectTask */
osThreadId_t EffectTaskHandle;
const osThreadAttr_t EffectTask_attributes = {
  .name = "EffectTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* USER CODE BEGIN PV */
SK9822_Handle_t hsk9822;

// For OLED status page
extern struct netif gnetif;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART5_Init(void);
static void MX_I2C1_Init(void);
void ControlTaskRun(void *argument);
void EffectTaskRun(void *argument);

/* USER CODE BEGIN PFP */
static const char *OledStatus_GetProtocolText(DeviceConfig_DmxInput_t input_source);
static void OledStatus_FormatIpv4(char *text, size_t text_len, const ip4_addr_t *address);
static void OledStatus_FormatIpv4Bytes(char *text, size_t text_len, const uint8_t address[4]);
static void OledStatus_Draw(void);
static void OledStatus_RunStep(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static const char *OledStatus_GetProtocolText(DeviceConfig_DmxInput_t input_source) {
  switch (input_source) {
    case DMX_INPUT_ARTNET:
      return "Art-Net";

    case DMX_INPUT_SACN:
      return "sACN";

    case DMX_INPUT_DMX512:
      return "DMX512";

    default:
      return "Unknown";
  }
}

static void OledStatus_FormatIpv4(char *text, size_t text_len, const ip4_addr_t *address) {
  if ((text == NULL) || (text_len == 0U) || (address == NULL)) {
    return;
  }

  if (ip4addr_ntoa_r(address, text, (int)text_len) == NULL) {
    text[0] = '?';
    text[1] = '\0';
  }
}

static void OledStatus_FormatIpv4Bytes(char *text, size_t text_len, const uint8_t address[4]) {
  ip4_addr_t ipv4_address;

  if (address == NULL) {
    if ((text != NULL) && (text_len > 0U)) {
      text[0] = '\0';
    }
    return;
  }

  IP4_ADDR(&ipv4_address, address[0], address[1], address[2], address[3]);
  OledStatus_FormatIpv4(text, text_len, &ipv4_address);
}

static void OledStatus_Draw(void) {
  const DeviceConfig_t *cfg = DeviceConfig_Get();
  char segments_line[OLED_STATUS_LINE_BUFFER_LEN];
  char protocol_line[OLED_STATUS_LINE_BUFFER_LEN];
  char dmx_line[OLED_STATUS_LINE_BUFFER_LEN];
  char ip_line[OLED_STATUS_LINE_BUFFER_LEN];
  char mask_line[OLED_STATUS_LINE_BUFFER_LEN];
  char ip_text[OLED_STATUS_IPV4_TEXT_LEN];
  char mask_text[OLED_STATUS_IPV4_TEXT_LEN];

  if (cfg == NULL) {
    return;
  }

  /* Static network config is not applied to lwIP yet, so show saved static
   * values in static mode and the live lease while DHCP is active. */
  if (cfg->network.ip_mode == IP_MODE_STATIC) {
    OledStatus_FormatIpv4Bytes(ip_text, sizeof(ip_text), cfg->network.static_ip);
    OledStatus_FormatIpv4Bytes(mask_text, sizeof(mask_text), cfg->network.static_mask);
  } else {
    OledStatus_FormatIpv4(ip_text, sizeof(ip_text), netif_ip4_addr(&gnetif));
    OledStatus_FormatIpv4(mask_text, sizeof(mask_text), netif_ip4_netmask(&gnetif));
  }

  (void)snprintf(segments_line,
                 sizeof(segments_line),
                 "Seg:%u",
                 (unsigned int)cfg->layout.segment_count);
  (void)snprintf(protocol_line,
                 sizeof(protocol_line),
                 "Prot:%s",
                 OledStatus_GetProtocolText(cfg->dmx.input_source));
  (void)snprintf(dmx_line,
                 sizeof(dmx_line),
                 "DMX:%u",
                 (unsigned int)cfg->dmx.dmx_start_address);
  (void)snprintf(ip_line, sizeof(ip_line), "IP:%s", ip_text);
  (void)snprintf(mask_line, sizeof(mask_line), "Mask:%s", mask_text);

  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  (void)ssd1306_WriteString(segments_line, Font_6x8, White);
  ssd1306_SetCursor(0, OLED_STATUS_LINE_HEIGHT);
  (void)ssd1306_WriteString(protocol_line, Font_6x8, White);
  ssd1306_SetCursor(0, 2U * OLED_STATUS_LINE_HEIGHT);
  (void)ssd1306_WriteString(dmx_line, Font_6x8, White);
  ssd1306_SetCursor(0, 3U * OLED_STATUS_LINE_HEIGHT);
  (void)ssd1306_WriteString(ip_line, Font_6x8, White);
  ssd1306_SetCursor(0, 4U * OLED_STATUS_LINE_HEIGHT);
  (void)ssd1306_WriteString(mask_line, Font_6x8, White);
}

static void OledStatus_RunStep(void) {
  SSD1306_Error_t status;

  OledStatus_Draw();

  /*
   * Start the OLED flush over I2C DMA, then only block when we need bounded
   * confirmation that the transfer completed.
   */
  status = ssd1306_UpdateScreenAsync();
  if (status == SSD1306_BUSY) {
    status = ssd1306_WaitForUpdate(OLED_STATUS_FLUSH_TIMEOUT_MS);
    if (status == SSD1306_OK) {
      status = ssd1306_UpdateScreenAsync();
    }
  }

  if (status == SSD1306_OK) {
    HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
    (void)ssd1306_WaitForUpdate(OLED_STATUS_FLUSH_TIMEOUT_MS);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
    HAL_Delay(100); //startup delay. if this is not here, the code will not run after power cycle for some reason, i did not investigate why yet (probably power, HSE, BOOT0 related?)

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_UART5_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  
  DeviceConfig_Init();

  //5V PSU disabled at startup
  HAL_GPIO_WritePin(PSU_5V_EN_GPIO_Port, PSU_5V_EN_Pin, GPIO_PIN_RESET);
  SK9822_Init(&hsk9822, &hspi1);
  HAL_GPIO_WritePin(PSU_5V_EN_GPIO_Port, PSU_5V_EN_Pin, GPIO_PIN_SET); // This is temporary: we must only enable the PSU if the input voltage is high enough
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(ControlTaskRun, NULL, &ControlTask_attributes);

  /* creation of EffectTask */
  EffectTaskHandle = osThreadNew(EffectTaskRun, NULL, &EffectTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while(1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 50;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00C042E4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 250000;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart5, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_RED_Pin|PSU_5V_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BUTTON3_Pin BUTTON1_Pin BUTTON2_Pin BUTTON4_Pin
                           PSU_5V_PG_Pin USBC_PG_Pin */
  GPIO_InitStruct.Pin = BUTTON3_Pin|BUTTON1_Pin|BUTTON2_Pin|BUTTON4_Pin
                          |PSU_5V_PG_Pin|USBC_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RED_Pin PSU_5V_EN_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|PSU_5V_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    SK9822_OnTxComplete(&hsk9822, hspi);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  ssd1306_I2C_MasterTxCpltCallback(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  ssd1306_I2C_ErrorCallback(hi2c);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_ControlTaskRun */
/**
  * @brief  Function implementing the ControlTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ControlTaskRun */
void ControlTaskRun(void *argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */
  
  // Context for the UI implementation
  // 
  // I2C:
  // - enabled with dma and dma transfer interrupt
  // - added a .i2c_buffers section in D2 for the dma buffers
  // 
  // Button mapping:
  // - BUTTON1: UP
  // - BUTTON2: DOWN
  // - BUTTON3: MENU/BACK
  // - BUTTON4: ENTER/SELECT

  // Start the configured DMX input source (Art-Net, sACN, or DMX512)
  const DeviceConfig_t *cfg = DeviceConfig_Get();
  DMX_Input_Start(cfg->dmx.input_source, EffectTaskHandle);

  /* Keep OLED ownership inside ControlTask so the status page uses the RTOS-aware DMA path. */
  ssd1306_Init();
  OledStatus_RunStep();

  // Release EffectTask only after the protocol input path is configured.
  osThreadFlagsSet(EffectTaskHandle, CONTROL_TASK_FLAG_INIT_READY);

  /* Infinite loop */
  for(;;)
  {
    OledStatus_RunStep();
    osDelay(OLED_STATUS_UPDATE_PERIOD_MS);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_EffectTaskRun */
/**
* @brief Function implementing the EffectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_EffectTaskRun */
void EffectTaskRun(void *argument)
{
  /* USER CODE BEGIN EffectTaskRun */
  // Wait until ControlTask has finished runtime initialization.
  osThreadFlagsWait(CONTROL_TASK_FLAG_INIT_READY, osFlagsWaitAny, osWaitForever);
  Pixel_Render_Init(&hsk9822);

  uint32_t last_frame_tick = 0;
  bool failsafe_applied = false;
  bool first_frame_received = false;  // No failsafe until first frame arrives

  for (;;)
  {
    // Wait for notification from DMX input source or timeout for periodic SPI refresh
    uint32_t flags = osThreadFlagsWait(0x01, osFlagsWaitAny, DMX_INPUT_REFRESH_INTERVAL_MS);
    bool new_frame = !(flags & osFlagsError) && (flags & 0x01);
    uint32_t now = osKernelGetTickCount();

    if (new_frame) {
      DMX_Input_Latch();
      last_frame_tick = now;
      failsafe_applied = false;
      first_frame_received = true;
      Pixel_Render_Frame();

    } else if (first_frame_received && !failsafe_applied &&
               (now - last_frame_tick) > DMX_Input_GetFailsafeTimeout()) {
      // FAILSAFE: controller disconnected
      failsafe_applied = true;
      Pixel_Render_Failsafe();

    } else {
      // IDLE REFRESH: retransmit current pixel data
      Pixel_Render_Refresh();
    }
  }
  /* USER CODE END EffectTaskRun */
}

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0x24000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while(1) {
    }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
