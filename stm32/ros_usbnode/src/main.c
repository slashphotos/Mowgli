/**
 ******************************************************************************
 * @file    main.c
 * @author  Georg Swoboda <cn@warp.at>
 * @date    21/09/2022
 * @version 1.0.0
 * @brief   main / bootup and initialization, motor control routines, usb init
 ******************************************************************************
 *
 * compile with -DBOARD_YARDFORCE500 to enable the YF500 GForce pinout
 *
 * ROS integration howto taken from here: https://github.com/Itamare4/ROS_stm32f1_rosserial_USB_VCP (Itamar Eliakim)
 *
 ******************************************************************************
 */

#include "board.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_adc.h"
#include "main.h"
// stm32 custom
#include "panel.h"
#include "blademotor.h"
#include "drivemotor.h"
#include "emergency.h"
#include "blademotor.h"
#include "drivemotor.h"
#include "tim.h"
//#include "ultrasonic_sensor.h"
#include "perimeter.h"
#include "adc.h"
#include "charger.h"
#include "soft_i2c.h"
#include "spiflash.h"
#include "i2c.h"
#include "imu/imu.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "nbt.h"
#include "dma.h"
#include "wwdg.h"

// ros
#include "cpp_main.h"
#include "ringbuffer.h"

#define ROS_PUBLISH_MOWGLI 1
uint8_t usb_serial_command[64];

//static void WATCHDOG_vInit(void);
//tatic void WATCHDOG_Refresh(void);
void TIM4_Init(void);
void HALLSTOP_Sensor_Init(void);

static nbt_t main_chargecontroller_nbt;
static nbt_t main_statusled_nbt;
static nbt_t main_emergency_nbt;
static nbt_t main_blademotor_nbt;
static nbt_t main_drivemotor_nbt;
static nbt_t main_wdg_nbt;
static nbt_t main_buzzer_nbt;
static nbt_t main_ultrasonicsensor_nbt;

volatile uint8_t master_tx_busy = 0;
static uint8_t master_tx_buffer_len;
static char master_tx_buffer[255];

uint8_t do_chirp_duration_counter;
uint8_t do_chirp = 0;

openmower_status_e main_eOpenmowerStatus = OPENMOWER_STATUS_IDLE;

UART_HandleTypeDef MASTER_USART_Handler; // UART  Handle

// Drive Motors DMA
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
TIM_HandleTypeDef TIM3_Handle; // PWM Beeper
TIM_HandleTypeDef TIM4_Handle; // PWM Buzzer
IWDG_HandleTypeDef IwdgHandle = {0};
WWDG_HandleTypeDef WwdgHandle = {0};

typedef enum {
    DISARMED=0,
    ARMED
}STATE_e;


STATE_e status = DISARMED;

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_RTC_Init();
  MX_USART6_UART_Init();
  MX_TIM1_Init();
  MX_WWDG_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  logSerial("System Boot Completed!\n");
  logSerial("Services Initializations started...\n");
  memset (usb_serial_command, '\0', 64);  // clear the buffer
  HAL_GPIO_WritePin(Led_D3_GPIO_Port, Led_D3_Pin,1);
  
  /* init Status update variables*/
  int cycle = 0;

  /* Enable high voltage*/
  HAL_GPIO_WritePin(High_Voltage_Enable_GPIO_Port, High_Voltage_Enable_Pin, 1);
  logSerial("High Voltage Circuit: On\n");


  /* Initialize Driver Motors ESC */
  DRIVEMOTOR_Init();
  logSerial("Driver Motors: Ready\n");
    /* Initialize Blade Motors ESC */
  BLADEMOTOR_Init();
  logSerial("Blade Motor: Ready\n");

  /* Initilize Charge Controler*/
  TIM1->CCR1 = 0;  
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  logSerial("Charge Controler PWM Timers initialized!\n");

  /* Initializing Charger*/
  CHARGER_Init();

  /* Initializing Buzzer*/
  TIM3->CCR4 = 0;  
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  BUZZER_SET(1);
  HAL_Delay (500);
  BUZZER_SET(0);
  logSerial("Buzzer initialized!\n");

  if (SPIFLASH_TestDevice())
  {
    SPIFLASH_Config();
    SPIFLASH_IncBootCounter();
  }
  else
  {
    DB_TRACE(" * SPIFLASH: unable to locate SPI Flash\r\n");
  }
  DB_TRACE(" * SPIFLASH initialized\r\n");
  
  I2C_Init();
  DB_TRACE(" * Hard I2C initialized\r\n");
  if (I2C_Acclerometer_TestDevice())
  {
    I2C_Accelerometer_Setup();
  }
  else
  {
    chirp(3);
    DB_TRACE(" * WARNING: initalization of onboard accelerometer for tilt protection failed !\r\n");
  }
  DB_TRACE(" * Accelerometer (onboard/tilt safety) initialized\r\n");
  SW_I2C_Init();
  DB_TRACE(" * Soft I2C (J18) initialized\r\n");
  DB_TRACE(" * Testing supported IMUs:\r\n");
  IMU_TestDevice();
  IMU_Init();
  IMU_Calibrate();
  EMERGENCY_Init();
  logSerial("Services Initializations completed!\n");
  
  
  //PANEL_Init();
  //DB_TRACE(" * Panel initialized\r\n");
 
  // Initialize Main Timers
  NBT_init(&main_chargecontroller_nbt, 10);
  NBT_init(&main_statusled_nbt, 1000);
  NBT_init(&main_emergency_nbt, 10);
#if (DEBUG_TYPE != DEBUG_TYPE_UART) && (OPTION_ULTRASONIC == 1)
  NBT_init(&main_ultrasonicsensor_nbt, 50);
#endif
  NBT_init(&main_blademotor_nbt, 100);
  NBT_init(&main_drivemotor_nbt, 20);
  NBT_init(&main_wdg_nbt, 10);
  NBT_init(&main_buzzer_nbt, 200);

  DB_TRACE(" * NBT Main timers initialized\r\n");

#ifdef I_DONT_NEED_MY_FINGERS
  DB_TRACE("\r\n");
  DB_TRACE("=========================================================\r\n");
  DB_TRACE(" EMERGENCY/SAFETY FEATURES ARE DISABLED IN board.h ! \r\n");
  DB_TRACE("=========================================================\r\n");
  DB_TRACE("\r\n");
#endif
  // Initialize ROS
  init_ROS();
  DB_TRACE(" * ROS serial node initialized\r\n");
  DB_TRACE("\r\n >>> entering main loop ...\r\n\r\n");
  // <chirp><chirp> means we are in the main loop
  chirp(2);

  while (1)
  {
    chatter_handler();
    motors_handler();
    //panel_handler();
    spinOnce();
    broadcast_handler();

    //DRIVEMOTOR_App_Rx();
    // Perimeter_vApp();

    if (NBT_handler(&main_chargecontroller_nbt))
    {
      ADC_Update();
      CHARGER_Update();
    }
    if (NBT_handler(&main_statusled_nbt))
    {
      StatusLEDUpdate();

      // DB_TRACE("master_rx_STATUS: %d  drivemotors_rx_buf_idx: %d  cnt_usart2_overrun: %x\r\n", master_rx_STATUS, drivemotors_rx_buf_idx, cnt_usart2_overrun);
    }
#if (DEBUG_TYPE != DEBUG_TYPE_UART) && (OPTION_ULTRASONIC == 1)
    /* try to send ros message without delay*/
    if (ULTRASONIC_MessageReceived() == 1)
    {
      ultrasonic_handler();
    }
    if (NBT_handler(&main_ultrasonicsensor_nbt))
    {
      ULTRASONICSENSOR_App();
    }
#endif
#if 0
    if (NBT_handler(&main_wdg_nbt))
    {
      WATCHDOG_Refresh();
    }
#endif
    if (NBT_handler(&main_drivemotor_nbt))
    {
      DRIVEMOTOR_Run();
    }

    if (NBT_handler(&main_blademotor_nbt))
    {

      uint32_t currentTick;
      static uint32_t old_tick;

      BLADEMOTOR_Run();

      // DB_TRACE(" temp : %.2f \n",blade_temperature);
      currentTick = HAL_GetTick();
      DB_TRACE("t: %d \n", (currentTick - old_tick));
      old_tick = currentTick;
    }

    if (NBT_handler(&main_buzzer_nbt))
    {
      // TODO
      if (do_chirp)
      {
        TIM3_Handle.Instance->CCR4 = 10; // chirp on
        TIM4_Handle.Instance->CCR3 = 10; // chirp on
        do_chirp--;
        do_chirp_duration_counter = 0;
      }
      if (do_chirp_duration_counter == 1)
      {
        TIM3_Handle.Instance->CCR4 = 0; // chirp off
        TIM4_Handle.Instance->CCR3 = 0; // chirp off
      }
      do_chirp_duration_counter++;
    }

#ifndef I_DONT_NEED_MY_FINGERS
    if (NBT_handler(&main_emergency_nbt))
    {
      EMERGENCY_Update();
    }
    cycle++;
    if(cycle%10==0){  //100ms
      HAL_GPIO_TogglePin (Led_D3_GPIO_Port, Led_D3_Pin);
    }
    if(cycle%100==0){ //1s
       // STATE_Send();
    }
    if(cycle>1000){
      cycle=1;
    }
    HAL_WWDG_Refresh(&hwwdg);
#endif
  }
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // we never get here ...
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

/**
 * @brief Init the Master Serial Port  - this what connects to the upstream controller
 * @retval None
 */
void MASTER_USART_Init()
{
#ifndef BOARD_YARDFORCE500B
  // enable port and usart clocks
  MASTER_USART_GPIO_CLK_ENABLE();
  MASTER_USART_USART_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct;
  // RX
  GPIO_InitStruct.Pin = MASTER_USART_RX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(MASTER_USART_RX_PORT, &GPIO_InitStruct);

  // TX
  GPIO_InitStruct.Pin = MASTER_USART_TX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
  // GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(MASTER_USART_TX_PORT, &GPIO_InitStruct);

  MASTER_USART_Handler.Instance = MASTER_USART_INSTANCE;     // USART1 (DEV)
  MASTER_USART_Handler.Init.BaudRate = 115200;               // Baud rate
  MASTER_USART_Handler.Init.WordLength = UART_WORDLENGTH_8B; // The word is  8  Bit format
  MASTER_USART_Handler.Init.StopBits = USART_STOPBITS_1;     // A stop bit
  MASTER_USART_Handler.Init.Parity = UART_PARITY_NONE;       // No parity bit
  MASTER_USART_Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE; // No hardware flow control
  MASTER_USART_Handler.Init.Mode = USART_MODE_TX_RX;         // Transceiver mode

  HAL_UART_Init(&MASTER_USART_Handler); // HAL_UART_Init() Will enable  UART1

  /* UART4 DMA Init */
  /* UART4_RX Init */
  hdma_uart4_rx.Instance = DMA2_Stream3;
  hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_uart4_rx.Init.Mode = DMA_NORMAL;
  hdma_uart4_rx.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_uart4_rx) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_LINKDMA(&MASTER_USART_Handler, hdmarx, hdma_uart4_rx);

  /* UART4 DMA Init */
  /* UART4_TX Init */
  hdma_uart4_tx.Instance = DMA2_Stream5;
  hdma_uart4_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_uart4_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_uart4_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_uart4_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_uart4_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_uart4_tx.Init.Mode = DMA_NORMAL;
  hdma_uart4_tx.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_uart4_tx) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_LINKDMA(&MASTER_USART_Handler, hdmatx, hdma_uart4_tx);

  // enable IRQ
  HAL_NVIC_SetPriority(MASTER_USART_IRQ, 0, 0);
  HAL_NVIC_EnableIRQ(MASTER_USART_IRQ);

  __HAL_UART_ENABLE_IT(&MASTER_USART_Handler, UART_IT_TC);
#endif
}

#if 0
/**
 * @brief Init LED
 * @retval None
 */
void LED_Init()
{
  LED_GPIO_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);
}

/**
 * @brief Poll RAIN Sensor
 * @retval 1 if rain is detected, 0 if no rain
 */
int RAIN_Sense(void)
{
  return (!HAL_GPIO_ReadPin(RAIN_SENSOR_PORT, RAIN_SENSOR_PIN)); // pullup, active low
}

/**
 * @brief Init RAIN Sensor (PE2) Input
 * @retval None
 */
void RAIN_Sensor_Init()
{
  RAIN_SENSOR_GPIO_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = RAIN_SENSOR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(RAIN_SENSOR_PORT, &GPIO_InitStruct);
}

/**
 * @brief Init HALL STOP Sensor (PD2&3) Inputs
 * @retval None
 */
void HALLSTOP_Sensor_Init()
{
  HALLSTOP_GPIO_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(HALLSTOP_PORT, &GPIO_InitStruct);
}

/**
 * @brief Poll HALLSTOP_Left_Sense Sensor
 * @retval  1 if trigger , 0 if no stop sensor trigger
 */
int HALLSTOP_Left_Sense(void)
{
#if OPTION_BUMPER == 1
  return (HAL_GPIO_ReadPin(HALLSTOP_PORT, GPIO_PIN_2));
#else
  return 0;
#endif
}

/**
 * @brief Poll HALLSTOP_Right_Sense
 * @retval 1 if trigger , 0 if no stop sensor trigger
 */
int HALLSTOP_Right_Sense(void)
{
#if OPTION_BUMPER == 1
  return (HAL_GPIO_ReadPin(HALLSTOP_PORT, GPIO_PIN_3));
#else
  return 0;
#endif
}

/**
 * @brief Init TF4 (24V Power Switch)
 * @retval None
 */
void TF4_Init()
{
  TF4_GPIO_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = TF4_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(TF4_GPIO_PORT, &GPIO_InitStruct);
}
#endif
void logSerial(uint8_t *message)
{
 CDC_Transmit_FS(message, strlen(message));
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
  while (1)
  {
    DB_TRACE("Error Handler reached, oops\r\n");
    chirp(1);
  }
  /* USER CODE END Error_Handler_Debug */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}
#if 0
/**
 * @brief TIM3 Initialization Function
 *
 * Beeper is on PB1 (PWM)
 *
 * @param None
 * @retval None
 */
void TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
  __HAL_RCC_TIM3_CLK_ENABLE();

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  TIM3_Handle.Instance = TIM3;
  TIM3_Handle.Init.Prescaler = 36000; // 72Mhz -> 2khz
  TIM3_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM3_Handle.Init.Period = 50;
  TIM3_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TIM3_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&TIM3_Handle) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&TIM3_Handle, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&TIM3_Handle) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&TIM3_Handle, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&TIM3_Handle, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /**TIM3 GPIO Configuration
  PB1     ------> TIM3_CH4
  */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
 * @brief TIM4 Initialization Function
 *
 * Buzzer is on PB1 (PWM)
 *
 * @param None
 * @retval None
 */
void TIM4_Init(void)
{
  __HAL_RCC_TIM4_CLK_ENABLE();

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  TIM4_Handle.Instance = TIM4;
  TIM4_Handle.Init.Prescaler = 36000; // 72Mhz -> 2khz
  TIM4_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM4_Handle.Init.Period = 50;
  TIM4_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TIM4_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&TIM4_Handle) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&TIM4_Handle, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&TIM4_Handle) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&TIM4_Handle, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&TIM4_Handle, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  //__HAL_AFIO_REMAP_TIM4_ENABLE(); // to use PD14 it is a full remap

  __HAL_RCC_GPIOB_CLK_ENABLE();
  /**TIM4 GPIO Configuration
  PB1    ------> TIM4_CH3
  */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
 * Enable DMA controller clock
 */
void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration (DRIVE MOTORS)  */

  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);


  /* DMA1_Stream2_IRQn interrupt configuration  (BLADE MOTOR)  */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration (BLADE MOTOR) */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}
#endif

void BUZZER_SET(uint8_t on_off){

  if (on_off)
  {
    TIM3->CCR4 = 10; // chirp on
  }else{
    TIM3->CCR4 = 0; // chirp off
  }
}
/*
 * Update the states for the Emergency, Charge and Low Bat LEDs
 */
void StatusLEDUpdate(void)
{
  if (EMERGENCY_State())
  {
    DB_TRACE("Emergency !");
    PANEL_Set_LED(PANEL_LED_LIFTED, PANEL_LED_FLASH_FAST);
  }
  else
  {
    PANEL_Set_LED(PANEL_LED_LIFTED, PANEL_LED_OFF);
  }

  if (charger_state == CHARGER_STATE_CONNECTED) // Connected to charger
  {
    PANEL_Set_LED(PANEL_LED_CHARGING, PANEL_LED_ON);
  }
  else if (charger_state == CHARGER_STATE_CHARGING_CC) // CC mode 1A
  {
    PANEL_Set_LED(PANEL_LED_CHARGING, PANEL_LED_FLASH_FAST);
  }
  else if (charger_state == CHARGER_STATE_CHARGING_CV) // CV mode
  {
    PANEL_Set_LED(PANEL_LED_CHARGING, PANEL_LED_FLASH_SLOW);
  }
  else
  {
    PANEL_Set_LED(PANEL_LED_CHARGING, PANEL_LED_OFF);
  }

  // show a lowbat warning if battery voltage drops below LOW_BAT_THRESHOLD ? (random guess, needs more testing or a compare to the stock firmware)
  if (battery_voltage <= LOW_CRI_THRESHOLD)
  {
    PANEL_Set_LED(PANEL_LED_BATTERY_LOW, PANEL_LED_FLASH_FAST); // low
  }
  else if (battery_voltage <= LOW_BAT_THRESHOLD)
  {
    PANEL_Set_LED(PANEL_LED_BATTERY_LOW, PANEL_LED_FLASH_SLOW); // really low
  }
  else
  {
    PANEL_Set_LED(PANEL_LED_BATTERY_LOW, PANEL_LED_OFF); // bat ok
  }

  HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN); // flash LED
}

/*
 * print hex bytes
 */
void msgPrint(uint8_t *msg, uint8_t msg_len)
{
  int i;
  DB_TRACE("msg: ");
  for (i = 0; i < msg_len; i++)
  {
    DB_TRACE(" %02x", msg[i]);
  }
  DB_TRACE("\r\n");
}
#if 0
/*
 * calc crc byte
 */
uint8_t crcCalc(uint8_t *msg, uint8_t msg_len)
{
  uint8_t crc = 0x0;
  uint8_t i;

  for (i = 0; i < msg_len; i++)
  {
    crc += msg[i];
  }
  return (crc);
}
#endif
/*
 * 2khz chirps
 */
void chirp(uint8_t count)
{
  uint8_t i;

  for (i = 0; i < count; i++)
  {
    TIM3_Handle.Instance->CCR4 = 10;
    TIM4_Handle.Instance->CCR3 = 10;
    HAL_Delay(100);
    TIM3_Handle.Instance->CCR4 = 0;
    TIM4_Handle.Instance->CCR3 = 0;
    HAL_Delay(50);
  }
}

/*
 * Debug print via MASTER USART
 */
void vprint(const char *fmt, va_list argp)
{
  char string[200];
  if (0 < vsprintf(string, fmt, argp)) // build string
  {
#if DEBUG_TYPE == DEBUG_TYPE_SWO
    for (int i = 0; i < strlen(string); i++)
    {
      ITM_SendChar(string[i]);
    }
#elif DEBUG_TYPE == DEBUG_TYPE_UART
    MASTER_Transmit((unsigned char *)string, strlen(string));
#endif
  }
}

/*
 * Debug print
 */
void debug_printf(const char *fmt, ...)
{
  va_list argp;
  va_start(argp, fmt);
  vprint(fmt, argp);
  va_end(argp);
}

/*
 * Send message via MASTER USART (DMA Normal Mode)
 */
void MASTER_Transmit(uint8_t *buffer, uint8_t len)
{
  // wait until tx buffers are free (send complete)
  while (master_tx_busy)
  {
  }
  master_tx_busy = 1;
  // copy into our master_tx_buffer
  master_tx_buffer_len = len;
  memcpy(master_tx_buffer, buffer, master_tx_buffer_len);
  HAL_UART_Transmit_DMA(&MASTER_USART_Handler, (uint8_t *)master_tx_buffer, master_tx_buffer_len); // send message via UART
}
#if 0
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* TIM1 clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspPostInit 0 */

  /* USER CODE END TIM1_MspPostInit 0 */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PE8     ------> TIM1_CH1N
    PE9     ------> TIM1_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM1_MspPostInit 1 */

  /* USER CODE END TIM1_MspPostInit 1 */
  }
  else if(timHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspPostInit 0 */

  /* USER CODE END TIM3_MspPostInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM3 GPIO Configuration
    PB1     ------> TIM3_CH4
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM3_MspPostInit 1 */

  /* USER CODE END TIM3_MspPostInit 1 */
  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
}

/*
 * Initialize Watchdog - not tested yet (by Nekraus)
 */
static void WATCHDOG_vInit(void)
{
  #if 0
#if defined(DB_ACTIVE)
  /* setup DBGMCU block - stop IWDG at break in debug mode */
  __HAL_FREEZE_IWDG_DBGMCU();
#endif /* DB_ACTIVE */

  /* change the period to 50ms */
  IwdgHandle.Instance = IWDG;
  IwdgHandle.Init.Prescaler = IWDG_PRESCALER_256;
  IwdgHandle.Init.Reload = 0xFFF;
  /* Enable IWDG (LSI automatically enabled by HW) */

  /* if window feature is not applied Init() precedes Start() */
  if (HAL_IWDG_Init(&IwdgHandle) != HAL_OK)
  {
#ifdef DB_ACTIVE
    DB_TRACE(" IWDG init Error\n\r");
#endif /* DB_ACTIVE */
  }

/* Initialize WWDG for run time if applicable */
#if defined(DB_ACTIVE)
  /* setup DBGMCU block - stop WWDG at break in debug mode */
  __HAL_FREEZE_WWDG_DBGMCU();
#endif /* DB_ACTIVE */

  /* Setup period - 20ms */
  __WWDG_CLK_ENABLE();
  WwdgHandle.Instance = WWDG;
  WwdgHandle.Init.Prescaler = WWDG_PRESCALER_8;
  WwdgHandle.Init.Counter = 0x7F; /* 40.02 ms*/
  WwdgHandle.Init.Window = 0x7F;  /* 0ms */
  // if( HAL_WWDG_Init(&WwdgHandle) != HAL_OK )
  {
#ifdef DB_ACTIVE
    DB_TRACE(" WWDG init Error\n\r");
#endif /* DB_ACTIVE */
  }
} /* WATCHDOG_vInit() */

/*
 * Feed the watchdog every 10ms
 */
static void WATCHDOG_Refresh(void)
{
  /* Update WWDG counter */
  WwdgHandle.Instance = WWDG;
  if (HAL_WWDG_Refresh(&WwdgHandle) != HAL_OK)
  {
#ifdef DB_ACTIVE
    DB_TRACE(" WWDG refresh error\n\r");
#endif /* DB_ACTIVE */
  }

  /* Reload IWDG counter */
  IwdgHandle.Instance = IWDG;
  if (HAL_IWDG_Refresh(&IwdgHandle) != HAL_OK)
  {
#ifdef DB_ACTIVE
    DB_TRACE(" IWDG refresh error\n\r");
#endif /* DB_ACTIVE */
  }
  #endif
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  // do nothing here
}

/*
 * called when DMA transfer completes
 * update <xxxx>_tx_busy to let XXXX_Transmit function now when the DMA buffer is free
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
#ifdef MASTER_USART_INSTANCE
  if (huart->Instance == MASTER_USART_INSTANCE)
  {
    if (__HAL_USART_GET_FLAG(&MASTER_USART_Handler, USART_FLAG_TC))
    {
      master_tx_busy = 0;
    }
  }
#endif
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  // do nothing here
}

/*
 * Master UART receive ISR
 * DriveMotors UART receive ISR
 * PANEL UART receive ISR
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#ifdef MASTER_USART_INSTANCE
  if (huart->Instance == MASTER_USART_INSTANCE)
  {
#if (DEBUG_TYPE != DEBUG_TYPE_UART) && (OPTION_ULTRASONIC == 1)
    ULTRASONICSENSOR_ReceiveIT();
#endif
  }
#endif
  if (huart->Instance == BLADEMOTOR_USART_INSTANCE)
  {
    BLADEMOTOR_ReceiveIT();
  }
  else if (huart->Instance == DRIVEMOTORS_USART_INSTANCE)
  {
    DRIVEMOTOR_ReceiveIT();
  }
}
#endif