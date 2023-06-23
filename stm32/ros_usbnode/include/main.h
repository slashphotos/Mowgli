/**
  ******************************************************************************
  * @file    main.h
  * @author  Georg Swoboda <cn@warp.at>
  * @brief   main / bootup and initialization, motor control routines, usb init
  ******************************************************************************
  * Version 1.0.0
  *  
  ******************************************************************************
  */

/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */


/*
* OPENMOWER HIGH STATUS
*/
typedef enum {
  OPENMOWER_STATUS_MOWING = 0,
  OPENMOWER_STATUS_DOCKING,
  OPENMOWER_STATUS_UNDOCKING,
  OPENMOWER_STATUS_IDLE,
  OPENMOWER_STATUS_RECORD,
  OPENMOWER_STATUS_MAX_STATUS,
}openmower_status_e;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define DB_ACTIVE 1
#define DB_TRACE(...)\
            do { if (DB_ACTIVE) debug_printf( __VA_ARGS__); } while (0)

void ADC_Test();
float ADC_BatteryVoltage(uint8_t adc_conversions);
float ADC_ChargeVoltage(uint8_t adc_conversions);
float ADC_ChargeCurrent(uint8_t adc_conversions);
void ChargeController(void);
void EmergencyController(void);
void StatusLEDUpdate(void);
void setDriveMotors(uint8_t left_speed, uint8_t right_speed, uint8_t left_dir, uint8_t right_dir);
void setBladeMotor(uint8_t on_off);
uint8_t crcCalc(uint8_t *msg, uint8_t msg_len);
void msgPrint(uint8_t *msg, uint8_t msg_len);

void logSerial(uint8_t *message);
void chirp(uint8_t count);


extern uint16_t  chargecontrol_pwm_val;
extern uint8_t   chargecontrol_is_charging;
extern uint8_t do_chirp;
extern openmower_status_e main_eOpenmowerStatus;

// uart statistics
extern uint16_t cnt_uart4_overrun;      // master
extern uint16_t cnt_usart2_overrun;     // drive motors

// SPI3 handle (used by spiflash.c)
extern SPI_HandleTypeDef SPI3_Handle;

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void LED_Init();
void TF4_Init();
void RAIN_Sensor_Init();
void PAC5210RESET_Init();
void MASTER_USART_Init();
void DRIVEMOTORS_USART_Init();
void SystemClock_Config();
void ADC1_Init(void);
void TIM1_Init(void);
void TIM2_Init(void);
void TIM3_Init(void);
void MX_DMA_Init(void);
void Emergency_Init(void);
void SPI3_Init();
void SPI3_DeInit();


// UART Wrapper functions to hide HAL bullshit ...
void MASTER_Transmit(uint8_t *buffer, uint8_t len);
void DRIVEMOTORS_Transmit(uint8_t *buffer, uint8_t len);

// Sensor Wrapper functions
int RAIN_Sense(void);
int HALLSTOP_Left_Sense(void);
int HALLSTOP_Right_Sense(void);

int BUTTON_Home(void);
int BUTTON_Play(void);
void BUZZER_SET(uint8_t on_off);

void debug_printf(const char *fmt, ...);


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Rain_Sensor_Pin GPIO_PIN_2
#define Rain_Sensor_GPIO_Port GPIOE
#define Stop_Button_Yellow_Pin GPIO_PIN_0
#define Stop_Button_Yellow_GPIO_Port GPIOC
#define Blade_NTC_Pin GPIO_PIN_2
#define Blade_NTC_GPIO_Port GPIOC
#define Charge_Current_Pin GPIO_PIN_1
#define Charge_Current_GPIO_Port GPIOA
#define Charge_Voltage_Pin GPIO_PIN_2
#define Charge_Voltage_GPIO_Port GPIOA
#define Battery_Voltage_Pin GPIO_PIN_3
#define Battery_Voltage_GPIO_Port GPIOA
#define Perimeter_Sense_Pin GPIO_PIN_6
#define Perimeter_Sense_GPIO_Port GPIOA
#define Charger_Input_Voltage_Pin GPIO_PIN_7
#define Charger_Input_Voltage_GPIO_Port GPIOA
#define High_Voltage_Enable_Pin GPIO_PIN_5
#define High_Voltage_Enable_GPIO_Port GPIOC
#define Buzzer_Pin GPIO_PIN_1
#define Buzzer_GPIO_Port GPIOB
#define Led_D3_Pin GPIO_PIN_2
#define Led_D3_GPIO_Port GPIOB
#define Charger_Control_Lowside_Pin GPIO_PIN_8
#define Charger_Control_Lowside_GPIO_Port GPIOE
#define Charger_Control_Highside_Pin GPIO_PIN_9
#define Charger_Control_Highside_GPIO_Port GPIOE
#define Blade_Motor_Reset_Pin GPIO_PIN_14
#define Blade_Motor_Reset_GPIO_Port GPIOE
#define Driver_Motor_Enable_Pin GPIO_PIN_15
#define Driver_Motor_Enable_GPIO_Port GPIOE
#define Home_Button_Pin GPIO_PIN_13
#define Home_Button_GPIO_Port GPIOB
#define Driver_Motor_EnableD8_Pin GPIO_PIN_8
#define Driver_Motor_EnableD8_GPIO_Port GPIOD
#define Stop_Button_White_Pin GPIO_PIN_8
#define Stop_Button_White_GPIO_Port GPIOC
#define Play_Button_Pin GPIO_PIN_9
#define Play_Button_GPIO_Port GPIOC
#define Mechanical_Tilt_Pin GPIO_PIN_8
#define Mechanical_Tilt_GPIO_Port GPIOA
#define Wheel_lift_blue_Pin GPIO_PIN_0
#define Wheel_lift_blue_GPIO_Port GPIOD
#define Wheel_lift_red_Pin GPIO_PIN_1
#define Wheel_lift_red_GPIO_Port GPIOD
#define Driver_Motor_EnableD7_Pin GPIO_PIN_7
#define Driver_Motor_EnableD7_GPIO_Port GPIOD
#define Perimeter_Sense_Control_Pin GPIO_PIN_8
#define Perimeter_Sense_Control_GPIO_Port GPIOB
#define Perimeter_Sense_ControlB9_Pin GPIO_PIN_9
#define Perimeter_Sense_ControlB9_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
