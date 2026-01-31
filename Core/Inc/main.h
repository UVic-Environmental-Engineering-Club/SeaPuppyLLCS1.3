/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
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

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Pump_En_Pin GPIO_PIN_13
#define Pump_En_GPIO_Port GPIOC
#define Pitch_En_3V3_Pin GPIO_PIN_0
#define Pitch_En_3V3_GPIO_Port GPIOF
#define Pitch_Dir_3V3_Pin GPIO_PIN_1
#define Pitch_Dir_3V3_GPIO_Port GPIOF
#define Roll_DIR_Pin GPIO_PIN_3
#define Roll_DIR_GPIO_Port GPIOF
#define En_24V_ADC_Pin GPIO_PIN_5
#define En_24V_ADC_GPIO_Port GPIOF
#define Green_OB_LED1_Pin GPIO_PIN_6
#define Green_OB_LED1_GPIO_Port GPIOF
#define Green_OB_LED2_Pin GPIO_PIN_7
#define Green_OB_LED2_GPIO_Port GPIOF
#define Red_OB_LED1_Pin GPIO_PIN_8
#define Red_OB_LED1_GPIO_Port GPIOF
#define Red_OB_LED2_Pin GPIO_PIN_9
#define Red_OB_LED2_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define ADC_24V_Pin GPIO_PIN_4
#define ADC_24V_GPIO_Port GPIOA
#define Brake_En_Pin GPIO_PIN_4
#define Brake_En_GPIO_Port GPIOC
#define Enable_24V_Pin GPIO_PIN_5
#define Enable_24V_GPIO_Port GPIOC
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define ADC_11V_Pin GPIO_PIN_1
#define ADC_11V_GPIO_Port GPIOB
#define Roll_PWM_Pin GPIO_PIN_10
#define Roll_PWM_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define UART3_TX_Pin GPIO_PIN_8
#define UART3_TX_GPIO_Port GPIOD
#define UART3_RX_Pin GPIO_PIN_9
#define UART3_RX_GPIO_Port GPIOD
#define Pump_Pul_3V3_Pin GPIO_PIN_14
#define Pump_Pul_3V3_GPIO_Port GPIOD
#define Pitch_Pul_3V3_Pin GPIO_PIN_15
#define Pitch_Pul_3V3_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define Enable_11V_Pin GPIO_PIN_7
#define Enable_11V_GPIO_Port GPIOC
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define Pump_Dir_Pin GPIO_PIN_12
#define Pump_Dir_GPIO_Port GPIOC
#define CAN_Set_Zero_Pin GPIO_PIN_2
#define CAN_Set_Zero_GPIO_Port GPIOD
#define Roll_Hall_Pin GPIO_PIN_3
#define Roll_Hall_GPIO_Port GPIOB
#define TOF_SCL_Pin GPIO_PIN_6
#define TOF_SCL_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define TOF_SDA_Pin GPIO_PIN_9
#define TOF_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
