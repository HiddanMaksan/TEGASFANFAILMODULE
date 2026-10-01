/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : I2C scan + AT24CM01 write/verify combined self-test,
  *                   plus WS2812B PWM+DMA LED driver.
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

#include <math.h>
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
#define ROTARYPUSHBUTTON_Pin GPIO_PIN_5
#define ROTARYPUSHBUTTON_GPIO_Port GPIOA
#define ROTARYCLK_Pin GPIO_PIN_6
#define ROTARYCLK_GPIO_Port GPIOA
#define ROTARYENCODERDATA_Pin GPIO_PIN_7
#define ROTARYENCODERDATA_GPIO_Port GPIOA
#define INTCAN_Pin GPIO_PIN_1
#define INTCAN_GPIO_Port GPIOB
#define LED17_Pin GPIO_PIN_14
#define LED17_GPIO_Port GPIOB
#define LED9_Pin GPIO_PIN_15
#define LED9_GPIO_Port GPIOB
#define LED11_Pin GPIO_PIN_9
#define LED11_GPIO_Port GPIOA
#define LED10_Pin GPIO_PIN_10
#define LED10_GPIO_Port GPIOA
#define PA15_TFT_NSS_Pin GPIO_PIN_15
#define PA15_TFT_NSS_GPIO_Port GPIOA
#define ROTARYENCODERCLOCK_Pin GPIO_PIN_4
#define ROTARYENCODERCLOCK_GPIO_Port GPIOB
#define TFTRS_Pin GPIO_PIN_5
#define TFTRS_GPIO_Port GPIOB
#define TFTRESET_Pin GPIO_PIN_6
#define TFTRESET_GPIO_Port GPIOB
#define WP_Pin GPIO_PIN_7
#define WP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
