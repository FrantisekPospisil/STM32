/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

#include "stm32f0xx_ll_adc.h"
#include "stm32f0xx_ll_crs.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern uint8_t		System;
extern uint8_t		Menu;

extern uint16_t		TimerBeep;
extern uint16_t		TimerStart;
extern uint16_t		TimerSec;

extern uint8_t		Temperature[4];
extern uint32_t		TemperatureIn;
extern uint32_t		TemperatureOut;
extern uint16_t		MeanArray[200];
extern uint8_t		MeanPointer;
extern uint16_t		MeanMax;
extern uint32_t		MeanAdd;
extern uint32_t		TemperatureMeasure;

extern uint32_t		AdResult;

extern uint16_t		TemperatureMin;
extern uint16_t		TemperatureMax;
extern uint16_t		TemperatureSet0;
extern uint16_t		TemperatureSet;
extern uint16_t		TemperatureSetStandby;

extern uint16_t		StandbyTemperature;
extern uint16_t		StandbyDifference;
extern uint16_t		StandbyTimerMax;
extern uint16_t		TimerStandby;

extern uint16_t		Const1;
extern uint16_t		Const2;
extern int16_t		Difference;
extern int32_t		DifferenceIntegral;
extern int16_t		SolderPWM;

extern uint16_t		CounterTemperature;

extern uint8_t		EepromBuffer[30];


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
void Beep( uint32_t Tone, uint16_t Time);
void Music( void );
void MusicEnd(void);
void MusicUp(void);

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define TONE_C		64			// kmitocty tonu pro melodie
#define TONE_C1		262
#define TONE_D1		294
#define TONE_E1		330
#define TONE_F1		349
#define TONE_Fx1	371
#define TONE_G1		392
#define TONE_A1		440
#define TONE_H1		494
#define TONE_C2		523
#define TONE_C3		1047
#define TONE_C4		2093
#define TIME_1		1200		// doba trvani tonu v milisekundach vcetne mezery
#define TIME_1a		1180		// doba trvani tonu bez mezery
#define TIME_2		400
#define TIME_2a		380
#define TIME_3		200
#define TIME_3a		180
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI1_NSS_Pin LL_GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define Button_Pin LL_GPIO_PIN_5
#define Button_GPIO_Port GPIOC
#define Button_EXTI_IRQn EXTI4_15_IRQn
#define LCD_D0_Pin LL_GPIO_PIN_0
#define LCD_D0_GPIO_Port GPIOB
#define LCD_D1_Pin LL_GPIO_PIN_1
#define LCD_D1_GPIO_Port GPIOB
#define LCD_D2_Pin LL_GPIO_PIN_2
#define LCD_D2_GPIO_Port GPIOB
#define SPI2_NSS_Pin LL_GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
#define SPI2_H_Pin LL_GPIO_PIN_13
#define SPI2_H_GPIO_Port GPIOB
#define SPI2_W_Pin LL_GPIO_PIN_14
#define SPI2_W_GPIO_Port GPIOB
#define Encoder_0_Pin LL_GPIO_PIN_6
#define Encoder_0_GPIO_Port GPIOC
#define Encoder_0_EXTI_IRQn EXTI4_15_IRQn
#define Encoder_1_Pin LL_GPIO_PIN_7
#define Encoder_1_GPIO_Port GPIOC
#define Heating_Pin LL_GPIO_PIN_8
#define Heating_GPIO_Port GPIOA
#define Buzzer_Pin LL_GPIO_PIN_15
#define Buzzer_GPIO_Port GPIOA
#define LCD_D3_Pin LL_GPIO_PIN_3
#define LCD_D3_GPIO_Port GPIOB
#define LCD_RS_Pin LL_GPIO_PIN_4
#define LCD_RS_GPIO_Port GPIOB
#define LCD_E_Pin LL_GPIO_PIN_5
#define LCD_E_GPIO_Port GPIOB
#define LCD_LED_Pin LL_GPIO_PIN_6
#define LCD_LED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
