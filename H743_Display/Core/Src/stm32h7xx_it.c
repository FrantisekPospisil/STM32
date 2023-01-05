/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

	  if(BeepTimer>0) {						// pocita milisekundy kdy ma bezet PWM pro piezomenic
	    BeepTimer--;
		if(BeepTimer==0) {
		  LL_TIM_DisableAllOutputs(TIM2);
		  LL_TIM_DisableCounter(TIM2);
		}
	  }

	  switch(TouchCounter) {
		case 0: {
		  TouchCounter = 3;
		  HAL_GPIO_WritePin(SPI4_NSS_GPIO_Port, SPI4_NSS_Pin, GPIO_PIN_RESET);
		  LL_SPI_TransmitData8(SPI4, 0x90);		// Y
		  break;
		}
		case 1: {
		  TouchCounter = 2;
		  HAL_GPIO_WritePin(SPI4_NSS_GPIO_Port, SPI4_NSS_Pin, GPIO_PIN_SET);
		  break;
		}
		case 2: {
		  TouchCounter = 3;
		  HAL_GPIO_WritePin(SPI4_NSS_GPIO_Port, SPI4_NSS_Pin, GPIO_PIN_RESET);
		  LL_SPI_ReceiveData8(SPI4);
		  TouchX = ((LL_SPI_ReceiveData8(SPI4) << 8) | LL_SPI_ReceiveData8(SPI4)) >> 3;
		  LL_SPI_TransmitData8(SPI4, 0x90);		// Y
		  break;
		}
		case 3: {
		  TouchCounter = 4;
		  LL_SPI_TransmitData8(SPI4, 0x00);
		  LL_SPI_TransmitData8(SPI4, 0x00);
		  break;
		}
		case 4: {
		  TouchCounter = 5;
		  LL_SPI_ReceiveData8(SPI4);
		  TouchY = ((LL_SPI_ReceiveData8(SPI4) << 8) | LL_SPI_ReceiveData8(SPI4)) >> 3;
		  LL_SPI_TransmitData8(SPI4, 0xD0);		// X
		  break;
		}
		case 5: {
		  TouchCounter = 1;
		  LL_SPI_TransmitData8(SPI4, 0x00);
		  LL_SPI_TransmitData8(SPI4, 0x00);

		  if((TouchX > 5) && (TouchY < 4090) && (TouchTimer == 0)) {
		    if(TouchPointer < 16) {
			  TouchXadd += TouchX;
			  TouchYadd += TouchY;
			  TouchPointer++;
		    }
			if(TouchPointer == 1) {
			  TouchX = TouchX - TouchError;
			  TouchY = TouchY - TouchError;
			  if(((TouchXadd - TouchX) > (TouchError * 2)) || ((TouchYadd - TouchY) > (TouchError * 2))) {
			    TouchXadd = 0;
				TouchYadd = 0;
			    TouchPointer = 0;
			  }
			}
			if(TouchPointer == 16) {
			  TouchXadd = TouchXadd >> 4;
			  TouchYadd = TouchYadd >> 4;
			  TouchX = TouchXadd - TouchError;
			  TouchY = TouchYadd - TouchError;
			  if(((TouchXadd - TouchX) < (TouchError * 2)) && ((TouchYadd - TouchY) < (TouchError * 2))) {

				TouchXresult = (( TouchXadd * 800 ) >> 12 );	// prepocita na souradnice displeje
				TouchYresult = 480 - (( TouchYadd * 480 ) >> 12 );
				TouchPointer = 0;
				TouchXadd = 0;
				TouchYadd = 0;
		      }
			}
		  }
		  else {
			TouchPointer = 0;
		  }
		  break;
		}
	  }
	  if(TouchTimer > 0) TouchTimer--;


  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
