/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_it.h"
#include "stdio.h"
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
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim16;
extern int foodOrWater;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
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
  * @brief This function handles Prefetch fault, memory access fault.
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
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
uint32_t start=0,end=0;
uint32_t start1=0,end1=0;

uint8_t flagCH1=0;
uint8_t flagCH2=0;
uint8_t msg[9];
uint8_t msg1[9];
uint8_t sent1=0;
uint8_t sent2=0;
char newline[]= "\r\n";
char water[]= "water: ";
char food[]= "food: ";
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */

	 /* USER CODE BEGIN TIM1_CC_IRQn 0 */
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */
	
	if (((foodOrWater ==0) && (flagCH1 ==0)) || ((foodOrWater ==1) && (flagCH2 ==0)))
	{
			if ((foodOrWater ==0) && (flagCH1 ==0))
			{
				start = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);  // capturing start
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
				flagCH1 =1;

			}
		 if((foodOrWater ==1) && (flagCH2 ==0))
			{
			start1 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_3);  // capturing start
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
								flagCH2 =1;

			}
		}
	
		else{

		if ((foodOrWater ==0) && (flagCH1 ==1))
		{
				end = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);  // capturing end
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
							sprintf((char*)msg,"%f",((end-start) * 0.343/2));
			if(((end-start)* 0.343/2)>32)
			{
					uint8_t out;
					sprintf((char *) &out,"%d",0);
				if(sent1 <=1)
				{
					HAL_UART_Transmit(&huart1, &out,sizeof(out),HAL_MAX_DELAY);
					sent1 = sent1+1;
				}
			}
				HAL_UART_Transmit(&huart2,(uint8_t *)water,7,HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2,msg,9,HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2,(uint8_t *)newline,2,HAL_MAX_DELAY);
				flagCH1 =0;
		}
		 
		if((foodOrWater ==1) && (flagCH2 ==1)) {
				end1 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_3);  // capturing end

				__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
				int x = ((end1-start1)* 0.343/2);
		
				sprintf((char*)msg1,"%f",((end1-start1) * 0.343/2));
				if( x > 32)
				{
					uint8_t out;
					sprintf((char *) &out,"%d",1);
					if(sent2 <=1 )
					{
						HAL_UART_Transmit(&huart1, &out,sizeof(out),HAL_MAX_DELAY);
						sent2=sent2+1;
					}
				}
				HAL_UART_Transmit(&huart2,(uint8_t *)food,6,HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2,msg1,9,HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2,(uint8_t *)newline,2,HAL_MAX_DELAY);
			flagCH2 =0;
		}
	}
		 
  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
