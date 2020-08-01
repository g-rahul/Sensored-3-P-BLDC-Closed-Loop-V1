/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main_closed_loop.h"
#include "stdio.h"
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
volatile   uint32_t tick;
volatile   uint32_t Current_PWM;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
extern void PWM_update (unsigned char Next_Hall_Sequence);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
//extern ADC_HandleTypeDef hadc1;
//extern TIM_HandleTypeDef htim1;

extern uint32_t  LS , HU , HV , HW ;
extern unsigned int Expected_COMM_PWM_Counts, Measured_COMM_PWM_Counts;
extern unsigned char PreDriver_Sequence, Hall_IN, Motor_Status, Hall_State_Unknown;
extern unsigned char Hall_DIR_sequence[];
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  HAL_RCC_NMI_IRQHandler();
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

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

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */
	
	/*Get Current PWM Value is it required ???  */
	  Current_PWM = htim1.Instance->CCR1;
	
	/* Shut Down Pwm Outputs */
  HAL_TIM_PWM_Stop(&htim1,HU);
	HAL_TIM_PWM_Stop(&htim1,HV);
	HAL_TIM_PWM_Stop(&htim1,HW);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
	  
	
	/* Start Over Current Timer2 */
	  HAL_TIM_Base_Start(&htim2);
	
	/* Check if OC fault still exist */
	    while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4))
		{ 
		tick = __HAL_TIM_GET_COUNTER(&htim2);	
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
		
	/* Check if OC Condition is present for more than 2uS stop motor */
		if(tick>30)
		{
			while(1)
			{
			HAL_TIM_PWM_Stop(&htim1,LS);
      HAL_TIM_PWM_Stop(&htim1,HU);
	    HAL_TIM_PWM_Stop(&htim1,HV);
	    HAL_TIM_PWM_Stop(&htim1,HW);
	    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET);	
			}
		}
		}
		
  /*Stop Over Current Timer2 and Reset Counter  */
		HAL_TIM_Base_Stop(&htim2);
		__HAL_TIM_SET_COUNTER(&htim2,0);
		
	/*Wait till PWM Timer Update event if not set*/
		while(~(__HAL_TIM_GET_FLAG(&htim2,TIM_FLAG_UPDATE)))
		{}
			
	/*Re Commutate to upadate PWM values back*/	
		Hall_IN = ((GPIOB->IDR)&0xF000)>>13;
		PreDriver_Sequence = Hall_DIR_sequence[Hall_IN]; 
 	  PWM_update(PreDriver_Sequence);
			
  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupts.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */
	
	//Reset all outputs
	HAL_TIM_PWM_Stop(&htim1,HU);
	HAL_TIM_PWM_Stop(&htim1,HV);
	HAL_TIM_PWM_Stop(&htim1,HW);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
	//
	
	HAL_TIM_Base_Stop(&htim3);                                     //stop counter
	
  Measured_COMM_PWM_Counts = (Measured_COMM_PWM_Counts + __HAL_TIM_GET_COUNTER(&htim3))>>1;       
                                                                // Accumulate and avg measured PWM counts per commutation cycle with prev value
	__HAL_TIM_SET_COUNTER(&htim3,0);                              //clear counter
	HAL_TIM_Base_Start(&htim3);                                   //restart counter
	
// Read Hall inputs
		    Hall_IN = ((GPIOB->IDR)&0xF000)>>13;
		    PreDriver_Sequence = Hall_DIR_sequence[Hall_IN]; 
 	      PWM_update(PreDriver_Sequence);
  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
