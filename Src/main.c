/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hall_sensor.h"
#include "main_closed_loop.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
#ifdef DIRECTION_CW
  // Direction = 0x00 
uint16_t Hall_DIR_sequence[] = {  0x00,          
                                      1,       // Hall position 001
                                      2,       // Hall position 010
                                      3,       // Hall position 011
                                      4,       // Hall position 100
                                      5,       // Hall position 101
                                      6,       // Hall position 110
                                        0x00   };
#endif

#ifdef DIRECTION_CCW                                        
#endif




// Motor and Commutation Variables
unsigned int Desired_PWM_DutyCycle, PWM_Update_Counter, ADC_Sample_Counter;
unsigned char Hall_IN=0,PreDriver_Sequence,Motor_Status, Hall_State_Unknown;
unsigned char Motor_status;

uint32_t  counter=0;
unsigned char  error = false;

// ADC Variables
unsigned long ADC_Results[4];
unsigned long Avg_vBUS, Avg_vPOT;
unsigned char SampleADC;
uint32_t sample = 0;

// Closed Loop Variables
unsigned char PID_Execute_Counter, ExecutePID;
unsigned int Expected_Hall_ISRs_1sec;
unsigned int Expected_COMM_PWM_Counts, Measured_COMM_PWM_Counts, JJ[100],j=0;
unsigned int Kp, Ki;
signed int s16_Proportional_Error, s16_Integral_Error, s16_PID_ControlLoop_Output;
unsigned int PID_Applied_PWM_DutyCycle;

//PWM Channel Variables
 uint32_t LW = GPIO_PIN_3 , LS = TIM_CHANNEL_1, HU = TIM_CHANNEL_2 , HV = TIM_CHANNEL_3 , HW = TIM_CHANNEL_4 , LU = GPIO_PIN_1 , LV = GPIO_PIN_2  ;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

void PWM_update (unsigned char Next_Hall_Sequence);
void Start_Motor(void);
void Stop_Motor(void);
void Start_ADC_Conversion(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//************************************************************************************************************************//

void Start_Motor(void)
{
    // Read Speed Input and update dutycycle variable
#ifdef ANALOG_SPEEDIN
    Start_ADC_Conversion();  
    
   
    
    Avg_vBUS = ADC_Results[0] >>3;
    Avg_vPOT = ADC_Results[1] >>3;  
    Desired_PWM_DutyCycle = Avg_vPOT >> 3;    // ADC12 => 4096 ADC counts = 100% speed
                                              // 100% Dutycyle counts = 1066
                                              // ADC Pot Counts/4 = Dutycycle counts required    
#endif  
    
    if (Desired_PWM_DutyCycle < MIN_PWM_DUTYCYCLE)  // if < Min DutyCycle %age - latch to min value, 1023           
        Desired_PWM_DutyCycle = MIN_PWM_DUTYCYCLE;  
    
    if (Desired_PWM_DutyCycle < LOW_KP_KI_DUTYCYCLE)   // Dutycycle threshold which requires different Kp/Ki values
    {
        Kp = 500;
        Ki = 1;
    }
    else
    {
        Kp = 4000;
        Ki = 1;
    }
    // Since Expected_COMM_PWM_Counts change only when ADC input value that is read changes, 
    // the expected value is computed in main while loop
    Expected_Hall_ISRs_1sec = ((unsigned long)(MAX_HALL_ISRs_1SEC)* (unsigned long)(Desired_PWM_DutyCycle))/(TIMER_PWM_PERIOD);          
    Expected_COMM_PWM_Counts = (unsigned long)(TIMER_COUNTER_FREQ)*(unsigned long)(1000)/(Expected_Hall_ISRs_1sec);
                                              // Expected PWM counts based on max speed and speed input         
      
    // Read Hall inputs
    Hall_IN = 0;
    Hall_IN = ((GPIOB->IDR)&0xF000)>>13;
    PWM_update(Hall_IN);
    Motor_Status = Running;
}

void Stop_Motor(void)
{
   
    Motor_Status = Stopped;	
}
//************************************************************************************************************************//
void PWM_update (unsigned char Hall_IN)
{
  //Hall_U=> PB.15; Hall_V=> PB.14; Hall_W=> PB.13
	/*Stop All Signals*/
	  HAL_TIM_PWM_Stop(&htim1,HU);
		HAL_TIM_PWM_Stop(&htim1,HV);
		HAL_TIM_PWM_Stop(&htim1,HW);
	  HAL_GPIO_WritePin(GPIOA,LV,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,LU,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,LW,GPIO_PIN_RESET);
	
	/*Commutate to next sequence*/
      switch(Hall_IN)
      {
      case 1:		// Hall position 001             
        HAL_TIM_PWM_Start(&htim1,HW);
		    HAL_GPIO_WritePin(GPIOA,LW,GPIO_PIN_SET);
         
        break;
        
      case 2:		// Hall position 001          
        HAL_TIM_PWM_Start(&htim1,HV);
		    HAL_GPIO_WritePin(GPIOA,LV,GPIO_PIN_SET); 
            
        break;
    
      case 3:		// Hall position 001            
        HAL_TIM_PWM_Start(&htim1,HW);
		    HAL_GPIO_WritePin(GPIOA,LW,GPIO_PIN_SET);
        break;
    
      case 4:		// Hall position 001            
        HAL_TIM_PWM_Start(&htim1,HU);
		    HAL_GPIO_WritePin(GPIOA,LU,GPIO_PIN_SET);      
        break;
    
      case 5:		// Hall position 001            
        HAL_TIM_PWM_Start(&htim1,HU);
		    HAL_GPIO_WritePin(GPIOA,LU,GPIO_PIN_SET);
        break;
    
      case 6:		// Hall position 001           
        HAL_TIM_PWM_Start(&htim1,HV);
		    HAL_GPIO_WritePin(GPIOA,LV,GPIO_PIN_SET);
        break;
        
      default:
					HAL_GPIO_WritePin(GPIOA,LV,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA,LU,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA,LW,GPIO_PIN_RESET);
          Hall_State_Unknown = true;
        //Stop_Motor();
        break;
      }

  Hall_State_Unknown = false;
}    

void Start_ADC_Conversion(void)
{
	/*Initiate ADC conversion after clearing the buffer*/
   ADC_Results[0] = 0x0;
   ADC_Results[1] = 0x0;
	 if(HAL_OK != HAL_ADC_Start_IT(&hadc1))
		Error_Handler();
 
}





//************************************************************************************************************************//
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	volatile unsigned int i;
  

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	

	/* Calibrate ADC*/
		if(HAL_OK != HAL_ADCEx_Calibration_Start(&hadc1))
		Error_Handler();
		
	/* Initialize Variables */
  ExecutePID = false;                       
  PID_Execute_Counter = 0x0;
  Measured_COMM_PWM_Counts = 0x0;
  Expected_COMM_PWM_Counts = 0x0;
  
  s16_Proportional_Error = 0x0;
  s16_Integral_Error = 0x0;
  s16_PID_ControlLoop_Output = 0x0;
  
  Motor_Status = Stopped;  
  Hall_State_Unknown = true;
  SampleADC = false; 
		
	/* Initialize Timer Peripherals */
	htim1.Instance->ARR= 4095;
	 
	
	while(HAL_GetTick()<4000)
	{
		/*Waiting period before turn on*/;
	}
	
	/*Turn on Gate Driver*/;
	HAL_GPIO_WritePin(GPIOC,GDEN_Pin,GPIO_PIN_SET);
	
	/* Start Low Side PWM*/
	HAL_TIM_PWM_Start(&htim1,LS);
	
	/* Start Motor */
  if ((Motor_Status == Stopped)&&(Hall_State_Unknown == true))
	{Start_Motor();}
	
	/* Start Timer  */
	if(HAL_OK != HAL_TIM_Base_Start_IT(&htim1))
			Error_Handler();
	
	//counter=htim1.Instance->CNT;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		  
		   counter = __HAL_TIM_GET_COUNTER(&htim1);
	     //Measured_COMM_PWM_Counts=3000;
		
		if(SampleADC == true)
      {
          // Trigger ADC Sampling
          #ifdef ANALOG_SPEEDIN
              //Start_ADC_Conversion();  
                              
              
                              
              Avg_vBUS = ADC_Results[0] >> 3;
              Avg_vPOT = ADC_Results[1] >> 3;  
              Desired_PWM_DutyCycle = Avg_vPOT/SPEEDIN_PWM_FACTOR;       
                                                        // ADC12 => 4096 ADC counts = 100% speed
                                                        // 100% Dutycyle counts = TIMER_PWM_PERIOD
                                                        // ADC Pot Counts/SPEEDIN_PWM_FACTOR = Dutycycle counts required 
                  Start_ADC_Conversion();  				
          #endif  
              
          if (Desired_PWM_DutyCycle < MIN_PWM_DUTYCYCLE)    // if < Min DutyCycle %age - latch to min value, 1023           
              Desired_PWM_DutyCycle = MIN_PWM_DUTYCYCLE;       
          // Apply Kp, Ki values depending on desired dutycycle
          if (Desired_PWM_DutyCycle < LOW_KP_KI_DUTYCYCLE)  // Dutycycle threshold which requires different Kp/Ki values
          {
              Kp = 500;
              Ki = 0;
          }
          else
          {
						
              Kp = 4000;
              Ki = 1;
          }
          
          SampleADC = false;
          
          // Since Expected_COMM_PWM_Counts change only when ADC input value that is read changes, 
          // the expected value is computed in main while loop
          Expected_Hall_ISRs_1sec = ((unsigned long)(MAX_HALL_ISRs_1SEC)* (unsigned long)(Desired_PWM_DutyCycle))/(TIMER_PWM_PERIOD);          
          Expected_COMM_PWM_Counts = (unsigned long)(TIMER_COUNTER_FREQ)*(unsigned long)(1000)/(Expected_Hall_ISRs_1sec);
                                              // Expected PWM counts based on max speed and speed input         
      }   

      if((ExecutePID == true) && (Measured_COMM_PWM_Counts!=0))
      {
          // PID Motor Speed Control Loop 
            
          // Compute Proportional and Integral Errors
          s16_Proportional_Error = Expected_COMM_PWM_Counts - Measured_COMM_PWM_Counts; // signed int P error = desired-actual
          s16_Integral_Error += s16_Proportional_Error;                                 // signed int I error = Accumulate P errors
              
          s16_PID_ControlLoop_Output = ((((long)Kp*(long)s16_Proportional_Error) + (((long)Ki*(long)s16_Integral_Error))) >> 14)/3; 
                                                                            // PID Control Loop Equation; Kd = 0
              
          PID_Applied_PWM_DutyCycle = Desired_PWM_DutyCycle + (s16_PID_ControlLoop_Output>>3);
          JJ[j++] = PID_Applied_PWM_DutyCycle;
          if(j>100)
            j=0;
              
          if (PID_Applied_PWM_DutyCycle < MIN_PWM_DUTYCYCLE)   // 1023      
          {
              PID_Applied_PWM_DutyCycle = MIN_PWM_DUTYCYCLE;   // < Min DutyCycle %age - latch to min value, 1023   
              s16_Integral_Error = 0x0;
          }
          if (PID_Applied_PWM_DutyCycle > TIMER_PWM_PERIOD)   // 1023
          {
              PID_Applied_PWM_DutyCycle = TIMER_PWM_PERIOD;   // > Max PWM DutyCycle - latch to 100% or max value, 1023   
              s16_Integral_Error = 0x0;
          }
          
          // Initialize PWM outputs with initial dutycycle counts
          htim1.Instance->CCR1 = PID_Applied_PWM_DutyCycle;
					htim1.Instance->CCR2 = PID_Applied_PWM_DutyCycle;
					htim1.Instance->CCR3 = PID_Applied_PWM_DutyCycle;
					htim1.Instance->CCR4 = PID_Applied_PWM_DutyCycle;
          
          ExecutePID = false;
      }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analog WatchDog 1
  */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.HighThreshold = 0;
  AnalogWDGConfig.LowThreshold = 0;
  AnalogWDGConfig.Channel = ADC_CHANNEL_6;
  AnalogWDGConfig.ITMode = ENABLE;
  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 17;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GDEN_Pin|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 GDEN_Pin PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GDEN_Pin|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ITRIP_Pin */
  GPIO_InitStruct.Pin = ITRIP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ITRIP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HAL_TIM_Base_Start(&htim4);
	
	htim1.Instance->CCR1 = 0;
	htim1.Instance->CCR2 = 0;
	htim1.Instance->CCR3 = 0;
	htim1.Instance->CCR4 = 0;
	
	while(HAL_GPIO_ReadPin(GPIOA,ITRIP_Pin))
	{
		if(__HAL_TIM_GET_COUNTER(&htim4)>300)
		{
		while(1);
		}
		}
	HAL_TIM_Base_Stop(&htim4);
	__HAL_TIM_SET_COUNTER(&htim4,0);
		
	//while (__HAL_TIM_GET_FLAG()!=SET);	error
	htim1.Instance->CCR1 = PID_Applied_PWM_DutyCycle;
	htim1.Instance->CCR2 = PID_Applied_PWM_DutyCycle;
	htim1.Instance->CCR3 = PID_Applied_PWM_DutyCycle;
	htim1.Instance->CCR4 = PID_Applied_PWM_DutyCycle;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// heart beat signal = PWM period
    PID_Execute_Counter++;   
    ADC_Sample_Counter++;

    if(PID_Execute_Counter > PID_EXECUTE_PWM_PERIODS)
    {
        ExecutePID = true;
        PID_Execute_Counter = 0x0;
    }
       
    if(ADC_Sample_Counter > ADC_SAMPLING_PWM_PERIODS)
    {
        ADC_Sample_Counter = 0x0; 
        SampleADC = true;  
    }    
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  
	
	ADC_Results[1] += HAL_ADC_GetValue(&hadc1);            // Move results
  sample=sample+1;  
	
	if (sample >= 8)
    {
      if(HAL_OK != HAL_ADC_Stop_IT(&hadc1))
			Error_Handler();
			
			sample = 0;
		}    
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
error = true ;
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
