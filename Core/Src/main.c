/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
uint16_t TEST = 0;
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
/* Definitions for Traffic_control */
osThreadId_t Traffic_controlHandle;
const osThreadAttr_t Traffic_control_attributes = {
  .name = "Traffic_control",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SN_Crossing */
osThreadId_t SN_CrossingHandle;
const osThreadAttr_t SN_Crossing_attributes = {
  .name = "SN_Crossing",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for interrupt */
osThreadId_t interruptHandle;
const osThreadAttr_t interrupt_attributes = {
  .name = "interrupt",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for WE_Crossing */
osThreadId_t WE_CrossingHandle;
const osThreadAttr_t WE_Crossing_attributes = {
  .name = "WE_Crossing",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for SN_STATE */
osMessageQueueId_t SN_STATEHandle;
const osMessageQueueAttr_t SN_STATE_attributes = {
  .name = "SN_STATE"
};
/* Definitions for WE_STATE */
osMessageQueueId_t WE_STATEHandle;
const osMessageQueueAttr_t WE_STATE_attributes = {
  .name = "WE_STATE"
};
/* Definitions for PedestrainStatet */
osSemaphoreId_t PedestrainStatetHandle;
const osSemaphoreAttr_t PedestrainStatet_attributes = {
  .name = "PedestrainStatet"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void Traffic_Light_Control(void *argument);
void SN_Crossing_Task(void *argument);
void Button_Interrupt(void *argument);
void WE_Crossing_Task(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of PedestrainStatet */
  PedestrainStatetHandle = osSemaphoreNew(1, 1, &PedestrainStatet_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of SN_STATE */
  SN_STATEHandle = osMessageQueueNew (16, sizeof(uint16_t), &SN_STATE_attributes);

  /* creation of WE_STATE */
  WE_STATEHandle = osMessageQueueNew (16, sizeof(uint16_t), &WE_STATE_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Traffic_control */
  Traffic_controlHandle = osThreadNew(Traffic_Light_Control, NULL, &Traffic_control_attributes);

  /* creation of SN_Crossing */
  SN_CrossingHandle = osThreadNew(SN_Crossing_Task, NULL, &SN_Crossing_attributes);

  /* creation of interrupt */
  interruptHandle = osThreadNew(Button_Interrupt, NULL, &interrupt_attributes);

  /* creation of WE_Crossing */
  WE_CrossingHandle = osThreadNew(WE_Crossing_Task, NULL, &WE_Crossing_attributes);

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
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, G_SN_Pin|Y_SN_Pin|R_SN_Pin|PedestrainMove_SN_Pin
                          |PedestrainStop_SN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, G_WE_Pin|Y_WE_Pin|R_WE_Pin|PedestrainMove_WE_Pin
                          |PedestrainStop_WE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : G_SN_Pin Y_SN_Pin R_SN_Pin PedestrainMove_SN_Pin
                           PedestrainStop_SN_Pin */
  GPIO_InitStruct.Pin = G_SN_Pin|Y_SN_Pin|R_SN_Pin|PedestrainMove_SN_Pin
                          |PedestrainStop_SN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PS_NS_Pin */
  GPIO_InitStruct.Pin = PS_NS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PS_NS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : G_WE_Pin Y_WE_Pin R_WE_Pin PedestrainMove_WE_Pin
                           PedestrainStop_WE_Pin */
  GPIO_InitStruct.Pin = G_WE_Pin|Y_WE_Pin|R_WE_Pin|PedestrainMove_WE_Pin
                          |PedestrainStop_WE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PS_WE_Pin */
  GPIO_InitStruct.Pin = PS_WE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PS_WE_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Traffic_Light_Control */
/**
  * @brief  Function implementing the Traffic_control thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Traffic_Light_Control */
void Traffic_Light_Control(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  if (osSemaphoreAcquire(PedestrainStatetHandle, osWaitForever) == osOK)
	     {
	       /* North-South Green, West-East Red */
	       HAL_GPIO_WritePin(PedestrainStop_SN_GPIO_Port, PedestrainStop_SN_Pin, GPIO_PIN_SET);
	 	   HAL_GPIO_WritePin(PedestrainStop_WE_GPIO_Port, PedestrainStop_SN_Pin, GPIO_PIN_SET);

	       HAL_GPIO_WritePin(GPIOA, G_SN_Pin, GPIO_PIN_SET);
	       HAL_GPIO_WritePin(GPIOA, Y_SN_Pin, GPIO_PIN_RESET);
	       HAL_GPIO_WritePin(GPIOA, R_SN_Pin, GPIO_PIN_RESET);
	       HAL_GPIO_WritePin(GPIOB, R_WE_Pin, GPIO_PIN_SET);
	       HAL_GPIO_WritePin(GPIOB, G_WE_Pin, GPIO_PIN_RESET);
	       HAL_GPIO_WritePin(GPIOB, Y_WE_Pin, GPIO_PIN_RESET);
	       osDelay(8000);

	       /* North-South Yellow, West-East Red */
	       HAL_GPIO_WritePin(GPIOA, G_SN_Pin, GPIO_PIN_RESET);
	       HAL_GPIO_WritePin(GPIOA, Y_SN_Pin, GPIO_PIN_SET);
	       HAL_GPIO_WritePin(GPIOA, R_SN_Pin, GPIO_PIN_RESET);
	       osDelay(2000);

	       /* North-South Red, West-East Green */
	       HAL_GPIO_WritePin(GPIOA, R_SN_Pin, GPIO_PIN_SET);
	       HAL_GPIO_WritePin(GPIOA, G_SN_Pin, GPIO_PIN_RESET);
	       HAL_GPIO_WritePin(GPIOA, Y_SN_Pin, GPIO_PIN_RESET);
	       HAL_GPIO_WritePin(GPIOB, G_WE_Pin, GPIO_PIN_SET);
	       HAL_GPIO_WritePin(GPIOB, Y_WE_Pin, GPIO_PIN_RESET);
	       HAL_GPIO_WritePin(GPIOB, R_WE_Pin, GPIO_PIN_RESET);
	       osDelay(8000);

	       /* North-South Red, West-East Yellow */
	       HAL_GPIO_WritePin(GPIOB, G_WE_Pin, GPIO_PIN_RESET);
	       HAL_GPIO_WritePin(GPIOB, Y_WE_Pin, GPIO_PIN_SET);
	       HAL_GPIO_WritePin(GPIOB, R_WE_Pin, GPIO_PIN_RESET);
	       osDelay(2000);

	       osSemaphoreRelease(PedestrainStatetHandle);
	     }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_SN_Crossing_Task */
/**
* @brief Function implementing the SN_Crossing thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SN_Crossing_Task */
void SN_Crossing_Task(void *argument)
{
  /* USER CODE BEGIN SN_Crossing_Task */
  /* Infinite loop */
  for(;;)
  {
	  if (TEST == S_N)
	  {
		/* North-South pedestrian crossing */
		osThreadSuspend(Traffic_controlHandle);

		HAL_GPIO_WritePin(GPIOA, R_SN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, G_SN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, Y_SN_Pin, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(PedestrainStop_SN_GPIO_Port, PedestrainStop_SN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PedestrainMove_SN_GPIO_Port, PedestrainMove_SN_Pin, GPIO_PIN_SET);
		osDelay(5000);
		HAL_GPIO_WritePin(PedestrainMove_SN_GPIO_Port, PedestrainMove_SN_Pin, GPIO_PIN_RESET);
		osDelay(1000);
		HAL_GPIO_WritePin(PedestrainMove_SN_GPIO_Port, PedestrainMove_SN_Pin, GPIO_PIN_SET);
		osDelay(1000);
		HAL_GPIO_WritePin(PedestrainMove_SN_GPIO_Port, PedestrainMove_SN_Pin, GPIO_PIN_RESET);
		osDelay(1000);
		HAL_GPIO_WritePin(PedestrainMove_SN_GPIO_Port, PedestrainMove_SN_Pin, GPIO_PIN_SET);
		osDelay(1000);
		HAL_GPIO_WritePin(PedestrainMove_SN_GPIO_Port, PedestrainMove_SN_Pin, GPIO_PIN_RESET);
		osDelay(1000);


		HAL_GPIO_WritePin(PedestrainMove_SN_GPIO_Port, PedestrainMove_SN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PedestrainStop_SN_GPIO_Port, PedestrainStop_SN_Pin, GPIO_PIN_SET);
		TEST = 0;
		osThreadResume(Traffic_controlHandle);
	  }

	  osDelay(100);
  }
  /* USER CODE END SN_Crossing_Task */
}

/* USER CODE BEGIN Header_Button_Interrupt */
/**
* @brief Function implementing the interrupt thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Button_Interrupt */
void Button_Interrupt(void *argument)
{
	uint32_t last_tick = osKernelGetTickCount();
	    const uint32_t debounce_time = 50; // Debounce time in milliseconds

	    for (;;)
	    {
	        // Check North-South button
	        if (HAL_GPIO_ReadPin(PS_NS_GPIO_Port, PS_NS_Pin) == GPIO_PIN_RESET)
	        {
	            // Debounce check
	        	osDelay(debounce_time);
	            if ((osKernelGetTickCount() - last_tick) >= debounce_time)
	            {
	                printf("North-South button pressed!\n");

	                // Create and send a message for North-South pedestrian crossing
	                TEST = 10;

	                last_tick = osKernelGetTickCount(); // Update last tick for debouncing
	            }
	        }
	        // Check West-East button
	        if (HAL_GPIO_ReadPin(PS_WE_GPIO_Port, PS_WE_Pin) == GPIO_PIN_RESET)
	        {
	            // Debounce check
	        	osDelay(debounce_time);
	            if ((osKernelGetTickCount() - last_tick) >= debounce_time)
	            {
	                printf("West-East button pressed!\n");

	                // Create and send a message for West-East pedestrian crossing
	                TEST =20;

	                last_tick = osKernelGetTickCount(); // Update last tick for debouncing
	            }
	        }

	        // Delay to reduce CPU usage
	        osDelay(10);
	    }
}

/* USER CODE BEGIN Header_WE_Crossing_Task */
/**
* @brief Function implementing the WE_Crossing thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_WE_Crossing_Task */
void WE_Crossing_Task(void *argument)
{
  /* USER CODE BEGIN WE_Crossing_Task */
  /* Infinite loop */
  for(;;)
  {
	  if (TEST == W_E)
	  {
		/* North-South pedestrian crossing */
		osThreadSuspend(Traffic_controlHandle);

		HAL_GPIO_WritePin(GPIOB, R_WE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, G_WE_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, Y_WE_Pin, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(PedestrainStop_WE_GPIO_Port, PedestrainStop_WE_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PedestrainMove_WE_GPIO_Port, PedestrainMove_WE_Pin, GPIO_PIN_SET);
		osDelay(5000);

		HAL_GPIO_WritePin(PedestrainMove_WE_GPIO_Port, PedestrainMove_WE_Pin, GPIO_PIN_RESET);
		osDelay(1000);
		HAL_GPIO_WritePin(PedestrainMove_WE_GPIO_Port, PedestrainMove_WE_Pin, GPIO_PIN_SET);
		osDelay(1000);
		HAL_GPIO_WritePin(PedestrainMove_WE_GPIO_Port, PedestrainMove_WE_Pin, GPIO_PIN_RESET);
		osDelay(1000);
		HAL_GPIO_WritePin(PedestrainMove_WE_GPIO_Port, PedestrainMove_WE_Pin, GPIO_PIN_SET);
		osDelay(1000);
		HAL_GPIO_WritePin(PedestrainMove_WE_GPIO_Port, PedestrainMove_WE_Pin, GPIO_PIN_RESET);
		osDelay(1000);

		HAL_GPIO_WritePin(PedestrainMove_WE_GPIO_Port, PedestrainMove_WE_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PedestrainStop_WE_GPIO_Port, PedestrainStop_WE_Pin, GPIO_PIN_SET);
		TEST = 0;
		osThreadResume(Traffic_controlHandle);
	  }

	  osDelay(100);
  }
  /* USER CODE END WE_Crossing_Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
