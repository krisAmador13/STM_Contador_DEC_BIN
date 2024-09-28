/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */
int contador = 0;//Determina el comportamiento de los leds
int contador_estado=0;//Determina el modo de conteo 


uint32_t lastDebounceTime_suma = 0;
uint32_t lastDebounceTime_resta = 0;
uint32_t lastDebounceTime_cambio = 0;

GPIO_PinState lastButtonState_suma = GPIO_PIN_RESET;
GPIO_PinState lastButtonState_resta = GPIO_PIN_RESET;
GPIO_PinState lastButtonState_cambio = GPIO_PIN_RESET;

GPIO_PinState buttonState_suma = GPIO_PIN_RESET;
GPIO_PinState buttonState_resta = GPIO_PIN_RESET;
GPIO_PinState buttonState_cambio = GPIO_PIN_RESET;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
GPIO_PinState debounce_suma(void);
GPIO_PinState debounce_resta(void);
GPIO_PinState debounce_cambio(void);
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

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {debounce_suma();
  debounce_resta();
  debounce_cambio();

	  		  int sumaState = HAL_GPIO_ReadPin(Bot_Suma_GPIO_Port, Bot_Suma_Pin);
	          int restaState = HAL_GPIO_ReadPin(Bot_Resta_GPIO_Port, Bot_Resta_Pin);
	          int cambioState = HAL_GPIO_ReadPin(Bot_Modo_GPIO_Port, Bot_Modo_Pin);


	                      // Lógica de contador para cada caso
	          	  	  	  if(cambioState==GPIO_PIN_RESET){
	          	  	  		  contador_estado++;
	          	  	  	  	  if(contador_estado>1){
	          	  	  	  		  contador_estado=0;
	          	  	  	  	  }
	          	  	  	  	 }

	                      if (sumaState == GPIO_PIN_SET) {
	                          contador++;
	                      } else if (restaState == GPIO_PIN_RESET) {
	                          contador--;
	                      }
	                      // Evita que el contador se pase (overflow y underflow)
	                      if (contador > 15) {
	                          contador = 0;
	                      } else if (contador < 0) {
	                          contador = 0;
	                      }
	          //}





	   if(contador_estado==0){//Activar modo binario 
		 if (contador==0){
			  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET); // Encender LED 1
			  			 // HAL_Delay(500); // Esperar 500 ms
			  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET); // Encender LED 2
			  			 // HAL_Delay(500);
			  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET); // Encender LED 3
			  			 // HAL_Delay(500);
			  HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET); // Encender LED 4
			  HAL_Delay(100);
		  }

		  else if (contador==1) {
	  	  	  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET); // Encender LED 1
	         // HAL_Delay(500); // Esperar 500 ms
	          HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET); // Encender LED 2
	         // HAL_Delay(500);
	          HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET); // Encender LED 3
	         // HAL_Delay(500);
	          HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET); // Encender LED 4
	          HAL_Delay(500);


		  }
		  else if (contador==2) {
		  	  	  	  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET); // Encender LED 1
		  	         // HAL_Delay(500); // Esperar 500 ms
		  	          HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET); // Encender LED 2
		  	         // HAL_Delay(500);
		  	          HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET); // Encender LED 3
		  	         // HAL_Delay(500);
		  	          HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET); // Encender LED 4
		  	          HAL_Delay(500);


		  		  }
		  else if (contador==3) {
					  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET); // Encender LED 1
		  		  	         // HAL_Delay(500); // Esperar 500 ms
					  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET); // Encender LED 2
		  		  	         // HAL_Delay(500);
					  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET); // Encender LED 3
		  		  	         // HAL_Delay(500);
					  HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET); // Encender LED 4
					  HAL_Delay(500);
		  }
		  else if (contador==4) {
					  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET); // Encender LED 1
		  		  		  	         // HAL_Delay(500); // Esperar 500 ms
					  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET); // Encender LED 2
		  		  		  	         // HAL_Delay(500);
					  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET); // Encender LED 3
		  		  		  	         // HAL_Delay(500);
					  HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET); // Encender LED 4
					  HAL_Delay(500);
		  		  }

		  else if (contador==5) {
					  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET); // Encender LED 1
		  		  		  	         // HAL_Delay(500); // Esperar 500 ms
					  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET); // Encender LED 2
		  		  		  	         // HAL_Delay(500);
					  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET); // Encender LED 3
		  		  		  	         // HAL_Delay(500);
					  HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET); // Encender LED 4
					  HAL_Delay(500);
		  		  }
		  else if (contador==6) {
					  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET); // Encender LED 1
		  		  		  		  	         // HAL_Delay(500); // Esperar 500 ms
					  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET); // Encender LED 2
		  		  		  		  	         // HAL_Delay(500);
					  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET); // Encender LED 3
		  		  		  		  	         // HAL_Delay(500);
					  HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET); // Encender LED 4
					  HAL_Delay(500);
		  		  		  }
		  else if (contador==7) {
					  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET); // Encender LED 1
		  		  		  		  	         // HAL_Delay(500); // Esperar 500 ms
					  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET); // Encender LED 2
		  		  		  		  	         // HAL_Delay(500);
					  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET); // Encender LED 3
		  		  		  		  	         // HAL_Delay(500);
					  HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET); // Encender LED 4
					  HAL_Delay(500);
		  		  		  }
		  else if (contador==8) {
					  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET); // Encender LED 1
		  		  		  		  	         // HAL_Delay(500); // Esperar 500 ms
					  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET); // Encender LED 2
		  		  		  		  	         // HAL_Delay(500);
					  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET); // Encender LED 3
		  		  		  		  	         // HAL_Delay(500);
					  HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET); // Encender LED 4
					  HAL_Delay(500);
		  		  		  }
		  else if (contador==9) {
					  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET); // Encender LED 1
		 		  		  		  		  	         // HAL_Delay(500); // Esperar 500 ms
					  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET); // Encender LED 2
		 		  		  		  		  	         // HAL_Delay(500);
					  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET); // Encender LED 3
		 		  		  		  		  	         // HAL_Delay(500);
					  HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET); // Encender LED 4
					  HAL_Delay(500);
		 		  		  		  }
		  else if (contador==10) {
					  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET); // Encender LED 1
		  		 		  		  		  		  	         // HAL_Delay(500); // Esperar 500 ms
					  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET); // Encender LED 2
		  		 		  		  		  		  	         // HAL_Delay(500);
					  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET); // Encender LED 3
		  		 		  		  		  		  	         // HAL_Delay(500);
					  HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET); // Encender LED 4
					  HAL_Delay(500);
		  		 		  		  		  }
		  else if (contador==11) {
					  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET); // Encender LED 1
		  		  		 		  		  		  		  	         // HAL_Delay(500); // Esperar 500 ms
					  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET); // Encender LED 2
		  		  		 		  		  		  		  	         // HAL_Delay(500);
					  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET); // Encender LED 3
		  		  		 		  		  		  		  	         // HAL_Delay(500);
					  HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET); // Encender LED 4
					  HAL_Delay(500);
		  		  		 		  		  		  }
		  else if (contador==12) {
					  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET); // Encender LED 1
		  		  		 		  		  		  		  	         // HAL_Delay(500); // Esperar 500 ms
					  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET); // Encender LED 2
		  		  		 		  		  		  		  	         // HAL_Delay(500);
					  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET); // Encender LED 3
		  		  		 		  		  		  		  	         // HAL_Delay(500);
					  HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET); // Encender LED 4
					  HAL_Delay(500);
		  		  		 		  		  		  }
		  else if (contador==13) {
					  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET); // Encender LED 1
		  		  		 		  		  		  		  	         // HAL_Delay(500); // Esperar 500 ms
					  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET); // Encender LED 2
		  		  		 		  		  		  		  	         // HAL_Delay(500);
					  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET); // Encender LED 3
		  		  		 		  		  		  		  	         // HAL_Delay(500);
					  HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET); // Encender LED 4
					  HAL_Delay(500);
		  		  		 		  		  		  }
		  else if (contador==14) {
					  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET); // Encender LED 1
		  		  		 		  		  		  		  	         // HAL_Delay(500); // Esperar 500 ms
					  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET); // Encender LED 2
		  		  		 		  		  		  		  	         // HAL_Delay(500);
					  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET); // Encender LED 3
		  		  		 		  		  		  		  	         // HAL_Delay(500);
					  HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET); // Encender LED 4
					  HAL_Delay(500);
		  		  		 		  		  		  }
		  if (contador==15) {
					  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET); // Encender LED 1
		  		  		  		 		  		  		  		  	         // HAL_Delay(500); // Esperar 500 ms
					  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET); // Encender LED 2
		  		  		  		 		  		  		  		  	         // HAL_Delay(500);
					  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET); // Encender LED 3
		  		  		  		 		  		  		  		  	         // HAL_Delay(500);
					  HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET); // Encender LED 4
					  HAL_Delay(500);
		  		  		  		 		  		  		  }




	      }


	   if (contador_estado==1){//Activar modo decimal
		   HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
				  if (contador==0){
					  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET); // Encender LED 1
	  			  			 // HAL_Delay(500); // Esperar 500 ms
					  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET); // Encender LED 2
	  			  			 // HAL_Delay(500);
					  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET); // Encender LED 3
	  			  			 // HAL_Delay(500);
					  HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET); // Encender LED 4
					  HAL_Delay(100);
	  		  }

				  else if (contador==1) {
					  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET); // Encender LED 1
					  // HAL_Delay(500); // Esperar 500 ms
					  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET); // Encender LED 2
					  // HAL_Delay(500);
					  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET); // Encender LED 3
					  // HAL_Delay(500);
					  HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET); // Encender LED 4
					  HAL_Delay(500);


	  		  }
				  else if (contador==2) {
	  		  	  	  	  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET); // Encender LED 1
	  		  	         // HAL_Delay(500); // Esperar 500 ms
	  		  	          HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET); // Encender LED 2
	  		  	         // HAL_Delay(500);
	  		  	          HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET); // Encender LED 3
	  		  	         // HAL_Delay(500);
	  		  	          HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET); // Encender LED 4
	  		  	          HAL_Delay(500);


	  		  		  }
				  else if (contador==3) {
	  					  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET); // Encender LED 1
	  		  		  	         // HAL_Delay(500); // Esperar 500 ms
	  					  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET); // Encender LED 2
	  		  		  	         // HAL_Delay(500);
	  					  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET); // Encender LED 3
	  		  		  	         // HAL_Delay(500);
	  					  HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET); // Encender LED 4
	  					  HAL_Delay(500);
	  		  }
				  else if (contador==4) {
	  					  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET); // Encender LED 1
	  		  		  		  	         // HAL_Delay(500); // Esperar 500 ms
	  					  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET); // Encender LED 2
	  		  		  		  	         // HAL_Delay(500);
	  					  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET); // Encender LED 3
	  		  		  		  	         // HAL_Delay(500);
	  					  HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET); // Encender LED 4
	  					  HAL_Delay(500);
	  		  		  }

  	  }
  }
}


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_1_Pin|LED_2_Pin|LED_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Bot_Suma_Pin */
  GPIO_InitStruct.Pin = Bot_Suma_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Bot_Suma_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Bot_Resta_Pin Bot_Modo_Pin */
  GPIO_InitStruct.Pin = Bot_Resta_Pin|Bot_Modo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_4_Pin */
  GPIO_InitStruct.Pin = LED_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_1_Pin LED_2_Pin LED_3_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin|LED_2_Pin|LED_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
GPIO_PinState debounce_suma(void) {
    GPIO_PinState currentState = HAL_GPIO_ReadPin(Bot_Suma_GPIO_Port, Bot_Suma_Pin);
    if (currentState != lastButtonState_suma) {
        lastDebounceTime_suma = HAL_GetTick();
    }

    if ((HAL_GetTick() - lastDebounceTime_suma) > 50) {
        if (currentState != lastButtonState_suma) {
            lastButtonState_suma = currentState;
        }
    }

    return lastButtonState_suma;
}

/**
  * @brief Función de antirrebote para el botón de resta.
  * @retval El estado actual del botón de resta después del antirrebote.
  */
GPIO_PinState debounce_resta(void) {
    GPIO_PinState currentState = HAL_GPIO_ReadPin(Bot_Resta_GPIO_Port, Bot_Resta_Pin);
    if (currentState != lastButtonState_resta) {
        lastDebounceTime_resta = HAL_GetTick();
    }

    if ((HAL_GetTick() - lastDebounceTime_resta) > 50) {
        if (currentState != lastButtonState_resta) {
            lastButtonState_resta = currentState;
        }
    }

    return lastButtonState_resta;
}

/**
  * @brief Función de antirrebote para el botón de cambio de modo.
  * @retval El estado actual del botón de cambio después del antirrebote.
  */
GPIO_PinState debounce_cambio(void) {
    GPIO_PinState currentState = HAL_GPIO_ReadPin(Bot_Modo_GPIO_Port, Bot_Modo_Pin);
    if (currentState != lastButtonState_cambio) {
        lastDebounceTime_cambio = HAL_GetTick();
    }

    if ((HAL_GetTick() - lastDebounceTime_cambio) > 50) {
        if (currentState != lastButtonState_cambio) {
            lastButtonState_cambio = currentState;
        }
    }

    return lastButtonState_cambio;
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

