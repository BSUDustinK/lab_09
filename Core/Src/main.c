/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c LAB9 ADC
  * @brief          : Main program body
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
#include "usb_host.h"
#include "seg7.h"

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
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
int DelayValue = 50;

static int LookUpTemp[140] = {
	-40,-39,-38,-37,-36,-35,-34,-33,-32,-31,
	-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,
	-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,
	-10,-9,-8,-7,-6,-5,-4,-3,-2,-1,
	0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,
	20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,
	40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59
	,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79
	,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,

};
static int LookUpResistor[140] = {431000,401700,374500,349400,326100,304500,284500,265900,248600,232500,217600,203800,190900,178900,167700,157300,147600,138500,130100,122200,114800,108000,101500,
95530,89910,84660,79740,75140,70830,66780,63000,59450,56110,52990,50050,47300,44710,42280,39990,37840,35820,33910,32120,30430,28840,27340,25920,24590,23330,22140,21020,19970,18970,18020,17130,16290,15490,14740,14030,13350,12720,12110,11540,11000,10490,10000,
9539,9102,8688,8294,7921,7566,7229,6909,6605,6315,6040,5779,5530,5293,5067,4852,4647,4452,4267,4090,3921,3760,3606,3459,3319,3186,3058,2937,2820,2709,2603,2501,2404,2311,2222,2137,2056,1978,1903,1832,1764,1698,1636,1576,1518,1463,1410,1359,1311,1264,1219,1176,1135,1095,1057,1020,
985.2,951.3,918.7,887.3,857.1,828,799.9,772.9,746.8,721.8,697.7,674.5,652.2,630.7,610.1,590.3,571.2,552.9,
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
//static void MX_I2C1_Init(void);
//static void MX_I2S3_Init(void);
//static void MX_SPI1_Init(void);
static void MX_TIM7_Init(void);
//void MX_USB_HOST_Process(void);

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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  /*** Configure GPIOs ***/
  GPIOD->MODER = 0x55555555; // set all Port D pins to outputs

  // Port A mode register - makes A0 to A3 analog pins
  GPIOA->MODER = 0x00000303;

  GPIOE->MODER |= 0x55555555; // Port E mode register - make E8 to E15 outputs

  /*** Configure ADC1 ***/
  RCC->APB2ENR |= 0x1 << 8;// Turns on ADC1 clock by forcing bit 8 of the RCC APB2ENR register to 1 while keeping other bits unchanged

  ADC1->CR2 |= 0x1;// Turns on ADC1 by forcing bit 0 OF CR2 to 1 while keeping other bits unchanged


  /*****************************************************************************************************
  These commands are handled as part of the MX_TIM7_Init() function and don't need to be enabled
  RCC->AHB1ENR |= 1<<5; // Enable clock for timer 7
  __enable_irq(); // Enable interrupts
  NVIC_EnableIRQ(TIM7_IRQn); // Enable Timer 7 Interrupt in the NVIC controller
  *******************************************************************************************************/


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int analog_value, volts, volts_tenths, volts_hundredths, raw_1000, raw_100, raw_10, raw_1, resistorLvl, tempRead, luxRead;
  int i = 0;
  //Seven_Segment(0x15EEADC1 ); //Message for showing that the initial project compiles and runs
  HAL_Delay(2000);
  while (1)
  {

	  	  //Selects which sensor to read from, 0 for the light sensor 4 for the thermistor
		  ADC1->SQR3 = (i == 0? 0: 4);

		  HAL_Delay(1);

		  ADC1->CR2 |= 1<<30; // Start a conversion on ADC1 by forcing bit 30 in CR2 to 1 while keeping other bits unchanged

		  if(ADC1->SR & 1<<1){

			  //reads the value of the Analog Converter and preserves the read out up to 10^-3
		  	analog_value = (ADC1->DR) * 100;
		  	i =(i+1)%2;

		  	switch(i){

		  		// Handles the reading and display of the Light sensor
	  	  	case 0:		//This is where we read the value of the LIGHT SENSOR
	  	  		volts = 3*(analog_value/4095); //converts to range 000-300
	  	  		luxRead = (volts * 128)/10; //Converts to range of 000-3840

	  	  		/**** DISPLAY 1000THS PLACE OF RAW DECIMAL ON DISPLAY 3 ****/
	  	  		raw_1000 = (luxRead/1000)%10; // 1/1000TH'S PLACE
	  	  		Seven_Segment_Digit(3,raw_1000,0); // Digit 3

	  	  		/**** DISPLAY 100THS PLACE OF RAW DECIMAL ON DISPLAY 2 ****/
	  	  		raw_100 = (luxRead/100)%10; // 1/100th's place
	  	  		Seven_Segment_Digit(2,raw_100,0); // Digit 2

	  	  		/**** DISPLAY 10THS PLACE OF RAW DECIMAL ON DISPLAY 1 ****/
	  	  		raw_10 = (luxRead/10)%10;  // 1/10ths
	  	  		Seven_Segment_Digit(1,raw_10,1); // Digit 1

	  	  		/**** DISPLAY 1'S PLACE OF RAW DECIMAL ON DISPLAY 0 ****/
	  	  		raw_1 = luxRead%10;  // 1s
	  	  		Seven_Segment_Digit(0,raw_1,0); // Digit 0

	  	  		break;

	  		case 1:		//This is where we read the value of the thermal resistor

	  			//Find value of resistor using the divider circuit formula reversed.
	  			volts = ((3*analog_value)/4095);  //Converts  to range 000-300 mV
	  			resistorLvl = (3000000/volts) - 10000; //we use 3,000,000 to accomadate multiplying voltage by 100. They factor out in the fraction but preserve the precision we want.
	  			//TODO Finish implementation with a look up table
	  			tempRead = Linear_Interpolation(resistorLvl);

	  			/**** DISPLAY temp 10'S DIGIT ON DISPLAY 7 ****/
	  			if(tempRead < 0) {
	  				Seven_Segment_Digit(7,44,0); // negative
	  			} else {
	  				Seven_Segment_Digit(7,((tempRead/100)%10),0); // Digit 7
	  			}
            
	  			/**** DISPLAY temp 1'S DIGIT ON DISPLAY 6 ****/
	  			//volts_tenths = (tempRead/10)%10; // 1/10s place
	  			Seven_Segment_Digit(6,(tempRead/10)%10,1);  // Digit 6

	  			/**** DISPLAY temp 1/10TH'S DIGIT ON DISPLAY 5 ****/
	  			//volts_hundredths = tempRead%10; // 1/100TH'S PLACE
	  			Seven_Segment_Digit(5,tempRead%10,0); // Digit 5

	  			//Displays C on Display 4
	  	  	  	Seven_Segment_Digit(4,0x0C,0);
	  			break;
	  		}
	  		HAL_Delay(100);
	  	}
	HAL_Delay(1);
  }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 *@brief Returns an approximate value for temperature that is calculated by finding what two values on the table it is between and finds the slope of the line created between the two points.
 *@param int  The value of the resistance created by the thermistor
 *@retval int the temperature in C multiplied by 10 to preserve the tenths place
 *
 */
int Linear_Interpolation(int resistanceReadout){
	int index = 0;
	char foundValue = 0;

	/*find index of the value less than the resistance read out, we choose this because the resistance on the table decends as the index increases.
	 * Once we have found the index we can leave the loop.
	 * */
	while((foundValue == 0) && (index < 139)){
		if(LookUpResistor[index] <= resistanceReadout){
			foundValue = 1;
		} else {
			index++;
		}
	}
  //This if statement ensures we never try to find the element at array[0 - 1]
	if(index == 0){
		return -40; //-40C is the lower limit of the thermistor. any value below this can not be measured. 
	} else {

    //This is the linear interpolation of the end points for the line found in the while loop. This gives us the matching temperature to the resistance
		int Y_o = LookUpTemp[index-1];
		int Y_1 = LookUpTemp[index];
		return 10 * ( Y_o * (LookUpResistor[index] - resistanceReadout) + Y_1 * (resistanceReadout - LookUpResistor[index-1]) ) / ( LookUpResistor[index] - LookUpResistor[index-1] );
	}
}


/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
