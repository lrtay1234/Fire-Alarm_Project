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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <strings.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// 1602 I2C address
#define I2C_ADDR 0x27 // I2C address of the PCF8574
// 1602 dimensions
#define LCD_ROWS 2 // Number of rows on the LCD
#define LCD_COLS 16 // Number of columns on the LCD
// 1602 message bit numbers
#define DC_BIT 0 // Data/Command bit (register select bit)
#define EN_BIT 2 // Enable bit
#define BL_BIT 3 // Back light bit
#define D4_BIT 4 // Data 4 bit
#define D5_BIT 5 // Data 5 bit
#define D6_BIT 6 // Data 6 bit
#define D7_BIT 7 // Data 7 bit

// my macro for my display delay
#define Delay(ms) HAL_Delay(ms)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

void CharLCD_Write_Nibble (uint8_t nibble, uint8_t dc);
void CharLCD_Send_Cmd(uint8_t cmd);
void CharLCD_Send_Data(uint8_t data);
void CharLCD_Init();
void CharLCD_Write_String(char *str);
void CharLCD_Set_Cursor(uint8_t row, uint8_t column);
void CharLCD_Clear(void);



void delay_us (uint16_t us);
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void DHT11_Start (void);
uint8_t DHT11_Check_Response (void);
uint8_t DHT11_Read (void);

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

  // these are my GPOI pins, I2C display, and clocks 16 and 3
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM16_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */


  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // start PWM on Timer 3’s Channel 3 (Servo)

  HAL_GPIO_WritePin(Fan_GPIO_Port, Fan_Pin, 0); // turns off fan

  HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, 0); // turns red LED off

  HAL_GPIO_WritePin(Yellow_LED_GPIO_Port, Yellow_LED_Pin, 0); // turns Yellow LED off

  HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, 0); // turns on Green LED

  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0); // this is my buzzer being set to low

  HAL_TIM_Base_Start(&htim16); // start timer 16
    CharLCD_Init();
    CharLCD_Clear();



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

const int PWM_PERIOD = 20000; // num timer cycles for each PWM period
float dutyCycle = 0.025; // start start at 0 degrees for servo

CharLCD_Set_Cursor(0,0);
CharLCD_Write_String("Testing");

// below we are testing the servo to go 180 degrees then back to 0
// then turn on all LEDs then turn them all of

while(dutyCycle < 0.125)
{
 // Update CCR based on duty cycle
 int resetValue = (int) (dutyCycle * PWM_PERIOD);
 TIM3->CCR3 = resetValue;
 dutyCycle += 0.001; // increase duty cycle by 1%
 HAL_Delay(5); // wait 5 ms before updating again
}


Delay(1000);

// below is turning my servo from 0 degrees to 180
while(dutyCycle > 0.025)
  {
   // Update CCR based on duty cycle
   int resetValue = (int) (dutyCycle * PWM_PERIOD);
   TIM3->CCR3 = resetValue;
   dutyCycle -= 0.001; // decrease duty cycle by 1%
   HAL_Delay(5); // wait 5 ms before updating again
  }


Delay(1000);
 // flashing my LEDs to signal testing and that they work
HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, 1);
HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, 1);
HAL_GPIO_WritePin(Yellow_LED_GPIO_Port, Yellow_LED_Pin, 1);

Delay(1000);

HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, 0);
HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, 0);
HAL_GPIO_WritePin(Yellow_LED_GPIO_Port, Yellow_LED_Pin, 0);

Delay(1000);

  while (1)
  {

	  // initialize humidity, temperature, as well as a sum checker as all 8 bits or one byte
	  uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2, SUM;
	    float Temperature_F = 0; // temperature in Faren
	    float Temperature_C = 0; // tempature in Celsius
	    float Humidity = 0; // optional to include humidity
	    uint8_t Presence = 0;
	    char lcd_buffer[20]; // Buffer for text

	        DHT11_Start(); // send the starting 18 ms signal

	        // DHT_Check-Response makes sure the DHT11 and the microcontroller
	        // are communicating
	        Presence = DHT11_Check_Response();

	        if(Presence == 1)
	        {
	        	// byte 2 cant be read for a DHT11, so it jsut sends 0x00
	            Rh_byte1 = DHT11_Read(); // get whole number from humidity
	            Rh_byte2 = DHT11_Read(); // get decimal number for humidity
	            Temp_byte1 = DHT11_Read(); // get whole number for temp
	            Temp_byte2 = DHT11_Read(); // get decimal for temp
	            SUM = DHT11_Read(); // double check the binary math is correct

	            // Basic Checksum to ensure data is correct
	            if (SUM == (Rh_byte1 + Rh_byte2 + Temp_byte1 + Temp_byte2))
	            {
	              // Takes int value and converts to float
	              // also conversion to farenhieght
	          	  Temperature_F = ((float)Temp_byte1 * 1.8) + 32;
	          	  Temperature_C = ((float)Temp_byte1);

	          	  // not used but it is able
	          	  Humidity = (float)Rh_byte1;


	          	  // part 2 display fire above 75 degree Farenheight
	          	  if (Temperature_F >= 75.0)
	          	  {
					  // Red LED turns on
	    	      	  HAL_GPIO_WritePin(Yellow_LED_GPIO_Port, Yellow_LED_Pin, 0);
	          		  HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, 1);
	          		  HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, 0);

		          	  // Display Temperature
		          	  CharLCD_Set_Cursor(0,0);

		          	  // sprintf - string print formatted
		          	  // it is writing text into the lcd_buffer spot in memory
		          	  // tempt is a string, % - insert varible, .1 - round to 1 decimal place
		          	  // f means its a float, F (space+F) copy text
		          	  // temperature is the variable for %.1f
		          	  sprintf(lcd_buffer, "Tempt: %.1f F    ", Temperature_F);
		          	  CharLCD_Write_String(lcd_buffer);

		          	  CharLCD_Set_Cursor(1,0);

		          	  sprintf(lcd_buffer, "Tempt: %.1f C  ", Temperature_C);
		          	  CharLCD_Write_String(lcd_buffer);

		          	  Delay(3000);
		          	  // Display Fire get out in row two
		          	  CharLCD_Set_Cursor(0,0);

		          	  // Display get out!!
		          	  CharLCD_Write_String("FIRE GET OUT!!");

		          	  CharLCD_Set_Cursor(1,0);

		          	  // Clear display
		          	  CharLCD_Write_String("                       ");


		          	  // turn on buzzer
		          	  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);

		          	  Delay(2000);


		        	  while(dutyCycle < 0.125)
		        	  {
		        	   // Update CCR based on duty cycle
		        	   int resetValue = (int) (dutyCycle * PWM_PERIOD);
		        	   TIM3->CCR3 = resetValue;
		        	   dutyCycle += 0.001; // increase duty cycle by 1%
		        	   HAL_Delay(5); // wait 100 ms before updating again
		        	  }

		        	  Delay(1000);

		          	  // Turn on the fan
		          	  HAL_GPIO_WritePin(Fan_GPIO_Port, Fan_Pin, 1);

	          	  }else{

	          		  // turn all Leds except the green one
	          		  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
	    	      	  HAL_GPIO_WritePin(Yellow_LED_GPIO_Port, Yellow_LED_Pin, 0);
	          		  HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, 0);
	          		  HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, 1);

		        	  while(dutyCycle > 0.025)
		        	  {
		        	   // Update CCR based on duty cycle
		        	   int resetValue = (int) (dutyCycle * PWM_PERIOD);
		        	   TIM3->CCR3 = resetValue;
		        	   dutyCycle -= 0.001; // decrease duty cycle by 1%
		        	   HAL_Delay(5); // wait 100 ms before updating again
		        	  }

	          		  HAL_GPIO_WritePin(Fan_GPIO_Port, Fan_Pin, 0);


		          	  // Display Temperature
		          	  CharLCD_Set_Cursor(0,0);

		          	  // sprintf - string print formatted
		          	  // it is writing text into the lcd_buffer spot in memory
		          	  // tempt is a string, % - insert varible, .1 - round to 1 decimal place
		          	  // f means its a float, F (space+F) copy text
		          	  // temperature is the variable for %.1f
		          	  sprintf(lcd_buffer, "Tempt: %.1f F    ", Temperature_F);
		          	  CharLCD_Write_String(lcd_buffer);


		          	  CharLCD_Set_Cursor(1,0);

		          	  sprintf(lcd_buffer, "Tempt: %.1f C   ", Temperature_C);
		          	  CharLCD_Write_String(lcd_buffer);

		          	  Delay(4000);

		          	  // Display Humidity
		          	  CharLCD_Set_Cursor(0,0);

		          	  // COde below is to show humidity
		          	  //sprintf(lcd_buffer, "Tempt: %.1f %%", Humidity);
		          	  //CharLCD_Write_String(lcd_buffer);

		          	  CharLCD_Write_String("Life is good   ");


		          	  CharLCD_Set_Cursor(1,0);

		          	  // Display get out!!
		          	  CharLCD_Write_String("                       ");


		          	  Delay(2000);


	          	  }

	            }
	        }
	        else
	        {
	      	  CharLCD_Set_Cursor(0,0);
	      	  CharLCD_Write_String("Sensor Error  ");

	      	  CharLCD_Set_Cursor(1,0);
	      	  CharLCD_Write_String("                ");

	      	  HAL_GPIO_WritePin(Yellow_LED_GPIO_Port, Yellow_LED_Pin, 1);
      		  HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, 0);
      		  HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, 0);

      		HAL_GPIO_WritePin(Fan_GPIO_Port, Fan_Pin, 0);

	        }

	        HAL_Delay(2000); // DHT11 is slow, wait 2 seconds between reads



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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 79;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Fan_Pin|Yellow_LED_Pin|Red_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Temp_GPIO_Port, Temp_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Buzzer_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Fan_Pin Temp_Pin Yellow_LED_Pin Red_LED_Pin */
  GPIO_InitStruct.Pin = Fan_Pin|Temp_Pin|Yellow_LED_Pin|Red_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Green_LED_Pin */
  GPIO_InitStruct.Pin = Green_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Green_LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


/**
 * @brief Write a 4-bit nibble to the LCD via I2C
 * @param nibble: 4-bit data to send (lower 4 bits)
 * @param dc: data/command (1 = data, 0 = command)
 * @retval None
 */
void CharLCD_Write_Nibble(uint8_t nibble, uint8_t dc) {
 uint8_t data = nibble << D4_BIT; // Shift nibble to D4-D7 position
 data |= dc << DC_BIT; // Set DC bit for data/command selection
 data |= 1 << BL_BIT; // Include backlight state in data
 data |= 1 << EN_BIT; // Set enable bit high
 HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDR << 1, &data, 1, 100); // Send data with EN high
 HAL_Delay(1); // Wait for data setup
 data &= ~(1 << EN_BIT); // Clear enable bit (falling edge triggers LCD)
 HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDR << 1, &data, 1, 100); // Send data with EN low
}


/**
 * @brief Send command to LCD
 * @param cmd: 8-bit command to send to LCD controller
 * @retval None
 */
void CharLCD_Send_Cmd(uint8_t cmd) {
 uint8_t upper_nibble = cmd >> 4; // Extract upper 4 bits
 uint8_t lower_nibble = cmd & 0x0F; // Extract lower 4 bits
 CharLCD_Write_Nibble(upper_nibble, 0); // Send upper nibble (DC=0 for command)
 CharLCD_Write_Nibble(lower_nibble, 0); // Send lower nibble (DC=0 for command)
 if (cmd == 0x01 || cmd == 0x02) { // Clear display or return home commands
 HAL_Delay(2); // These commands need extra time
 }
}


/**
 * @brief Send data (character) to LCD
 * @param data: 8-bit character data to display
 * @retval None
 */
void CharLCD_Send_Data(uint8_t data) {
 uint8_t upper_nibble = data >> 4; // Extract upper 4 bits
 uint8_t lower_nibble = data & 0x0F; // Extract lower 4 bits
 CharLCD_Write_Nibble(upper_nibble, 1); // Send upper nibble (DC=1 for data)
 CharLCD_Write_Nibble(lower_nibble, 1); // Send lower nibble (DC=1 for data)
}


/**
 * @brief Initialize LCD in 4-bit mode via I2C
 * @param None
 * @retval None
 */
void CharLCD_Init() {
 HAL_Delay(50); // Wait for LCD power-on reset (>40ms)
 CharLCD_Write_Nibble(0x03, 0); // Function set: 8-bit mode (first attempt)
 HAL_Delay(5); // Wait >4.1ms
 CharLCD_Write_Nibble(0x03, 0); // Function set: 8-bit mode (second attempt)
 HAL_Delay(1); // Wait >100us
 CharLCD_Write_Nibble(0x03, 0); // Function set: 8-bit mode (third attempt)
 HAL_Delay(1); // Wait >100us
 CharLCD_Write_Nibble(0x02, 0); // Function set: switch to 4-bit mode
 CharLCD_Send_Cmd(0x28); // Function set: 4-bit, 2 lines, 5x8 font
 CharLCD_Send_Cmd(0x0C); // Display control: display on/cursor off/blink off
 CharLCD_Send_Cmd(0x06); // Entry mode: increment cursor, no shift
 CharLCD_Send_Cmd(0x01); // Clear display
}


 /**
  * @brief Write string to LCD at current cursor position
  * @param str: Pointer to null-terminated string
  * @retval None
  */
void CharLCD_Write_String(char *str) {
  while (*str) { // Loop until null terminator
  CharLCD_Send_Data(*str++); // Send each character and increment pointer
  }
 }


 /**
  * @brief Set cursor position on LCD
  * @param row: Row number (0 or 1 for 2-line display)
  * @param column: Column number (0 to display width - 1)
  * @retval None
  */
void CharLCD_Set_Cursor(uint8_t row, uint8_t column) {
  uint8_t address;
  switch (row) {
  case 0:
  address = 0x00; break; // First line starts at address 0x00
  case 1:
  address = 0x40; break; // Second line starts at address 0x40
  default:
  address = 0x00; // Default to first line for invalid row
  }
  address += column; // Add column offset
  CharLCD_Send_Cmd(0x80 | address); // Set DDRAM address command (0x80 + address)
 }


/**
 * @brief Clear LCD display and return cursor to home position
 * @param None
 * @retval None
 */
void CharLCD_Clear(void) {
 CharLCD_Send_Cmd(0x01); // Clear display command
 HAL_Delay(2); // Wait for command execution
}



// Code 1 - 5 I had help from outside resource as well as AI


// 1. MICROSECOND DELAY (Using your TIM16 in microseconds)
void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim16, 0);  // Reset the counter to 0
	while (__HAL_TIM_GET_COUNTER(&htim16) < us);  // Wait until counter reaches the requested delay
}

// 2. "one wire protocol
// The two functions below flip from output and input
// the * is used to point to the port, example Port A is 48000000 then pin is 1, so 48000001
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	// the {0} makes the variables in the struct all 0 so its a clean slate
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // push-pull, send high or low voltages
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // we let go and stop forcing voltage down the wire
	GPIO_InitStruct.Pull = GPIO_NOPULL; // DHT11 usually has external pull-up, or use GPIO_PULLUP if internal needed
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

// 3. DHT11 START SIGNAL
void DHT11_Start (void)
{
	Set_Pin_Output (Temp_GPIO_Port, Temp_Pin);  // Set as Output
	HAL_GPIO_WritePin (Temp_GPIO_Port, Temp_Pin, 0);   // Pull Low
	HAL_Delay(18);   // Wait 18ms (Standard HAL_Delay is ms)

	// Set as Input, also at this time the DHT11 is sending a low for 80us
	// then sending a high for 80 us to say it is responding. (Our slave signal)
	Set_Pin_Input(Temp_GPIO_Port, Temp_Pin);
}

// 4. CHECK RESPONSE
// This makes sure the DHT11 is working properly.
// Send a signal, i wait 40 us, if pin is low at 40 us, then wait 80 us
// then let me see if there is a high signal, if so DHT is working (send 1)
// if not working I'll send a -1
uint8_t DHT11_Check_Response (void)
{
	uint8_t Response = 0;
	delay_us (40); // Wait 40us
	// Check if pin is low (Sensor pulled it low)
	if (!(HAL_GPIO_ReadPin (Temp_GPIO_Port, Temp_Pin)))
	{
		delay_us (80); // Wait 80us
		// Check if pin is high (Sensor pulled it high)
		if ((HAL_GPIO_ReadPin (Temp_GPIO_Port, Temp_Pin))) Response = 1;
		else Response = -1; // Error
	}
	while ((HAL_GPIO_ReadPin (Temp_GPIO_Port, Temp_Pin)));   // Wait for pin to go low again
	return Response;
}

// 5. READ DATA
// this is the key part, this is house the DHT11 communicates with us
// This will read the data 5 times, (40 bits sent in groups of 8, or 5 bytes are sent.)
// 1st 8 bits = Rh_byte1 or read humidity, this is also the whole number
// 2nd 8 bits = Rh_byte2 or read humidity, this is the decimal part, only sends 00000000 for the DHT11
// 3rd 8 bits = temp_byte1 or temperature, this is the whole number
// 4th 8 bits = temp_byte2 or temperature, this is the decimal part, only sends 00000000 for the DHT11
// 5th 8 bits = SUM = Checks to make sure it is adding the bits right, exp 01000001 = 65
uint8_t DHT11_Read (void)
{
	// this makes a loop that will go 8 times, to read 1 byte
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (Temp_GPIO_Port, Temp_Pin)));   // Wait for the pin to go high

		// here we will wait 40 us, if we get a low, we send a 0
		// if we get a high, we will send a 1
		delay_us (40);
		if (!(HAL_GPIO_ReadPin (Temp_GPIO_Port, Temp_Pin)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else
		{
			i|= (1<<(7-j));  // if the pin is high, write 1
		}
		while ((HAL_GPIO_ReadPin (Temp_GPIO_Port, Temp_Pin)));  // wait for the pin to go low
	}
	return i;
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
#ifdef USE_FULL_ASSERT
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

