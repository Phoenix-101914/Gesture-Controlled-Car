#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_gpio.h"
#include <stdio.h>
#include <string.h>


#define IMU_Address 0xD0
#define IMU_GPIO_CLOCK RCC_APB2Periph_GPIOB
#define Threshold 1
#define TARGET_SSID "RCCarNet"
#define TARGET_PASS "12345678"
#define TARGET_IP   "192.168.4.1" 
#define TARGET_PORT "8080"

uint16_t count;
static __IO uint32_t usTick;

uint8_t a = 0;
float accel_x = 0;
float accel_y = 0;
float accel_z = 0;

volatile uint32_t adc_result;

void DELAY_Init(void);
void DELAY_Us(uint32_t us);
void DELAY_Ms(uint32_t ms);
I2C_HandleTypeDef  I2C_HandleStruct;

void SystemClock_Config(void){ //Setting Clock HSE - 8MHz After PLL - 168MHz
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	
	HAL_RCC_OscConfig(&RCC_OscInitStruct);
	
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct,FLASH_LATENCY_5);
}

void DELAY_Init(){
	// Configure the SysTick timer to raise interript every 1 us
	SysTick_Config(SystemCoreClock / 1000000);
}

// SysTick_Handler function will be called every 1 us

void SysTick_Handler(){
	if (usTick != 0)
	{
		usTick--;
	}
	count++;
}

void DELAY_Us(uint32_t us){
	// Reload us value
	usTick = us;
	// Wait until usTick reach zero
	while (usTick);
}

void DELAY_Ms(uint32_t ms){
	// Wait until ms reach zero
	while (ms--){
		// Delay 1ms
		DELAY_Us(1000);
	}
}

void GPIO_Init(void){
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIODEN;
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	GPIOA->MODER &= ~((3U << 4) | (3U << 6)); 
  GPIOA->MODER |=  ((2U << 4) | (2U << 6)); 
  GPIOA->AFR[0] |= (0x7 << 8) | (0x7 << 12);
	GPIOD->MODER &= ~(3U << 24);
  GPIOD->MODER |=  (1U << 24);
	USART2->BRR = 0x16D; 
  USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
	
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; //2MHz
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void IMU_Init(){
	uint8_t data;
	__HAL_RCC_I2C1_CLK_ENABLE();
	I2C_HandleStruct.Instance = I2C1;
	I2C_HandleStruct.Init.ClockSpeed = 400000;
	I2C_HandleStruct.Init.DutyCycle = I2C_DUTYCYCLE_2;
	I2C_HandleStruct.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	HAL_I2C_Init(&I2C_HandleStruct);
	
	HAL_I2C_Mem_Read(&I2C_HandleStruct,IMU_Address,0x75,I2C_MEMADD_SIZE_8BIT,&a,1,HAL_MAX_DELAY); //Checking Read
	data = 0x01;
	HAL_I2C_Mem_Write(&I2C_HandleStruct,IMU_Address,0x6B,I2C_MEMADD_SIZE_8BIT,&data,1,HAL_MAX_DELAY); //Setting PWR
	HAL_I2C_Mem_Read(&I2C_HandleStruct,IMU_Address,0x6B,I2C_MEMADD_SIZE_8BIT,&a,1,HAL_MAX_DELAY);
	data = 0x08;
	HAL_I2C_Mem_Write(&I2C_HandleStruct,IMU_Address,0x1C,I2C_MEMADD_SIZE_8BIT,&data,1,HAL_MAX_DELAY);//Setting ACCEL_CONFIG to 4g
	HAL_I2C_Mem_Read(&I2C_HandleStruct,IMU_Address,0x1C,I2C_MEMADD_SIZE_8BIT,&a,1,HAL_MAX_DELAY);
	HAL_I2C_Mem_Write(&I2C_HandleStruct,IMU_Address,0x1B,I2C_MEMADD_SIZE_8BIT,&data,1,HAL_MAX_DELAY);//Setting GYRO_CONFIG to 500dps
	HAL_I2C_Mem_Read(&I2C_HandleStruct,IMU_Address,0x1B,I2C_MEMADD_SIZE_8BIT,&a,1,HAL_MAX_DELAY);
	data = 0x04;
	HAL_I2C_Mem_Write(&I2C_HandleStruct,IMU_Address,0x19,I2C_MEMADD_SIZE_8BIT,&data,1,HAL_MAX_DELAY);//Setting sample rate to 200Hz
	HAL_I2C_Mem_Read(&I2C_HandleStruct,IMU_Address,0x19,I2C_MEMADD_SIZE_8BIT,&a,1,HAL_MAX_DELAY);
	HAL_I2C_Mem_Write(&I2C_HandleStruct,IMU_Address,0x1A,I2C_MEMADD_SIZE_8BIT,&data,1,HAL_MAX_DELAY);//Setting LPF for 20Hz BW
	HAL_I2C_Mem_Read(&I2C_HandleStruct,IMU_Address,0x1A,I2C_MEMADD_SIZE_8BIT,&a,1,HAL_MAX_DELAY);
}

void IMU_Read(){
	uint8_t bits[6];
	HAL_I2C_Mem_Read(&I2C_HandleStruct,IMU_Address,0x3B,I2C_MEMADD_SIZE_8BIT,bits,6,HAL_MAX_DELAY);
	accel_x = 9.8f * ((int16_t)(bits[0]<<8) | bits[1])/8192.0f;
	accel_y = 9.8f * ((int16_t)(bits[2]<<8) | bits[3])/8192.0f;
	accel_z = 9.8f * ((int16_t)(bits[4]<<8) | bits[5])/8192.0f;
}

void UART_Send(char* str) {
	while (*str) {
		while (!(USART2->SR & USART_SR_TXE)); 
		USART2->DR = *str++;                  
	}
}

void ADC_Config(void){
	RCC->AHB1ENR |= (1 << 0); 
	RCC->APB2ENR |= (1 << 8); 
	GPIOA->MODER |= (3 << (1 * 2));
	ADC->CCR |= (1 << 16);
	ADC1->CR2 |= (1 << 1); 
	ADC1->SQR3 = 1;
	ADC1->CR2 |= (1 << 0); 
}

int main(void){
	char buffer[100];
	SystemClock_Config();
	DELAY_Init();
	GPIO_Init();
	DELAY_Ms(2000);
	UART_Send("AT+RST\r\n");
  DELAY_Ms(3000);
  UART_Send("AT+CWMODE=1\r\n");
  DELAY_Ms(1000);
	sprintf(buffer, "AT+CWJAP=\"%s\",\"%s\"\r\n", TARGET_SSID, TARGET_PASS);
  UART_Send(buffer);
  DELAY_Ms(8000);
	GPIOD->ODR |= (1 << 12);
	sprintf(buffer, "AT+CIPSTART=\"TCP\",\"%s\",%s\r\n", TARGET_IP, TARGET_PORT);
  UART_Send(buffer);
  DELAY_Ms(2000);
	IMU_Init();
	ADC_Config();
	ADC1->CR2 |= (1 << 30);
	while(1){
		IMU_Read();
		if (ADC1->SR & (1 << 1)) {
			adc_result = ADC1->DR;
		}
		if(accel_x < -9.8 + Threshold && accel_x > -9.8 - Threshold){//Right turn
			
		}
		else if(accel_x < 9.8 + Threshold && accel_x > 9.8 - Threshold){//Left turn
			
		}
		DELAY_Ms(100);
	}
}
