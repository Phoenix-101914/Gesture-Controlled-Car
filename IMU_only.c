#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_gpio.h"


#define IMU_Address 0xD0
#define IMU_GPIO_CLOCK RCC_APB2Periph_GPIOB
uint16_t count;
static __IO uint32_t usTick;
uint8_t a = 0;
float accel_x = 0;
float accel_y = 0;
float accel_z = 0;
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
	//RCC_OscInitStruct.LSEState =; Not used
	//RCC_OscInitStruct.HSIState =; Not used
	//RCC_OscInitStruct.HSICalibrationValue =; Not used
	//RCC_OscInitStruct.LSIState =; Not used
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
	SysTick_Config(SystemCoreClock / 3000000);
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

void GPIO_Init(void){//Configuring GPIO and I2C
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
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
int main(void){
	//HAL_Init();
	SystemClock_Config();
	DELAY_Init();
	GPIO_Init();
	IMU_Init();
	while(1){
		IMU_Read();
		if(accel_x > 0){
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
		}
		if(accel_y > 0){
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
		}
		if(accel_z != 9){
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
		}
		else{
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
		}
		DELAY_Ms(100);
	}
}
