#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_gpio.h"

#define ULTRASONIC_PORT GPIOD

uint16_t count;
static __IO uint32_t usTick;
float leftDist,rightDist,frontDist,backDist;
uint16_t leftTime,rightTime,frontTime,backTime;

void DELAY_Init(void);
void DELAY_Us(uint32_t us);
void DELAY_Ms(uint32_t ms);

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
	__HAL_RCC_GPIOD_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_12; //Trig pins 0-Front 1-Right 2-Left 3-Back
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ULTRASONIC_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7; //Echo pins 4-Front 5-Right 6-Left 7-Back
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ULTRASONIC_PORT, &GPIO_InitStruct);
}

void UltrasonicRead(void){
	// Reading Front
	ULTRASONIC_PORT->BSRR = (1<<16)|(1<<17)|(1<<18)|(1<<19);//Trig LOW
	DELAY_Us(2);
	ULTRASONIC_PORT->BSRR = 1<<0;//Trig HIGH for Front
	DELAY_Us(10);
	ULTRASONIC_PORT->BSRR = 1<<16;//Trig LOW
	
	uint16_t localCount = 0;
	count = 0;
	uint8_t state = 0;
	while(count < 10000 && frontTime == 0){
		if((ULTRASONIC_PORT->IDR & 0x0010) == 0x0010 && state == 0){
			state++;
		}
		if((ULTRASONIC_PORT->IDR & 0x0010) == 0x0000 && state == 1){
			frontTime = localCount;
			state--;
			break;
		}
		localCount++;
		DELAY_Us(1);
	}
	frontDist = (float)frontTime * 0.034f / 2;
	frontTime = 0;
	
	// Reading Right
	ULTRASONIC_PORT->BSRR = (1<<16)|(1<<17)|(1<<18)|(1<<19);//Trig LOW
	DELAY_Us(2);
	ULTRASONIC_PORT->BSRR = 1<<1;//Trig HIGH for Right
	DELAY_Us(10);
	ULTRASONIC_PORT->BSRR = 1<<17;//Trig LOW
	
	localCount = 0;
	count = 0;
	state = 0;
	while(count < 10000 && rightTime == 0){
		if((ULTRASONIC_PORT->IDR & 0x0020) == 0x0020 && state == 0){
			state++;
		}
		if((ULTRASONIC_PORT->IDR & 0x0020) == 0x0000 && state == 1){
			rightTime = localCount;
			state--;
			break;
		}
		localCount++;
		DELAY_Us(1);
	}
	rightDist = (float)rightTime * 0.034f / 2;
	rightTime = 0;
	
	// Reading left
	ULTRASONIC_PORT->BSRR = (1<<16)|(1<<17)|(1<<18)|(1<<19);//Trig LOW
	DELAY_Us(2);
	ULTRASONIC_PORT->BSRR = 1<<2;//Trig HIGH for left
	DELAY_Us(10);
	ULTRASONIC_PORT->BSRR = 1<<18;//Trig LOW
	
	localCount = 0;
	count = 0;
	state = 0;
	while(count < 10000 && leftTime == 0){
		if((ULTRASONIC_PORT->IDR & 0x0040) == 0x0040 && state == 0){
			state++;
		}
		if((ULTRASONIC_PORT->IDR & 0x0040) == 0x0000 && state == 1){
			leftTime = localCount;
			state--;
			break;
		}
		localCount++;
		DELAY_Us(1);
	}
	leftDist = (float)leftTime * 0.034f / 2;
	leftTime = 0;
	
	// Reading back
	ULTRASONIC_PORT->BSRR = (1<<16)|(1<<17)|(1<<18)|(1<<19);//Trig LOW
	DELAY_Us(2);
	ULTRASONIC_PORT->BSRR = 1<<3;//Trig HIGH for back
	DELAY_Us(10);
	ULTRASONIC_PORT->BSRR = 1<<19;//Trig LOW
	
	localCount = 0;
	count = 0;
	state = 0;
	while(count < 10000 && backTime == 0){
		if((ULTRASONIC_PORT->IDR & 0x0080) == 0x0080 && state == 0){
			state++;
		}
		if((ULTRASONIC_PORT->IDR & 0x0080) == 0x0000 && state == 1){
			backTime = localCount;
			state--;
			break;
		}
		localCount++;
		DELAY_Us(1);
	}
	backDist = (float)backTime * 0.034f / 2;
	backTime = 0;
	
	
	
}
int main(void){
	SystemClock_Config();
	DELAY_Init();
	GPIO_Init();
	while(1){
		UltrasonicRead();
		if(frontDist < 20 && frontDist != 0){
			ULTRASONIC_PORT->BSRR = 1 <<(12);
		}
		else{
			ULTRASONIC_PORT->BSRR = 1 <<(12 + 16);
		}
		DELAY_Ms(1);
	}
}
