/**
 * STM32F4 Discovery - CAR RECEIVER (8-Direction + Stop)
 * * Purpose: Controls Motors using digits 0-8 via WiFi.
 * * Features: Speed Control (ENA/ENB) + Smooth Input Filtering.
 * *
 * * [PROTOCOL MAPPING]
 * * 0: STOP
 * * 1: FORWARD          | 5: FWD-LEFT (Curve)
 * * 2: BACKWARD         | 6: FWD-RIGHT (Curve)
 * * 3: LEFT (Spin)      | 7: BACK-LEFT (Curve)
 * * 4: RIGHT (Spin)     | 8: BACK-RIGHT (Curve)
 * *
 * * [HARDWARE WIRING]
 * * LEFT SPEED (ENA):  PE9  (TIM1 CH1)
 * * RIGHT SPEED (ENB): PE11 (TIM1 CH2)
 * * LEFT DIR (IN1/2):  PD12, PD13
 * * RIGHT DIR (IN3/4): PD14, PD15
 * * ESP-01 RX -> PA2  | TX  -> PA3  | CH_PD -> 3.3V
 */

#include "stm32f4xx.h"

// --- CONSTANTS ---
#define SPD_MAX    2500  // 100% Speed
#define SPD_TURN   1800  // 70% (For Spin)
#define SPD_CURVE  1200  // 50% (For Inner Wheel on Curves)

// --- GLOBALS ---
volatile int setup_done = 0; 

// --- PROTOTYPES ---
void SystemClock_Config(void);
void GPIO_Config(void);
void USART_Config(void);
void PWM_Config(void); 
void USART_SendString(char* str);
void Set_Motor_State(int left_speed, int right_speed);
void Delay_1Second(void);
void Delay_Short(void);

int main(void)
{
    int i;
    
    // 1. Hardware Init
    SystemClock_Config();
    GPIO_Config();
    USART_Config();
    PWM_Config(); 
    
    // --- DIAGNOSTIC CHIRP ---
    // If motors move -> Wiring Good.
    GPIOD->BSRR = (1 << 15); // Blue LED ON
    Set_Motor_State(1000, 1000); 
    Delay_Short();
    Set_Motor_State(0, 0);       
    GPIOD->BSRR = (1 << 31); // Blue LED OFF
    
    // 2. ESP-01 Setup Sequence
    USART_SendString("AT+RST\r\n");
    for(i=0; i<3; i++) Delay_1Second(); 
    
    USART_SendString("AT+CWMODE=2\r\n");
    Delay_1Second();
    
    USART_SendString("AT+CWSAP=\"RCCarNet\",\"12345678\",5,3\r\n");
    Delay_1Second();
    Delay_1Second(); 
    
    USART_SendString("AT+CIPMUX=1\r\n");
    Delay_1Second();

    // Start Server
    for(i = 0; i < 3; i++) {
        USART_SendString("AT+CIPSERVER=1,8080\r\n");
        Delay_1Second(); 
    }

    // Setup Complete
    setup_done = 1;
    GPIOD->BSRR = (1 << 13); // Orange LED ON (Ready)

    while(1)
    {
        // Main loop empty
    }
}

// --- INTERRUPT HANDLER ---
void USART2_IRQHandler(void)
{
    if (USART2->SR & USART_SR_RXNE)
    {
        char c = USART2->DR;
        
        if (setup_done == 0) return;
        
        uint8_t cmd = 255; // Default to invalid
        
        // --- INPUT FILTERING ---
        // 1. Handle ASCII digits '0'-'8'
        if (c >= '0' && c <= '8') {
            cmd = c - '0';
        }
        // 2. Handle Raw Bytes 0-8 (for microcontroller senders)
        else if (c <= 8) {
            cmd = c;
        }
        
        // Only execute if valid command (0-8).
        // This ignores '\n', '\r', spaces, etc., preventing stutter.
        if (cmd <= 8) 
        {
            switch (cmd)
            {
                case 0: // STOP
                    Set_Motor_State(0, 0);
                    break;

                case 1: // FORWARD
                    Set_Motor_State(SPD_MAX, SPD_MAX);
                    break;

                case 2: // BACKWARD
                    Set_Motor_State(-SPD_MAX, -SPD_MAX);
                    break;

                case 3: // right (Spin)
                    Set_Motor_State(-SPD_TURN, SPD_TURN);
                    break;

                case 4: // left (Spin)
                    Set_Motor_State(SPD_TURN, -SPD_TURN);
                    break;

                case 5: // FWD-right (Curve)
                    // Left wheel slow, Right wheel fast
                    Set_Motor_State(SPD_CURVE, SPD_MAX);
                    break;

                case 6: // FWD-left (Curve)
                    // Left wheel fast, Right wheel slow
                    Set_Motor_State(SPD_MAX, SPD_CURVE);
                    break;

                case 7: // BACK-right (Curve)
                    Set_Motor_State(-SPD_CURVE, -SPD_MAX);
                    break;

                case 8: // BACK-left (Curve)
                    Set_Motor_State(-SPD_MAX, -SPD_CURVE);
                    break;
            }
        }
    }
}

// --- HELPER: Motor Control (ENA/ENB Logic) ---
void Set_Motor_State(int left_speed, int right_speed)
{
    // LEFT MOTOR (PD12/13 + PE9)
    if (left_speed > 0) {
        GPIOD->BSRR = (1 << (12+16)); // IN1 Low (Forward Config)
        GPIOD->BSRR = (1 << 13);      // IN2 High
        TIM1->CCR1 = left_speed;      
    } else if (left_speed < 0) {
        GPIOD->BSRR = (1 << 12);      // IN1 High (Back Config)
        GPIOD->BSRR = (1 << (13+16)); // IN2 Low
        TIM1->CCR1 = -left_speed;     
    } else {
        GPIOD->BSRR = (1 << (12+16)) | (1 << (13+16)); // Stop
        TIM1->CCR1 = 0;
    }

    // RIGHT MOTOR (PD14/15 + PE11)
    if (right_speed > 0) {
        GPIOD->BSRR = (1 << (14+16)); // IN3 Low (Forward Config)
        GPIOD->BSRR = (1 << 15);      // IN4 High
        TIM1->CCR2 = right_speed;     
    } else if (right_speed < 0) {
        GPIOD->BSRR = (1 << 14);      // IN3 High (Back Config)
        GPIOD->BSRR = (1 << (15+16)); // IN4 Low
        TIM1->CCR2 = -right_speed;
    } else {
        GPIOD->BSRR = (1 << (14+16)) | (1 << (15+16)); // Stop
        TIM1->CCR2 = 0;
    }
}

// --- DRIVERS ---

void PWM_Config(void)
{
    // Enable TIM1 (APB2) for PE9/PE11
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    TIM1->PSC = 167; 
    TIM1->ARR = 2499; 
    TIM1->CCMR1 |= (6 << 4) | (1 << 3) | (6 << 12) | (1 << 11);
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CR1 |= TIM_CR1_CEN;
}

void GPIO_Config(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN;
    
    // Direction Pins (PD12-15) -> Output
    GPIOD->MODER &= ~(0xFF000000); GPIOD->MODER |= (0x55000000); 
    
    // Speed Pins (PE9, PE11) -> AF1
    GPIOE->MODER &= ~( (3<<(9*2)) | (3<<(11*2)) );
    GPIOE->MODER |=  ( (2<<(9*2)) | (2<<(11*2)) );
    GPIOE->AFR[1] |= (1 << 4);  // PE9
    GPIOE->AFR[1] |= (1 << 12); // PE11

    // UART (PA2, PA3) -> AF7
    GPIOA->MODER &= ~((3 << 4) | (3 << 6)); 
    GPIOA->MODER |=  ((2 << 4) | (2 << 6)); 
    GPIOA->AFR[0] |= (0x7 << 8) | (0x7 << 12);
}

void USART_Config(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    USART2->BRR = 0x16D; 
    USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    NVIC_EnableIRQ(USART2_IRQn);
}

void USART_SendString(char* str) {
    while (*str) {
        while (!(USART2->SR & USART_SR_TXE)); 
        USART2->DR = *str++;
    }
}

void SystemClock_Config(void) {
    RCC->CR |= RCC_CR_HSEON;
    while(!(RCC->CR & RCC_CR_HSERDY));
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;
    RCC->PLLCFGR = (8 << 0) | (336 << 6) | (0 << 16) | (7 << 24) | RCC_PLLCFGR_PLLSRC_HSE;
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY));
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

void Delay_1Second(void) {
    volatile uint32_t i;
    for(i = 0; i < 42000000/5; i++); 
}

void Delay_Short(void) {
    volatile uint32_t i;
    for(i = 0; i < 4000000; i++); 
}
