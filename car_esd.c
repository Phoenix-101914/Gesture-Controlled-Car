/**
 * STM32F4 Discovery - FINAL WIFI RC CAR (With Obstacle Avoidance)
 * * Hardware Connections:
 * - L298N IN1 -> PD12 (Left Motor +)
 * - L298N IN2 -> PD13 (Left Motor -)
 * - L298N IN3 -> PD14 (Right Motor +)
 * - L298N IN4 -> PD15 (Right Motor -)
 * - ESP-01 RX -> PA2
 * - ESP-01 TX -> PA3
 * * NEW SENSORS (HC-SR04):
 * - Left Trig -> PB10, Left Echo -> PB11
 * - Right Trig-> PB12, Right Echo-> PB13
 */

#include "stm32f4xx.h"

// --- CONSTANTS ---
#define PWM_SPEED_FAST  2500  
#define PWM_SPEED_SLOW  1500  
#define SAFE_DISTANCE   10    // cm

// --- GLOBAL FLAGS ---
volatile int system_ready = 0; 
volatile int safety_lock = 0; // 1 = Obstacle Detected

// --- FUNCTION PROTOTYPES ---
void SystemClock_Config(void);
void GPIO_Config(void);
void USART_Config(void);
void PWM_Config(void); 
void Timer2_Microsecond_Config(void); // New: For measuring Echo
void USART_SendString(char* str);
void Set_Motor_Speed(int m1_a, int m1_b, int m2_a, int m2_b);
uint32_t Get_Distance(int sensor_id); // New: Read Sensor
void Delay_1Second(void);
void Delay_Short(void);
void Delay_US(uint32_t us);

int main(void)
{
    int i;
    uint32_t dist_left, dist_right;
    
    // 1. Initialization
    SystemClock_Config();
    GPIO_Config();
    USART_Config();
    PWM_Config(); 
    Timer2_Microsecond_Config();
    
    // Ensure motors are OFF
    Set_Motor_Speed(0, 0, 0, 0);
    
    // 2. ESP-01 Setup Sequence 
    USART_SendString("AT+RST\r\n");
    for(i=0; i<3; i++) Delay_1Second(); 
    USART_SendString("AT+CWMODE=2\r\n");
    Delay_1Second();
    for(i = 0; i < 4; i++) {
        USART_SendString("AT+CIPMUX=1\r\n");
        Delay_Short();
        USART_SendString("AT+CIPSERVER=1,8080\r\n");
        Delay_1Second(); 
    }

    // 3. UNLOCK CONTROLS
    system_ready = 1;

    // 4. MAIN LOOP - OBSTACLE AVOIDANCE
    while(1)
    {
        // Check sensors every 100ms
        dist_left = Get_Distance(1);
        Delay_US(5000); // Short gap between pings
        dist_right = Get_Distance(2);
        
        // Safety Logic
        if (dist_left < SAFE_DISTANCE || dist_right < SAFE_DISTANCE)
        {
            // OBSTACLE DETECTED!
            if (safety_lock == 0) {
                // If we were just moving, STOP immediately
                Set_Motor_Speed(0, 0, 0, 0);
                safety_lock = 1; // Engage Lock
            }
            
            // Visual Warning: Turn on All LEDs (Red/Blue/Green/Orange)
            // (Note: This might conflict with Motor PWM LEDs, but shows panic)
        }
        else 
        {
            // PATH CLEAR
            safety_lock = 0; 
        }
        
        Delay_Short(); // Loop delay
    }
}

// --- LOGIC CENTER ---
void USART2_IRQHandler(void)
{
    if (USART2->SR & USART_SR_RXNE)
    {
        char c = USART2->DR;
        
        if (system_ready == 0) return;
        
        // MOTOR CONTROL LOGIC
        if (c == 'w' || c == 'W') {
            // Forward - ONLY ALLOWED IF SAFE
            if (safety_lock == 1) {
                // Obstacle! Ignore 'w', force stop.
                Set_Motor_Speed(0, 0, 0, 0);
            } else {
                Set_Motor_Speed(PWM_SPEED_FAST, 0, PWM_SPEED_FAST, 0);
            }
        }
        else if (c == 's' || c == 'S') {
            // Backward - Always Allowed (To back away from obstacle)
            Set_Motor_Speed(0, PWM_SPEED_FAST, 0, PWM_SPEED_FAST);
        }
        else if (c == 'a' || c == 'A') {
            // Left Spin
             if (safety_lock == 1) Set_Motor_Speed(0, 0, 0, 0); // Don't spin into wall
             else Set_Motor_Speed(0, PWM_SPEED_SLOW, PWM_SPEED_SLOW, 0);
        }
        else if (c == 'd' || c == 'D') {
            // Right Spin
            if (safety_lock == 1) Set_Motor_Speed(0, 0, 0, 0);
            else Set_Motor_Speed(PWM_SPEED_SLOW, 0, 0, PWM_SPEED_SLOW);
        }
        else if (c == 'q' || c == 'Q') {
            // Stop
            Set_Motor_Speed(0, 0, 0, 0);
        }
    }
}

// --- SENSOR FUNCTIONS ---

uint32_t Get_Distance(int sensor_id)
{
    uint32_t pTrig, pEcho;
    uint32_t echo_width = 0;
    uint32_t timeout = 100000; // Timeout to prevent hanging
    
    if(sensor_id == 1) { pTrig = 10; pEcho = 11; } // PB10/11
    else               { pTrig = 12; pEcho = 13; } // PB12/13
    
    // 1. Send Trigger Pulse (10us)
    GPIOB->BSRR = (1 << pTrig);    // Trig High
    Delay_US(10);
    GPIOB->BSRR = (1 << (pTrig + 16)); // Trig Low (Reset)
    
    // 2. Wait for Echo to go High
    while (!(GPIOB->IDR & (1 << pEcho))) {
        timeout--;
        if (timeout == 0) return 999; // Error/No connection
    }
    
    // 3. Measure Echo High Time
    // Reset Timer 2
    TIM2->CNT = 0;
    timeout = 100000;
    
    while (GPIOB->IDR & (1 << pEcho)) {
        timeout--;
        if (timeout == 0) break; 
    }
    
    echo_width = TIM2->CNT; // Time in microseconds
    
    // 4. Calculate Distance: Distance = (Time * 0.034) / 2 ~= Time / 58
    return (echo_width / 58);
}

void Timer2_Microsecond_Config(void)
{
    // Enable TIM2 Clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    
    // Prescaler: 168MHz / 2 (APB1) = 84MHz Timer Clock
    // We want 1MHz counting (1us precision)
    // 84,000,000 / 84 = 1,000,000
    TIM2->PSC = 83; // 84-1
    TIM2->ARR = 0xFFFFFFFF; // Max Count
    TIM2->CR1 |= TIM_CR1_CEN;
}

void Delay_US(uint32_t us)
{
    TIM2->CNT = 0;
    while(TIM2->CNT < us);
}

// --- DRIVER FUNCTIONS ---

void Set_Motor_Speed(int m1_a, int m1_b, int m2_a, int m2_b)
{
    TIM4->CCR1 = m1_a;
    TIM4->CCR2 = m1_b;
    TIM4->CCR3 = m2_a;
    TIM4->CCR4 = m2_b;
}

void PWM_Config(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    TIM4->PSC = 83;
    TIM4->ARR = 2499; 
    TIM4->CCMR1 |= (6 << 4) | (1 << 3) | (6 << 12) | (1 << 11);
    TIM4->CCMR2 |= (6 << 4) | (1 << 3) | (6 << 12) | (1 << 11);
    TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM4->CR1 |= TIM_CR1_CEN;
}

void GPIO_Config(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOBEN;
    
    // PD12-PD15 as AF2 (TIM4)
    GPIOD->MODER &= ~(0xFF000000); 
    GPIOD->MODER |=  (0xAA000000); 
    GPIOD->AFR[1] &= ~(0xFFFFFFFF); 
    GPIOD->AFR[1] |=  (0x22220000); 

    // PA2, PA3 for UART
    GPIOA->MODER &= ~((3 << 4) | (3 << 6)); 
    GPIOA->MODER |=  ((2 << 4) | (2 << 6)); 
    GPIOA->AFR[0] |= (0x7 << 8) | (0x7 << 12);
    
    // PB10, PB12 (Trig) -> Output (01)
    // PB11, PB13 (Echo) -> Input  (00)
    GPIOB->MODER &= ~( (3<<(10*2)) | (3<<(11*2)) | (3<<(12*2)) | (3<<(13*2)) );
    GPIOB->MODER |=  ( (1<<(10*2)) | (1<<(12*2)) );
    // Pull-down for Echo pins helps stability
    GPIOB->PUPDR |=  ( (2<<(11*2)) | (2<<(13*2)) );
}

void USART_Config(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    USART2->BRR = 0x16D; 
    USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    NVIC_EnableIRQ(USART2_IRQn);
}

void USART_SendString(char* str) {
    while (*str) {
        while (!(USART2->SR & USART_SR_TXE)); 
        USART2->DR = *str;
        str++;
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
    for(i = 0; i < 42000000; i++); 
}

void Delay_Short(void) {
    volatile uint32_t i;
    for(i = 0; i < 5000000; i++); 
}