
#include "stm32f4xx.h"

// --- BAUD RATE SELECTION ---
// COMMENTED OUT to force 115200 Baud (Default for Factory ESPs)
// #define USE_9600_BAUD  
// ---------------------------

void GPIO_Config(void);
void USART_Config(void);
void USART_SendString(char* str);
void Delay_Long(void); 
void SystemClock_Config(void); 

int main(void)
{
    // 1. Configure the Clock to 168MHz FIRST
    SystemClock_Config();
    SystemCoreClock = 168000000;

    // 2. Init Peripherals
    GPIO_Config();
    USART_Config();
    
    // 3. Turn on Green LED (PD12) to indicate "Starting"
    GPIOD->BSRR = (1 << 12);

    // Wait for ESP to boot up 
    Delay_Long();
    Delay_Long();
    Delay_Long();

    // 4. Reset Module
    USART_SendString("AT+RST\r\n");
    // Give it a LONG time to reset and stabilize at 115200
    Delay_Long(); 
    Delay_Long();
    Delay_Long();
    Delay_Long(); 

    // 5. Spam the Mode Command (Sometimes the first one is missed)
    USART_SendString("AT+CWMODE=2\r\n");
    Delay_Long();
    USART_SendString("AT+CWMODE=2\r\n");
    Delay_Long();

    // 6. Create Network (SSID: RCCarNet, Pass: 12345678)
    USART_SendString("AT+CWSAP=\"RCCarNet\",\"12345678\",5,3\r\n");
    Delay_Long();
    Delay_Long(); 
    Delay_Long(); // Extra wait for WiFi radio to restart with new name

    // 7. Enable Multiple Connections
    USART_SendString("AT+CIPMUX=1\r\n");
    Delay_Long();

    // 8. Start Server on Port 8080
    USART_SendString("AT+CIPSERVER=1,8080\r\n");
    Delay_Long();

    // 9. CONFIG DONE: Blink Orange LED (PD13)
    while(1)
    {
        GPIOD->ODR ^= (1 << 13); // Toggle Orange LED
        Delay_Long();
    }
}

// --- CLOCK CONFIGURATION ---
void SystemClock_Config(void)
{
    // Enable HSE (8MHz Crystal)
    RCC->CR |= RCC_CR_HSEON;
    while(!(RCC->CR & RCC_CR_HSERDY));

    // Flash Config
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;

    // Configure PLL for 168MHz
    RCC->PLLCFGR = (8 << 0) | (336 << 6) | (0 << 16) | (7 << 24) | RCC_PLLCFGR_PLLSRC_HSE;

    // Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY));

    // Set Bus Prescalers: APB1 = Div4 (42MHz), APB2 = Div2 (84MHz)
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;

    // Switch System Clock to PLL
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

// --- LOW LEVEL FUNCTIONS ---

void USART_SendString(char* str)
{
    while (*str)
    {
        while (!(USART2->SR & USART_SR_TXE)); 
        USART2->DR = *str;
        str++;
    }
}

void GPIO_Config(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIODEN;

    // LEDs (PD12, PD13)
    GPIOD->MODER |= (1 << 24) | (1 << 26); 

    // PA2 (TX), PA3 (RX)
    GPIOA->MODER &= ~((3 << 4) | (3 << 6)); 
    GPIOA->MODER |=  ((2 << 4) | (2 << 6)); 
    GPIOA->AFR[0] |= (0x7 << 8) | (0x7 << 12);
}

void USART_Config(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    
    // Baud Rate Calculation for 42MHz (APB1)
#ifdef USE_9600_BAUD
    // 9600 Baud
    USART2->BRR = 0x1117;
#else
    // 115200 Baud (Default for most ESP-01s)
    // 42,000,000 / (16 * 115200) = 22.786 -> 0x16D
    USART2->BRR = 0x16D; 
#endif
    
    USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

void Delay_Long(void)
{
    volatile uint32_t i;
    for(i = 0; i < 2000000; i++); 
}