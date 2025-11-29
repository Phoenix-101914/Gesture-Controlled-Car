#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>

// --- TARGET CAR CREDENTIALS ---
#define TARGET_SSID "RCCarNet"
#define TARGET_PASS "12345678"
#define TARGET_IP   "192.168.4.1" // Default ESP-01 AP IP
#define TARGET_PORT "8080"

// --- PROTOTYPES ---
void SystemClock_Config(void);
void USART_Config(void);
void GPIO_LED_Config(void);
void USART_SendString(char* str);
void Delay_ms(volatile uint32_t ms);
void ESP_SendCommand(char* cmd, int delay_after);

int main(void)
{
    char buffer[100];
    
    // 1. Hardware Initialization
    SystemClock_Config();
    GPIO_LED_Config(); // Initialize Green LED (PD12)
    USART_Config();

    // 2. Initial Stability Delay (Let ESP power up)
    Delay_ms(2000);

    // 3. ESP-01 Setup Sequence
    ESP_SendCommand("AT+RST\r\n", 3000);
    ESP_SendCommand("AT+CWMODE=1\r\n", 1000); // Station Mode (Client)

    // 4. Connect to the Car's Wi-Fi
    // This fills the buffer with: AT+CWJAP="RCCarNet","12345678"
    sprintf(buffer, "AT+CWJAP=\"%s\",\"%s\"\r\n", TARGET_SSID, TARGET_PASS);
    ESP_SendCommand(buffer, 8000); // Wait 8s for connection

    // --- VISUAL FEEDBACK: CONNECTED ---
    // Turn ON Green LED (PD12) to indicate Wi-Fi Connection Success
    GPIOD->ODR |= (1 << 12); 
    
    // 5. Connect to the Car's Server Socket (TCP)
    // Connecting to IP 192.168.4.1 Port 8080
    sprintf(buffer, "AT+CIPSTART=\"TCP\",\"%s\",%s\r\n", TARGET_IP, TARGET_PORT);
    ESP_SendCommand(buffer, 2000);
    
    // --- START MISSION ---

    // STEP A: Send '1' (Forward)
    // 1. Prepare to send 1 byte
    ESP_SendCommand("AT+CIPSEND=1\r\n", 100); 
    // 2. Send the command
    ESP_SendCommand("1", 0); 
    
    // STEP B: Drive for 3 Seconds
    Delay_ms(3000);

    // STEP C: Send '0' (Stop)
    ESP_SendCommand("AT+CIPSEND=1\r\n", 100);
    ESP_SendCommand("0", 0);

    // Mission Complete.
    // The Green LED stays ON to show the board is still powered and linked.
    while(1)
    {
        // Infinite loop
    }
}

// --- HELPER FUNCTIONS ---

// Function to send AT command and wait blindly
void ESP_SendCommand(char* cmd, int delay_after)
{
    USART_SendString(cmd);
    if(delay_after > 0) Delay_ms(delay_after);
}

// Configure PD12 as Output for LED
void GPIO_LED_Config(void)
{
    // Enable GPIOD Clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

    // Set PD12 to Output Mode (01)
    // Bits [25:24] control Pin 12
    GPIOD->MODER &= ~(3U << 24); // Clear
    GPIOD->MODER |=  (1U << 24); // Set bit 24
}

// Configure UART2 (PA2/PA3) 
void USART_Config(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // PA2, PA3 -> AF Mode
    GPIOA->MODER &= ~((3 << 4) | (3 << 6)); 
    GPIOA->MODER |=  ((2 << 4) | (2 << 6)); 
    GPIOA->AFR[0] |= (0x7 << 8) | (0x7 << 12);

    // Baud Rate 115200 @ 42MHz APB1
    USART2->BRR = 0x16D; 
    USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

void USART_SendString(char* str) {
    while (*str) {
        while (!(USART2->SR & USART_SR_TXE)); 
        USART2->DR = *str++;
    }
}

// Approximate delay for 168MHz Clock
void Delay_ms(volatile uint32_t ms) {
    for(uint32_t i = 0; i < ms; i++) {
        for(volatile uint32_t j = 0; j < 35000; j++) { __NOP(); }
    }
}

// 168MHz System Clock Configuration
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
