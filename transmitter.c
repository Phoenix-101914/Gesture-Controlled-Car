#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>

// --- CONFIGURATION ---
#define TARGET_SSID "RCCarNet"
#define TARGET_PASS "12345678"
#define TARGET_IP   "192.168.4.1" 
#define TARGET_PORT "8080"

// --- DELAY FUNCTION (Approx for 168MHz) ---
void Delay_ms(volatile uint32_t ms) {
    for(uint32_t i = 0; i < ms; i++) {
        for(volatile uint32_t j = 0; j < 35000; j++) { __NOP(); }
    }
}

// --- UART SEND FUNCTION ---
void UART_Send(char* str) {
    while (*str) {
        while (!(USART2->SR & USART_SR_TXE)); // Wait for TX Buffer Empty
        USART2->DR = *str++;                  // Send Char
    }
}

// --- CLOCK SETUP (168MHz) ---
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

// --- MAIN FUNCTION ---
int main(void) {
    char buffer[100]; // Temporary storage for formatting commands

    // 1. Hardware Setup
    SystemClock_Config();

    // Enable GPIOA (UART), GPIOD (LED), USART2
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // Configure PA2(TX) & PA3(RX) as Alternate Function 7
    GPIOA->MODER &= ~((3U << 4) | (3U << 6)); 
    GPIOA->MODER |=  ((2U << 4) | (2U << 6)); 
    GPIOA->AFR[0] |= (0x7 << 8) | (0x7 << 12);

    // Configure PD12 (Green LED) as Output
    GPIOD->MODER &= ~(3U << 24);
    GPIOD->MODER |=  (1U << 24);

    // Configure UART Baud Rate (115200 @ 42MHz APB1)
    USART2->BRR = 0x16D; 
    USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

    // 2. Wait for ESP-01 to Power Up
    Delay_ms(2000);

    // 3. Reset & Set Station Mode
    UART_Send("AT+RST\r\n");
    Delay_ms(3000);
    UART_Send("AT+CWMODE=1\r\n");
    Delay_ms(1000);

    // 4. Connect to Car Wi-Fi
    sprintf(buffer, "AT+CWJAP=\"%s\",\"%s\"\r\n", TARGET_SSID, TARGET_PASS);
    UART_Send(buffer);
    Delay_ms(8000); // Wait 8s for Wi-Fi connection

    // --- INDICATE CONNECTION SUCCESS ---
    GPIOD->ODR |= (1 << 12); // Green LED ON

    // 5. Connect to Server (TCP)
    sprintf(buffer, "AT+CIPSTART=\"TCP\",\"%s\",%s\r\n", TARGET_IP, TARGET_PORT);
    UART_Send(buffer);
    Delay_ms(2000);

    // ==========================================
    //       ACTUAL CONTROL LOGIC STARTS HERE
    // ==========================================

    // --- STEP A: SEND '1' (Forward) ---
    // We send the length first (1 byte), then the data.
    UART_Send("AT+CIPSEND=1\r\n"); 
    Delay_ms(100); 
    UART_Send("1"); // The character '1' triggers Forward on Receiver
    
    // --- STEP B: WAIT 3 SECONDS ---
    // The car will keep moving forward during this delay
    Delay_ms(3000); 

    // --- STEP C: SEND '0' (Stop) ---
    UART_Send("AT+CIPSEND=1\r\n");
    Delay_ms(100);
    UART_Send("0"); // The character '0' triggers Stop on Receiver

    // ==========================================
    //             LOGIC ENDS
    // ==========================================

    while(1) {
        // Mission complete. LED stays ON.
    }
}
