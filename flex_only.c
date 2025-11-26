#include "stm32f4xx.h"

/* Function to configure PA1 and ADC1 using registers */
void ADC_Config_BareMetal(void);

volatile uint32_t adc_result = 0;

int main(void)
{
    /* 1. Register Level Initialization */
    ADC_Config_BareMetal();

    /* 2. Start the ADC Conversion */
    /* Access ADC1 Control Register 2 (CR2) and set SWSTART bit (Bit 30) */
    ADC1->CR2 |= (1 << 30);

    while (1)
    {
        /* 3. Wait for End Of Conversion (EOC) flag */
        /* Check Status Register (SR), Bit 1 (EOC) */
        if (ADC1->SR & (1 << 1)) 
        {
            /* 4. Read the Data Register (DR) */
            /* Reading DR automatically clears the EOC flag */
            adc_result = ADC1->DR;
        }
    }
}

void ADC_Config_BareMetal(void)
{
    /* --- STEP 1: Enable Clocks (RCC) --- */
    
    /* Enable GPIOA Clock */
    /* RCC->AHB1ENR: Bit 0 is GPIOAEN */
    RCC->AHB1ENR |= (1 << 0); 
    
    /* Enable ADC1 Clock */
    /* RCC->APB2ENR: Bit 8 is ADC1EN */
    RCC->APB2ENR |= (1 << 8); 

    /* --- STEP 2: Configure GPIO PA1 as Analog --- */
    
    /* PA1 is on GPIOA. We need to set Mode Register (MODER).
       Each pin has 2 bits. Pin 1 uses bits [3:2].
       00 = Input, 01 = Output, 10 = Alternate Function, 11 = Analog.
       We want Analog (11).
    */
    GPIOA->MODER |= (3 << (1 * 2)); // Set bits 2 and 3 to '1'

    /* --- STEP 3: Configure ADC Common Registers --- */
    
    /* ADC Clock Prescaler (in CCR register). 
       The ADC clock must not exceed 36MHz. APB2 is 84MHz default.
       We divide by 4 to get safe 21MHz.
       Bits [17:16] in ADC->CCR: 01 = PCLK2 divided by 4.
    */
    ADC->CCR |= (1 << 16);

    /* --- STEP 4: Configure ADC1 Specifics --- */

    /* Resolution: Default is 12-bit (Bits 24:25 in CR1 are 00), so no change needed. */
    
    /* Continuous Mode: We want it to keep measuring forever.
       Set CONT bit (Bit 1) in Control Register 2 (CR2). 
    */
    ADC1->CR2 |= (1 << 1); 

    /* Sequence: Tell ADC1 to read Channel 1 (PA1) as the 1st conversion.
       SQR3 register controls the 1st conversion in the sequence (SQ1).
       SQ1 bits are [4:0]. We write '1' (for Channel 1) there.
    */
    ADC1->SQR3 = 1; // Set 1st conversion to Channel 1

    /* Enable ADC1 Peripheral */
    /* Set ADON bit (Bit 0) in CR2 */
    ADC1->CR2 |= (1 << 0);
    
    /* Optional: Tiny delay to let ADC stabilize after power on */
    for(int i = 0; i < 1000; i++); 
}