#include "stm32g070xx.h"

// Define the pin number for the IR sensor, PIR sensor, and buzzer (PA5, PA7, PA8)
#define IR_PIN 5     // PA5 for IR sensor
#define PIR_PIN 7    // PA7 for PIR sensor
#define BUZZER_PIN 8 // PA8 for Buzzer
static void delay(uint32_t ms)
{
	SysTick->CTRL |=(1<<0)|(1<<2);
	SysTick->LOAD =(SystemCoreClock/1000)-1;
	for(uint32_t i=0;i<ms;i++)
	{
while (!((SysTick->CTRL)&(1<<16)));
	}
SysTick->CTRL&=~(1<<0);
}

// Function prototypes
void gpio_init(void);
void delay_ms(int ms);

int main(void) {
    gpio_init();  // Initialize GPIO

    while (1) {
        // Read the status of the IR sensor (PA5)
        int ir_status = (GPIOA->IDR & (1 << 5)) ? 1 : 0;

        // If the IR sensor detects movement, activate the PIR sensor (PA7)
        if (ir_status == 1) {
            // Read the status of the PIR sensor (PA7)
            int pir_status = (GPIOA->IDR & (1 << 7)) ? 1 : 0;

            // If the PIR sensor detects movement, turn on the buzzer (PA8)
            if (pir_status == 1) {
                GPIOA->ODR |= (1 << 8);  // Turn buzzer ON
            } else {
                GPIOA->ODR &= ~(1 << 8); // Turn buzzer OFF
            }
        } else {
            GPIOA->ODR &= ~(1 << BUZZER_PIN); // Ensure buzzer is OFF if no IR detection
        }

        delay(200);  // Delay for stability
    }
}

// GPIO Initialization Function
void gpio_init(void) {
     // Enable GPIOA clock
    RCC->IOPENR |= (1 << 0);

    // Set PA5 (IR sensor) and PA7 (PIR sensor) as input
    GPIOA->MODER &= (0<<10)|(0<<11);
   GPIOA->MODER &= (0<<14)|(0<<15);// Clear mode for PA7 (input)

    // Set PA8 (Buzzer) as output
    GPIOA->MODER &= ~(3 << (2 * 8));  // Clear mode
    GPIOA->MODER |= (1 << (2 * 8));   // Set PA8 as output

    // Ensure buzzer is off initially
    GPIOA->ODR &= ~(1 << 8);
	
}

// Simple delay function (not highly accurate, uses CPU cycles)
void delay_ms(int ms) {
    for (int i = 0; i < ms * 1000; i++) {
        __NOP();  // No operation, just to burn CPU cycles
    }
}


