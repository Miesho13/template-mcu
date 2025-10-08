#include "stm32f4xx.h"
#include "SEGGER_RTT.h"

#ifdef HAL
#include "stm32l4xx_hal.h"



void SysTick_Handler(void) {
    HAL_IncTick();
}

int main() {
    HAL_Init();

    SEGGER_RTT_Init();

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    GPIO_InitTypeDef GPIO_Init = {
        .Pin = GPIO_PIN_5,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pull = GPIO_PULLDOWN,
        .Speed = GPIO_SPEED_FREQ_LOW,
        .Alternate = 0,
    };

    HAL_GPIO_Init(GPIOA, &GPIO_Init);
    HAL_Delay(1000);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);

    while (1) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(1000);
    }
}

#else 

#define LD2_PORT GPIOD
#define LD2_PIN  12U 

static void gpio_init(void) {
    // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    (void)RCC->AHB1ENR;

    LD2_PORT->MODER   &= ~(3U << (LD2_PIN * 2));
    LD2_PORT->MODER   |=  (1U << (LD2_PIN * 2));
    LD2_PORT->OTYPER  &= ~(1U << LD2_PIN);
    LD2_PORT->OSPEEDR |=  (3U << (LD2_PIN * 2));
    LD2_PORT->PUPDR   &= ~(3U << (LD2_PIN * 2));
}

static void delay_ms(uint32_t ms) {
    SysTick->LOAD = (SystemCoreClock / 1000U) - 1U; // 1ms tick
    SysTick->VAL  = 0U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    for (uint32_t i = 0; i < ms; i++) {
        while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0U) { /* wait */ }
    }
    SysTick->CTRL = 0U;
}

int main() {
    SystemInit();
    SystemCoreClockUpdate();

    gpio_init();

    while (1) {
        LD2_PORT->BSRR = (1U << LD2_PIN);
        delay_ms(500);
        LD2_PORT->BSRR = (1U << (LD2_PIN + 16U));
        delay_ms(500);
    }
}

#endif
