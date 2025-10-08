#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"
#include "SEGGER_RTT.h"

#define LD2_PORT GPIOA
#define LD2_PIN  5U 

void SysTick_Handler(void) {
    HAL_IncTick();
}

static void gpio_init(void) {
    // Enable GPIOA clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    (void)RCC->AHB2ENR;

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
    SysTick->CTRL = 0U; // stop SysTick when done
}

int main() {
#ifdef HAL
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

#else
    SystemInit();
    SystemCoreClockUpdate();

    gpio_init();

    while (1) {
        LD2_PORT->BSRR = (1U << LD2_PIN);
        delay_ms(500);
        LD2_PORT->BSRR = (1U << (LD2_PIN + 16U));
        delay_ms(500);
    }
#endif
}
