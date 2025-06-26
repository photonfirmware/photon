#include "bootloader.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx.h"
#include "stm32_def.h"

#define SYSMEM_ADDRESS 0x1FFFEC00


void reboot_into_bootloader() {
    const uint32_t jump_address = *(__IO uint32_t*)(SYSMEM_ADDRESS + 4);

    /* Disable peripherals */
    HAL_RCC_DeInit();
    HAL_DeInit();

    /* Reset SysTick */
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    /* Disable PLL */

    /* Disable interrupts */
    __disable_irq();

    /* HACK
        Workaround issue where RS485 transceiver prevents using the system bootloader via UART.

        This occurs with some variants of the RS485 transceiver we use because we forgot pullup/pulldowns on the ~RE/DE
        pins. They're left floating and sometimes that means the ~RE is enabled which prevents the UART programmer from
        communicating with the STM32.

        This works around this by configuring those pins before jumping to the bootloader and *locking* them, so that
        they stick. This only work when jumping to the bootloader from the firmware, it won't work on a cold boot.
    */
     __HAL_RCC_GPIOA_CLK_ENABLE();
    /* ~RE pin should be pulled high to disable the RS485 receiver. */
    GPIO_InitTypeDef  RE_Init;
    RE_Init.Pin = GPIO_PIN_11;
    RE_Init.Mode = GPIO_MODE_OUTPUT_PP;
    RE_Init.Pull = GPIO_PULLUP;
    RE_Init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &RE_Init);
    HAL_GPIO_WritePin(GPIOA, RE_Init.Pin, GPIO_PIN_SET);
    HAL_GPIO_LockPin(GPIOA, RE_Init.Pin);

    /* DE should be pulled low to disable the RS485 transmitter */
    GPIO_InitTypeDef  DE_Init;
    DE_Init.Pin = GPIO_PIN_12;
    DE_Init.Mode = GPIO_MODE_OUTPUT_PP;
    DE_Init.Pull = GPIO_PULLDOWN;
    DE_Init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &DE_Init);
    HAL_GPIO_WritePin(GPIOA, DE_Init.Pin, GPIO_PIN_RESET);
    HAL_GPIO_LockPin(GPIOA, DE_Init.Pin);

    /* Jump to bootloader */
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
    __set_MSP(*(__IO uint32_t*)SYSMEM_ADDRESS);
    __enable_irq();
    ((void (*)(void)) jump_address)();

    while(1);
}
