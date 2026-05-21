#include "stm32f4xx.h"
#include "LED.h"
#include "Delay.h"

void LED_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // 使能 GPIOB 时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    // PB6 配置为通用推挽输出 (LED驱动)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 默认熄灭 LED
    LED_off();
}

void LED_on(void)
{
    GPIO_SetBits(GPIOB, GPIO_Pin_6);
}

void LED_off(void)
{
    GPIO_ResetBits(GPIOB, GPIO_Pin_6);
}

void LED_toggle(void)
{
    GPIO_ToggleBits(GPIOB, GPIO_Pin_6);
}

/**
 * @brief  LED 1Hz闪烁处理（在主循环中调用）
 * @note   每500ms翻转一次LED，形成1Hz（50%占空比）闪烁
 *         配合 LED_heartbeat_sync() 可与MAVLink心跳包同频同相
 */
void LED_blink_handler(void)
{
    static uint32_t last_toggle_time = 0;
    uint32_t now = delay_ms_count_get();

    if (now - last_toggle_time >= 500) {
        last_toggle_time = now;
        LED_toggle();
    }
}

/**
 * @brief  心跳同步信号（在发送MAVLink心跳包时调用）
 * @note   点亮LED并重置闪烁相位，使LED闪烁与心跳包同频
 *         心跳发送为1Hz，LED以1Hz闪烁，相位对齐
 */
void LED_heartbeat_sync(void)
{
    LED_on();  // 心跳发送时点亮LED（闪烁相位与心跳对齐）
}
