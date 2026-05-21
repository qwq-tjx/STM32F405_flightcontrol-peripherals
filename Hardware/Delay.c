#include "Delay.h"

volatile uint32_t current_time_ms = 0;

// ========== DWT Cycle Counter ==========
// DWT 运行在 CPU 频率，DWT_CYCCNT 每个周期递增一次
#define DWT_CYCLES_PER_US  (SystemCoreClock / 1000000)  // 168MHz / 1MHz = 168

// DWT 寄存器地址
#define DWT_CTRL    (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT  (*(volatile uint32_t *)0xE0001004)
#define DWT_LAR     (*(volatile uint32_t *)0xE0001FB0)
#define DWT_HAS_CYCCNT  0x01

void dwt_init(void)
{
    // 使能 TRCENA (在 CoreDebug 中)
    *(volatile uint32_t *)0xE000EDFC |= 0x01000000;
    
    // 解锁 DWT (必须！)
    DWT_LAR = 0xC5ACCE55;
    
    // 清零并使能 Cycle Counter
    DWT_CYCCNT = 0;
    DWT_CTRL |= DWT_HAS_CYCCNT;
}

void Delay_us(uint32_t xus)
{
    uint32_t start = DWT_CYCCNT;                              // 记录开始时刻
    uint32_t cycles = DWT_CYCLES_PER_US * xus;                // 计算所需周期数
    
    // 处理计数器溢出情况
    while (DWT_CYCCNT - start < cycles);
}

void Delay_ms(uint32_t xms)
{
    while(xms--)
    {
        Delay_us(1000);
    }
}

void Delay_s(uint32_t xs)
{
    while(xs--)
    {
        Delay_ms(1000);
    }
}

uint32_t delay_ms_count_get(void)
{
    return current_time_ms;
}

uint8_t delay_ms_count_check(uint32_t last_time, uint32_t interval)
{
    return (delay_ms_count_get() - last_time) >= interval;
}

uint32_t delay_us_count_get(void)
{
    // 返回当前微秒数
    return DWT_CYCCNT / DWT_CYCLES_PER_US;
}

void delay_tick_update(void)
{
    current_time_ms++;
}

void systick_init(void)
{
    // 初始化 DWT Cycle Counter (必须在 SysTick 之前)
    dwt_init();
    
    // 中断优先级分组
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    // 配置 SysTick 每 1ms 中断一次
    if (SysTick_Config(SystemCoreClock / 1000)) {
        while(1);
    }
    
    // SysTick 优先级 = 1
    NVIC_SetPriority(SysTick_IRQn, (0 << 2) | 1);
}
