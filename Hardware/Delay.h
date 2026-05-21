#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f4xx.h"

void systick_init(void);
void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
void Delay_s(uint32_t s);

// 获取当前系统运行毫秒数
uint32_t delay_ms_count_get(void);

// 获取当前系统运行微秒数（基于 DWT Cycle Counter）
uint32_t delay_us_count_get(void);

// 检查是否到达指定时间间隔
uint8_t delay_ms_count_check(uint32_t last_time, uint32_t interval);

// 更新时间计数（在 SysTick 中断中调用）
void delay_tick_update(void);

#endif
