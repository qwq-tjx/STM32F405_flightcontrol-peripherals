#ifndef __DSHOT_H
#define __DSHOT_H

#include "stm32f4xx.h"

// ==================== 临界区保护宏 ====================
// 保存/恢复 PRIMASK，防止错误地在中断已关闭的上下文中重新开中断
// 注意: 不可嵌套使用（如需嵌套请使用 _unsafe 版本函数）
extern uint32_t __dshot_crit_primask;
#define DSHOT_ENTER_CRITICAL()  do { __dshot_crit_primask = __get_PRIMASK(); __disable_irq(); } while(0)
#define DSHOT_EXIT_CRITICAL()   do { if (!__dshot_crit_primask) __enable_irq(); } while(0)

// 缓冲区长度
#define ESC_CMD_BUFFER_LEN     18    // 16数据 + 2位纯低电平

// DSHOT时序参数 (DSHOT15, PSC=19 → 66.67µs/bit)
#define DSHOT_BIT0             208   // 52*4
#define DSHOT_BIT1             420   // 105*4

// ==================== 外部接口函数 ====================

// 初始化DSHOT模块 (TIM8和DMA配置)
void DShot_Init(void);

// 设置单个通道的油门值
void DShot_SetThrottle(uint8_t channel, uint16_t throttle);


// 设置所有四个通道的油门值
void DShot_SetAllThrottles(uint16_t t1, uint16_t t2, uint16_t t3, uint16_t t4);


// 获取当前油门值
uint16_t DShot_GetThrottle(uint8_t channel);

// DMA传输完成后的重启动函数（需要在main.c的主循环中调用）
void DShot_HandleDMAFlags(void);

// 快速更新所有通道（非阻塞，返回1成功，0=DMA忙跳过）
uint8_t DShot_UpdateAllChannels(void);

// 快速更新单个通道（非阻塞，返回1成功，0=DMA忙跳过）
uint8_t DShot_UpdateSingleChannel(uint8_t channel, uint16_t throttle);

// 重启指定通道的 DMA
void DShot_RestartDMAChannel(uint8_t channel);

extern volatile uint16_t current_throttle[4];
extern volatile uint8_t  serial_throttle_updated;   // 串口调试标志：1=有新油门值待更新到DMA
extern volatile uint8_t  drone_throttle_updated;     // 飞控输出标志：1=ISR 已更新 current_throttle，需主循环刷新 DMA

// DMA缓冲区（用于测试或调试）
extern uint32_t dshot_dma_buffer_ch1[18];
extern uint32_t dshot_dma_buffer_ch2[18];
extern uint32_t dshot_dma_buffer_ch3[18];
extern uint32_t dshot_dma_buffer_ch4[18];


#endif

