#ifndef __DSHOT_H
#define __DSHOT_H

#include "stm32f4xx.h"

// ==================== 临界区保护宏 ====================
#define DSHOT_ENTER_CRITICAL()  do { __set_PRIMASK(1); } while(0)  // 关闭全局中断
#define DSHOT_EXIT_CRITICAL()   do { __set_PRIMASK(0); } while(0)  // 恢复全局中断

// 缓冲区长度
#define ESC_CMD_BUFFER_LEN     18    // 16数据 + 2位纯低电平

// DSHOT时序参数 (DSHOT300)
#define DSHOT_BIT0             208   // 52*4
#define DSHOT_BIT1             420   // 105*4
static uint8_t Custom_CRC4(uint16_t data_12bit);


// 构建16位帧: 12位油门 + 4位CRC
static uint16_t DShot_BuildFrame(uint16_t throttle);


// 填充DMA缓冲区（为不同通道添加微小延迟，减少同时翻转）
static void DShot_FillDMABuffer(uint32_t *buf, uint16_t frame, uint8_t channel);


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

// DMA缓冲区（用于测试或调试）
extern uint32_t dshot_dma_buffer_ch1[18];
extern uint32_t dshot_dma_buffer_ch2[18];
extern uint32_t dshot_dma_buffer_ch3[18];
extern uint32_t dshot_dma_buffer_ch4[18];


#endif

