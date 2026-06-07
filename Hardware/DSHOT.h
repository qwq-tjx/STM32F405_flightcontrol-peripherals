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

// ==================== 双缓冲数据结构 ====================
// 每通道两个 ping-pong 缓冲区, DMA 双缓冲模式下硬件自动切换
// 任何时候 DMA 读取一个缓冲区, CPU 安全写入另一个, 消除竞态
typedef struct {
    DMA_Stream_TypeDef *stream;       // DMA 数据流
    uint32_t            buf0[ESC_CMD_BUFFER_LEN];  // 缓冲区 0
    uint32_t            buf1[ESC_CMD_BUFFER_LEN];  // 缓冲区 1
} dshot_channel_buf_t;

// ==================== 外部接口函数 ====================

// 初始化DSHOT模块 (TIM8和DMA双缓冲配置)
void DShot_Init(void);

// 设置单个通道的油门值
void DShot_SetThrottle(uint8_t channel, uint16_t throttle);

// 设置所有四个通道的油门值
void DShot_SetAllThrottles(uint16_t t1, uint16_t t2, uint16_t t3, uint16_t t4);

// 获取当前油门值
uint16_t DShot_GetThrottle(uint8_t channel);

// DMA传输完成后的标志处理（双缓冲模式下仅清除标志，无需重启）
void DShot_HandleDMAFlags(void);

// 快速更新所有通道（填充 CPU 安全侧缓冲区，非阻塞，总是成功）
uint8_t DShot_UpdateAllChannels(void);

// 快速更新单个通道（填充 CPU 安全侧缓冲区，非阻塞）
uint8_t DShot_UpdateSingleChannel(uint8_t channel, uint16_t throttle);

// 重启指定通道的 DMA
void DShot_RestartDMAChannel(uint8_t channel);

extern volatile uint16_t current_throttle[4];
extern volatile uint8_t  serial_throttle_updated;   // 串口调试标志: 1=有新油门值待更新到DMA
extern volatile uint8_t  drone_throttle_updated;     // 飞控输出标志: 1=ISR 已更新 current_throttle，需主循环刷新 DMA

// 双缓冲数据（调试用外部访问）
extern dshot_channel_buf_t dshot_ch[4];

// 兼容旧代码的 extern 别名 (指向 ch[0-3].buf0)
extern uint32_t *dshot_dma_buffer_ch1;
extern uint32_t *dshot_dma_buffer_ch2;
extern uint32_t *dshot_dma_buffer_ch3;
extern uint32_t *dshot_dma_buffer_ch4;

#endif

