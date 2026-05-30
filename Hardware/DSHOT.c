#include "DSHOT.h"
#include "TIM.h"
#include "Delay.h"

// ==================== 静态变量 ====================
// 临界区 PRIMASK 保存变量 (DSHOT_ENTER_CRITICAL/EXIT_CRITICAL 使用)
uint32_t __dshot_crit_primask = 0;

// 当前四个通道的油门值 (0-4095)
volatile uint16_t current_throttle[4] = {0, 0, 0, 0};

// DMA缓冲区（外部可访问，用于测试）
uint32_t dshot_dma_buffer_ch1[ESC_CMD_BUFFER_LEN];
uint32_t dshot_dma_buffer_ch2[ESC_CMD_BUFFER_LEN];
uint32_t dshot_dma_buffer_ch3[ESC_CMD_BUFFER_LEN];
uint32_t dshot_dma_buffer_ch4[ESC_CMD_BUFFER_LEN];

// ==================== 内部函数 ====================

// 4位CRC: 4位分组求和后 mod 16
static uint8_t Custom_CRC4(uint16_t data_12bit)
{
    uint8_t group1 = (data_12bit >> 8) & 0x0F;
    uint8_t group2 = (data_12bit >> 4) & 0x0F;
    uint8_t group3 = data_12bit & 0x0F;
    
    uint8_t sum = group1 + group2 + group3;
    return sum & 0x0F;
}

// 构建16位帧: 12位油门 + 4位CRC
static uint16_t DShot_BuildFrame(uint16_t throttle)
{
    if(throttle > 4095) throttle = 4095;
    return (throttle << 4) | Custom_CRC4(throttle);
}

// 填充DMA缓冲区（为不同通道添加微小延迟，减少同时翻转）
static void DShot_FillDMABuffer(uint32_t *buf, uint16_t frame, uint8_t channel)
{
    uint8_t i;
    uint32_t base_delay = channel * 10;
    
    for(i = 0; i < 16; i++)
    {
        if(frame & (1 << (15 - i)))
            buf[i] = DSHOT_BIT1 + base_delay;
        else
            buf[i] = DSHOT_BIT0 + base_delay;
    }
    buf[16] = 0;
    buf[17] = 0;
}

// ==================== 外部接口函数 ====================

// 初始化DSHOT模块 (TIM8和DMA配置)
void DShot_Init(void)
{
    // 1. 初始化TIM8 PWM (ARR=559, PSC=19 → 8.4MHz, DSHOT15)
    TIM8_PWM_Init(560-1, 19);
    
    // 2. 初始化DMA缓冲区（全部为0油门）
    DShot_FillDMABuffer(dshot_dma_buffer_ch1, DShot_BuildFrame(0), 0);
    DShot_FillDMABuffer(dshot_dma_buffer_ch2, DShot_BuildFrame(0), 1);
    DShot_FillDMABuffer(dshot_dma_buffer_ch3, DShot_BuildFrame(0), 2);
    DShot_FillDMABuffer(dshot_dma_buffer_ch4, DShot_BuildFrame(0), 3);
    
    // ========== 3. 配置 DMA2_Stream2 (电机1, 通道1) ==========
    MYDMA_Config(DMA2_Stream2, DMA_Channel_7, (uint32_t)&(TIM8->CCR1), 
                 (uint32_t)dshot_dma_buffer_ch1, ESC_CMD_BUFFER_LEN, DMA_Priority_VeryHigh);
    TIM_DMACmd(TIM8, TIM_DMA_CC1, ENABLE);
    
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);
    
    // ========== 4. 配置 DMA2_Stream3 (电机2, 通道2) ==========
    MYDMA_Config(DMA2_Stream3, DMA_Channel_7, (uint32_t)&(TIM8->CCR2), 
                 (uint32_t)dshot_dma_buffer_ch2, ESC_CMD_BUFFER_LEN, DMA_Priority_VeryHigh);
    TIM_DMACmd(TIM8, TIM_DMA_CC2, ENABLE);
    NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream3_IRQn;
    NVIC_Init(&NVIC_InitStruct);
    DMA_ITConfig(DMA2_Stream3, DMA_IT_TC, ENABLE);
    
    // ========== 5. 配置 DMA2_Stream4 (电机3, 通道3) ==========
    MYDMA_Config(DMA2_Stream4, DMA_Channel_7, (uint32_t)&(TIM8->CCR3), 
                 (uint32_t)dshot_dma_buffer_ch3, ESC_CMD_BUFFER_LEN, DMA_Priority_VeryHigh);
    TIM_DMACmd(TIM8, TIM_DMA_CC3, ENABLE);
    NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream4_IRQn;
    NVIC_Init(&NVIC_InitStruct);
    DMA_ITConfig(DMA2_Stream4, DMA_IT_TC, ENABLE);
    
    // ========== 6. 配置 DMA2_Stream7 (电机4, 通道4) ==========
    MYDMA_Config(DMA2_Stream7, DMA_Channel_7, (uint32_t)&(TIM8->CCR4), 
                 (uint32_t)dshot_dma_buffer_ch4, ESC_CMD_BUFFER_LEN, DMA_Priority_VeryHigh);
    TIM_DMACmd(TIM8, TIM_DMA_CC4, ENABLE);
    NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream7_IRQn;
    NVIC_Init(&NVIC_InitStruct);
    DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);
    
    // 7. 启动所有DMA
    DMA_Cmd(DMA2_Stream2, ENABLE);
    DMA_Cmd(DMA2_Stream3, ENABLE);
    DMA_Cmd(DMA2_Stream4, ENABLE);
    DMA_Cmd(DMA2_Stream7, ENABLE);
}

// 设置单个通道的油门值
void DShot_SetThrottle(uint8_t channel, uint16_t throttle)
{
    if(channel >= 4) return;
    if(throttle > 4095) throttle = 4095;
    
    DSHOT_ENTER_CRITICAL();
    current_throttle[channel] = throttle;
    DSHOT_EXIT_CRITICAL();
}

// 设置所有四个通道的油门值（不立即更新，需手动调用 DShot_UpdateAllChannels）
void DShot_SetAllThrottles(uint16_t t1, uint16_t t2, uint16_t t3, uint16_t t4)
{
    DSHOT_ENTER_CRITICAL();
    if(t1 > 4095) t1 = 4095;
    if(t2 > 4095) t2 = 4095;
    if(t3 > 4095) t3 = 4095;
    if(t4 > 4095) t4 = 4095;
    
    current_throttle[0] = t1;
    current_throttle[1] = t2;
    current_throttle[2] = t3;
    current_throttle[3] = t4;
    DSHOT_EXIT_CRITICAL();
}

// 获取当前油门值
uint16_t DShot_GetThrottle(uint8_t channel)
{
    if(channel >= 4) return 0;
    return current_throttle[channel];
}

// DMA传输完成后的重启动函数（需要在main.c的主循环中调用）
void DShot_HandleDMAFlags(void)
{
    if(DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) != RESET)
    {
        DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
        DMA_SetCurrDataCounter(DMA2_Stream2, ESC_CMD_BUFFER_LEN);
        DMA_Cmd(DMA2_Stream2, ENABLE);
    }
    if(DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) != RESET)
    {
        DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
        DMA_SetCurrDataCounter(DMA2_Stream3, ESC_CMD_BUFFER_LEN);
        DMA_Cmd(DMA2_Stream3, ENABLE);
    }
    if(DMA_GetFlagStatus(DMA2_Stream4, DMA_FLAG_TCIF4) != RESET)
    {
        DMA_ClearFlag(DMA2_Stream4, DMA_FLAG_TCIF4);
        DMA_SetCurrDataCounter(DMA2_Stream4, ESC_CMD_BUFFER_LEN);
        DMA_Cmd(DMA2_Stream4, ENABLE);
    }
    if(DMA_GetFlagStatus(DMA2_Stream7, DMA_FLAG_TCIF7) != RESET)
    {
        DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);
        DMA_SetCurrDataCounter(DMA2_Stream7, ESC_CMD_BUFFER_LEN);
        DMA_Cmd(DMA2_Stream7, ENABLE);
    }
}

// 快速更新所有通道（批量同步，非阻塞）
// 返回值：1=成功更新，0=DMA忙，跳过本次更新
uint8_t DShot_UpdateAllChannels(void)
{
    uint16_t throttle_copy[4];
    
    // ========== 临界区开始 ==========
    DSHOT_ENTER_CRITICAL();
    
    // 复制油门值（防止读取时被中断修改）
    throttle_copy[0] = current_throttle[0];
    throttle_copy[1] = current_throttle[1];
    throttle_copy[2] = current_throttle[2];
    throttle_copy[3] = current_throttle[3];
    
    DSHOT_EXIT_CRITICAL();
    // ========== 临界区结束 ==========
    
    // 填充新数据到 DMA 缓冲区（使用本地副本）
    DShot_FillDMABuffer(dshot_dma_buffer_ch1, DShot_BuildFrame(throttle_copy[0]), 0);
    DShot_FillDMABuffer(dshot_dma_buffer_ch2, DShot_BuildFrame(throttle_copy[1]), 1);
    DShot_FillDMABuffer(dshot_dma_buffer_ch3, DShot_BuildFrame(throttle_copy[2]), 2);
    DShot_FillDMABuffer(dshot_dma_buffer_ch4, DShot_BuildFrame(throttle_copy[3]), 3);
    
    // 循环模式下DMA自动重载，无需手动重启
    // 数据会在下一个DMA周期自动使用新值
    return 1;
}

// 快速更新单个通道（非阻塞）
// 返回值：1=成功更新，0=DMA忙，跳过本次更新
uint8_t DShot_UpdateSingleChannel(uint8_t channel, uint16_t throttle)
{
    uint32_t *buf;
    DMA_Stream_TypeDef *dma_stream;
    uint32_t dma_flag;
    
    if (channel >= 4) return 0;
    if (throttle > 4095) throttle = 4095;
    
    // 选择对应的 DMA 缓冲区和流
    switch(channel) {
        case 0: buf = dshot_dma_buffer_ch1; dma_stream = DMA2_Stream2; dma_flag = DMA_FLAG_TCIF2; break;
        case 1: buf = dshot_dma_buffer_ch2; dma_stream = DMA2_Stream3; dma_flag = DMA_FLAG_TCIF3; break;
        case 2: buf = dshot_dma_buffer_ch3; dma_stream = DMA2_Stream4; dma_flag = DMA_FLAG_TCIF4; break;
        case 3: buf = dshot_dma_buffer_ch4; dma_stream = DMA2_Stream7; dma_flag = DMA_FLAG_TCIF7; break;
        default: return 0;
    }
    
    // 检查 DMA 是否完成
    if (DMA_GetFlagStatus(dma_stream, dma_flag) == RESET) return 0;
    
    // 填充 DMA 缓冲区
    DShot_FillDMABuffer(buf, DShot_BuildFrame(throttle), channel);
    
    // 重启 DMA
    DMA_Cmd(dma_stream, DISABLE);
    DMA_SetCurrDataCounter(dma_stream, ESC_CMD_BUFFER_LEN);
    DMA_Cmd(dma_stream, ENABLE);
    
    return 1;
}

// 重启指定通道的 DMA
void DShot_RestartDMAChannel(uint8_t channel)
{
    if (channel >= 4) return;
    
    DMA_Stream_TypeDef *stream;
    switch(channel) {
        case 0: stream = DMA2_Stream2; break;
        case 1: stream = DMA2_Stream3; break;
        case 2: stream = DMA2_Stream4; break;
        case 3: stream = DMA2_Stream7; break;
        default: return;
    }
    
    DMA_Cmd(stream, DISABLE);
    DMA_SetCurrDataCounter(stream, ESC_CMD_BUFFER_LEN);
    DMA_Cmd(stream, ENABLE);
}

// DMA 中断处理函数
void DMA2_Stream2_IRQHandler(void) { if(DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2)) { DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2); } }
void DMA2_Stream3_IRQHandler(void) { if(DMA_GetITStatus(DMA2_Stream3, DMA_IT_TCIF3)) { DMA_ClearITPendingBit(DMA2_Stream3, DMA_IT_TCIF3); } }
void DMA2_Stream4_IRQHandler(void) { if(DMA_GetITStatus(DMA2_Stream4, DMA_IT_TCIF4)) { DMA_ClearITPendingBit(DMA2_Stream4, DMA_IT_TCIF4); } }
void DMA2_Stream7_IRQHandler(void) { if(DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7)) { DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7); } }
