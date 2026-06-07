#include "DSHOT.h"
#include "TIM.h"
#include "Delay.h"

// ==================== 静态变量 ====================
// 临界区 PRIMASK 保存变量 (DSHOT_ENTER_CRITICAL/EXIT_CRITICAL 使用)
uint32_t __dshot_crit_primask = 0;

// 当前四个通道的油门值 (0-4095)
volatile uint16_t current_throttle[4] = {0, 0, 0, 0};

// ==================== 双缓冲结构体 ====================
// 每通道两个 ping-pong 缓冲区: DMA 读其中一个, CPU 安全写另一个
dshot_channel_buf_t dshot_ch[4];

// 兼容旧代码的 extern 别名 (指向 buf0，仅调试用)
uint32_t *dshot_dma_buffer_ch1;
uint32_t *dshot_dma_buffer_ch2;
uint32_t *dshot_dma_buffer_ch3;
uint32_t *dshot_dma_buffer_ch4;

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

// 填充指定缓冲区的 DShot 帧（为不同通道添加微小延迟，减少同时翻转）
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

// 获取通道 DMA 正在读取的缓冲区指针（即 CPU 安全侧 = 对面的缓冲区）
// 返回: 指向通道中可被 CPU 安全写入的缓冲区
static uint32_t *DShot_GetSafeBuffer(uint8_t channel)
{
    dshot_channel_buf_t *ch = &dshot_ch[channel];
    // CT=0 → DMA 读 buf0 → CPU 写 buf1
    // CT=1 → DMA 读 buf1 → CPU 写 buf0
    if (ch->stream->CR & DMA_SxCR_CT) {
        return ch->buf0;   // DMA 正在读 buf1, 安全写入 buf0
    }
    return ch->buf1;       // DMA 正在读 buf0, 安全写入 buf1
}

// 配置单通道 DMA 双缓冲模式 (不使用 MYDMA_Config, 避免启停竞态)
static void DShot_ConfigDMAChannel(uint8_t chan, DMA_Stream_TypeDef *stream,
                                    uint32_t ch_sel, uint32_t ccr_addr)
{
    dshot_channel_buf_t *ch = &dshot_ch[chan];
    ch->stream = stream;

    DMA_InitTypeDef DMA_InitStructure;

    // 1. 填充两侧缓冲区为 0 油门
    DShot_FillDMABuffer(ch->buf0, DShot_BuildFrame(0), chan);
    DShot_FillDMABuffer(ch->buf1, DShot_BuildFrame(0), chan);

    // 2. 使能 DMA2 时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    // 3. 复位 DMA 数据流
    DMA_DeInit(stream);
    while (DMA_GetCmdStatus(stream) != DISABLE) {}

    // 4. DMA 基础配置 (Memory→Peripheral, 循环模式 + 双缓冲)
    DMA_InitStructure.DMA_Channel             = ch_sel;
    DMA_InitStructure.DMA_PeripheralBaseAddr  = ccr_addr;
    DMA_InitStructure.DMA_Memory0BaseAddr     = (uint32_t)(ch->buf0);
    DMA_InitStructure.DMA_DIR                 = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize          = ESC_CMD_BUFFER_LEN;
    DMA_InitStructure.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc           = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize  = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize      = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode                = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority            = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode            = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold       = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst         = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst     = DMA_PeripheralBurst_Single;
    DMA_Init(stream, &DMA_InitStructure);

    // 5. 开启双缓冲模式 (CIRC=1 + DBM=1, 自动在 buf0↔buf1 间轮转)
    stream->CR |= DMA_SxCR_DBM;
    stream->M1AR = (uint32_t)(ch->buf1);   // 备选缓冲区地址

    // 6. 使能 TC 中断
    DMA_ITConfig(stream, DMA_IT_TC, ENABLE);

    // 7. NVIC 优先级: 抢占=1, 仅高于飞控主循环(TIM7=2)
    uint8_t irq_channel;
    if      (stream == DMA2_Stream2) irq_channel = DMA2_Stream2_IRQn;
    else if (stream == DMA2_Stream3) irq_channel = DMA2_Stream3_IRQn;
    else if (stream == DMA2_Stream4) irq_channel = DMA2_Stream4_IRQn;
    else                              irq_channel = DMA2_Stream7_IRQn;

    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = irq_channel;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    // 8. 使能 DMA (双缓冲启动后自动循环)
    DMA_Cmd(stream, ENABLE);
}

// ==================== 外部接口函数 ====================

// 初始化 DSHOT 模块 (TIM8 + DMA 双缓冲配置)
void DShot_Init(void)
{
    // 1. 兼容别名初始化
    dshot_dma_buffer_ch1 = dshot_ch[0].buf0;
    dshot_dma_buffer_ch2 = dshot_ch[1].buf0;
    dshot_dma_buffer_ch3 = dshot_ch[2].buf0;
    dshot_dma_buffer_ch4 = dshot_ch[3].buf0;

    // 2. 初始化 TIM8 PWM (ARR=559, PSC=19 → 8.4MHz, DSHOT15)
    TIM8_PWM_Init(560-1, 19);

    // 3. 配置四通道 DMA 双缓冲 (硬件映射: PC6→CH1, PC8→CH2, PC9→CH3, PC7→CH4)
    //    每个通道独立配置, DMA 双缓冲模式自动循环
    DShot_ConfigDMAChannel(0, DMA2_Stream2, DMA_Channel_7, (uint32_t)&(TIM8->CCR1));
    TIM_DMACmd(TIM8, TIM_DMA_CC1, ENABLE);
    DShot_ConfigDMAChannel(1, DMA2_Stream3, DMA_Channel_7, (uint32_t)&(TIM8->CCR2));
    TIM_DMACmd(TIM8, TIM_DMA_CC2, ENABLE);
    DShot_ConfigDMAChannel(2, DMA2_Stream4, DMA_Channel_7, (uint32_t)&(TIM8->CCR3));
    TIM_DMACmd(TIM8, TIM_DMA_CC3, ENABLE);
    DShot_ConfigDMAChannel(3, DMA2_Stream7, DMA_Channel_7, (uint32_t)&(TIM8->CCR4));
    TIM_DMACmd(TIM8, TIM_DMA_CC4, ENABLE);
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

// DMA传输完成后的标志处理（双缓冲模式下仅清除标志，无需重启 DMA）
void DShot_HandleDMAFlags(void)
{
    if(DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) != RESET) {
        DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
        // 双缓冲模式: DMA 已自动交换缓冲区和重载 NDTR，无需手动重启
    }
    if(DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) != RESET) {
        DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
    }
    if(DMA_GetFlagStatus(DMA2_Stream4, DMA_FLAG_TCIF4) != RESET) {
        DMA_ClearFlag(DMA2_Stream4, DMA_FLAG_TCIF4);
    }
    if(DMA_GetFlagStatus(DMA2_Stream7, DMA_FLAG_TCIF7) != RESET) {
        DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);
    }
}

// 快速更新所有通道（ping-pong: 填充 CPU 安全侧缓冲区，始终成功）
// 原理: 通过 DMA_SxCR.CT 位判断 DMA 正在读取哪个缓冲区, CPU 写入对面的缓冲区
//       两个缓冲区不会同时被读写, 彻底消除 DMA 竞态
uint8_t DShot_UpdateAllChannels(void)
{
    uint16_t throttle_copy[4];
    
    // ========== 临界区开始: 原子读取油门值 ==========
    DSHOT_ENTER_CRITICAL();
    throttle_copy[0] = current_throttle[0];
    throttle_copy[1] = current_throttle[1];
    throttle_copy[2] = current_throttle[2];
    throttle_copy[3] = current_throttle[3];
    DSHOT_EXIT_CRITICAL();
    // ========== 临界区结束 ==========
    
    // 填充各通道的 CPU 安全侧缓冲区 (DMA 正在读对面的缓冲区, 无竞态)
    for (uint8_t i = 0; i < 4; i++) {
        uint32_t *safe_buf = DShot_GetSafeBuffer(i);
        DShot_FillDMABuffer(safe_buf, DShot_BuildFrame(throttle_copy[i]), i);
    }
    
    // 数据已写入安全侧缓冲区, DMA 下一次自动切换后即开始使用新值
    return 1;
}

// 快速更新单个通道（ping-pong: 填充 CPU 安全侧缓冲区）
uint8_t DShot_UpdateSingleChannel(uint8_t channel, uint16_t throttle)
{
    if (channel >= 4) return 0;
    if (throttle > 4095) throttle = 4095;
    
    uint32_t *safe_buf = DShot_GetSafeBuffer(channel);
    DShot_FillDMABuffer(safe_buf, DShot_BuildFrame(throttle), channel);
    return 1;
}

// 重启指定通道的 DMA
void DShot_RestartDMAChannel(uint8_t channel)
{
    if (channel >= 4) return;
    
    dshot_channel_buf_t *ch = &dshot_ch[channel];
    DMA_Cmd(ch->stream, DISABLE);
    DMA_SetCurrDataCounter(ch->stream, ESC_CMD_BUFFER_LEN);
    DMA_Cmd(ch->stream, ENABLE);
}

// ==================== DMA TC 中断处理 ====================
// 双缓冲模式下仅清除中断标志, DMA 已自动交换缓冲区和重载 NDTR
void DMA2_Stream2_IRQHandler(void) { if(DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2)) { DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2); } }
void DMA2_Stream3_IRQHandler(void) { if(DMA_GetITStatus(DMA2_Stream3, DMA_IT_TCIF3)) { DMA_ClearITPendingBit(DMA2_Stream3, DMA_IT_TCIF3); } }
void DMA2_Stream4_IRQHandler(void) { if(DMA_GetITStatus(DMA2_Stream4, DMA_IT_TCIF4)) { DMA_ClearITPendingBit(DMA2_Stream4, DMA_IT_TCIF4); } }
void DMA2_Stream7_IRQHandler(void) { if(DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7)) { DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7); } }
