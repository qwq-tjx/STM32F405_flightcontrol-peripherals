#include "stm32f4xx.h"
#include "adc.h"
#include "mavlink.h"
#include "Serial.h"
#include "wit_c_sdk.h"
// 任务调度定时器初始化
extern void mavlink_send_imu_periodic(void);
extern float ADC_GetBatteryVoltage(void);
extern void mavlink_send_battery_voltage(float voltage);

/**
  * @brief  DMA 配置函数（支持优先级参数）
  * @param  DMA_Streamx: DMA 数据流 (如 DMA2_Stream0, DMA2_Stream2 等)
  * @param  chx: DMA 通道 (如 DMA_Channel_0, DMA_Channel_7)
  * @param  par: 外设地址
  * @param  mar: 存储器地址
  * @param  ndtr: 数据传输量
  * @param  priority: DMA 优先级 (DMA_Priority_Low, DMA_Priority_Medium, 
  *                   DMA_Priority_High, DMA_Priority_VeryHigh)
  * @retval 无
  */
void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx, u32 chx, u32 par, u32 mar, u16 ndtr, uint32_t priority)
{
    DMA_InitTypeDef  DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStruct;
    
    // ==================== 1. 使能 DMA 时钟 ====================
    if(((uint32_t)DMA_Streamx & 0xFFFF0000) == ((uint32_t)DMA2 & 0xFFFF0000))
    {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    }
    else
    {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    }
    
    // ==================== 2. 复位 DMA 数据流 ====================
    DMA_DeInit(DMA_Streamx);
    
    // 等待 DMA 数据流被禁用
    while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE) {}
    
    // ==================== 3. DMA 基本配置 ====================
    DMA_InitStructure.DMA_Channel = chx;                              // DMA 通道
    DMA_InitStructure.DMA_PeripheralBaseAddr = par;                   // 外设地址
    DMA_InitStructure.DMA_Memory0BaseAddr = mar;                      // 存储器地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;           // 传输方向: 存储器 -> 外设
    DMA_InitStructure.DMA_BufferSize = ndtr;                          // 传输数据量
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  // 外设地址不递增
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;           // 存储器地址递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word; // 外设数据宽度: 32位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;         // 存储器数据宽度: 32位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                     // 循环模式 - DMA持续自动重载！
    DMA_InitStructure.DMA_Priority = priority;                        // DMA 优先级（由参数传入）
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;            // 禁用 FIFO
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;     // FIFO 阈值
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;       // 存储器突发单次传输
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; // 外设突发单次传输
    
    // 初始化 DMA
    DMA_Init(DMA_Streamx, &DMA_InitStructure);
    
    // ==================== 4. 根据 DMA 数据流使能 DMA 中断 ====================
    // 使能传输完成中断
    DMA_ITConfig(DMA_Streamx, DMA_IT_TC, ENABLE);
    
    // ==================== 5. 配置 NVIC 中断优先级 ====================
    // 根据 DMA 数据流获取对应的中断通道
    uint8_t irq_channel = 0;
    
    if(DMA_Streamx == DMA2_Stream0)      irq_channel = DMA2_Stream0_IRQn;
    else if(DMA_Streamx == DMA2_Stream1) irq_channel = DMA2_Stream1_IRQn;
    else if(DMA_Streamx == DMA2_Stream2) irq_channel = DMA2_Stream2_IRQn;
    else if(DMA_Streamx == DMA2_Stream3) irq_channel = DMA2_Stream3_IRQn;
    else if(DMA_Streamx == DMA2_Stream4) irq_channel = DMA2_Stream4_IRQn;
    else if(DMA_Streamx == DMA2_Stream5) irq_channel = DMA2_Stream5_IRQn;
    else if(DMA_Streamx == DMA2_Stream6) irq_channel = DMA2_Stream6_IRQn;
    else if(DMA_Streamx == DMA2_Stream7) irq_channel = DMA2_Stream7_IRQn;
    else if(DMA_Streamx == DMA1_Stream0) irq_channel = DMA1_Stream0_IRQn;
    else if(DMA_Streamx == DMA1_Stream1) irq_channel = DMA1_Stream1_IRQn;
    else if(DMA_Streamx == DMA1_Stream2) irq_channel = DMA1_Stream2_IRQn;
    else if(DMA_Streamx == DMA1_Stream3) irq_channel = DMA1_Stream3_IRQn;
    else if(DMA_Streamx == DMA1_Stream4) irq_channel = DMA1_Stream4_IRQn;
    else if(DMA_Streamx == DMA1_Stream5) irq_channel = DMA1_Stream5_IRQn;
    else if(DMA_Streamx == DMA1_Stream6) irq_channel = DMA1_Stream6_IRQn;
    else if(DMA_Streamx == DMA1_Stream7) irq_channel = DMA1_Stream7_IRQn;
    else return;  // 不支持的 DMA 数据流，直接返回
    
    // 根据 DMA 优先级设置 NVIC 抢占优先级
    // DShot: VeryHigh/High → 抢占优先级 0（最高）
    // ADC:   Low/Medium   → 抢占优先级 1（较低）
    if(priority == DMA_Priority_VeryHigh || priority == DMA_Priority_High)
    {
        NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    }
    else
    {
        NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
        NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    }
    
    NVIC_InitStruct.NVIC_IRQChannel = irq_channel;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    
    // ==================== 6. 使能 DMA 数据流 ====================
    DMA_Cmd(DMA_Streamx, ENABLE);
}

void TIM8_PWM_Init(u32 arr,u32 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE); // 改用APB2时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); // 启用GPIOC时钟
	// ==================== 2. GPIO 复用配置 ====================
    // PC6 -> TIM8_CH1
    // PC7 -> TIM8_CH2
    // PC8 -> TIM8_CH3
    // PC9 -> TIM8_CH4
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // ==================== 3. 时基单元配置 ====================
    TIM_TimeBaseStructure.TIM_Prescaler = psc;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = arr;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; // 高级定时器特有，设为0
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
    // ==================== 4. 输出比较通道配置 ====================
    TIM_OCStructInit(&TIM_OCInitStructure); // 先全部填充默认值
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset; // 高级定时器必需
    TIM_OCInitStructure.TIM_Pulse = 0; // 初始占空比为0

     // 通道1 (PC6)
    TIM_OC1Init(TIM8, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);

    // 通道2 (PC7)
    TIM_OC2Init(TIM8, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);

    // 通道3 (PC8)
    TIM_OC3Init(TIM8, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);

    // 通道4 (PC9)
    TIM_OC4Init(TIM8, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);

    // ==================== 5. 自动重装载预装载 ====================
    TIM_ARRPreloadConfig(TIM8, ENABLE);

    // ==================== 6. 刹车与死区配置（不使用但必须初始化）====================
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
    TIM_BDTRStructInit(&TIM_BDTRInitStructure);
    TIM_BDTRConfig(TIM8, &TIM_BDTRInitStructure);

    // ==================== 7. 使能定时器 ====================
    TIM_Cmd(TIM8, ENABLE);
    TIM_CtrlPWMOutputs(TIM8, ENABLE); // 高级定时器必须使能主输出
    // 注: TIM8 DMA 中断已在 DShot_Init() 中配置，无需额外 NVIC
}

// ==================== TIM6 初始化 (1秒周期 - 电池检测) ====================
void TIM6_Init(u32 arr, u32 psc)
{
    NVIC_InitTypeDef NVIC_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    // 1. 使能TIM6时钟 (APB1)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    // 2. 配置时基
    TIM_TimeBaseStructure.TIM_Prescaler = psc;       // 预分频 (84MHz / psc)
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = arr;           // 自动重装载值
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

    // 3. 使能更新中断
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

    // 4. 配置NVIC
    NVIC_InitStruct.NVIC_IRQChannel = TIM6_DAC_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 4;  // 低优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    // 5. 使能定时器
    TIM_Cmd(TIM6, ENABLE);
}

// ==================== TIM7 初始化 (10ms周期 - MAVLink发送) ====================
void TIM7_Init(u32 arr, u32 psc)
{
    NVIC_InitTypeDef NVIC_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    // 1. 使能TIM7时钟 (APB1)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

    // 2. 配置时基
    TIM_TimeBaseStructure.TIM_Prescaler = psc;       // 预分频 (84MHz / psc)
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = arr;           // 自动重装载值
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

    // 3. 使能更新中断
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);

    // 4. 配置NVIC
    NVIC_InitStruct.NVIC_IRQChannel = TIM7_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;  // 中等优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    // 5. 使能定时器
    TIM_Cmd(TIM7, ENABLE);
}

// ==================== TIM6_DAC_IRQn 中断处理 (电池检测 - 1秒周期) ====================
void TIM6_DAC_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);

        // 读取电池电压并发送
        float voltage = ADC_GetBatteryVoltage();
        mavlink_send_battery_voltage(voltage);
	
    }
}

// ==================== TIM7_IRQHandler 中断处理 (MAVLink发送 - 10ms周期) ====================
void TIM7_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
        
        // 周期性发送IMU数据
        mavlink_send_imu_periodic();
    }
}
