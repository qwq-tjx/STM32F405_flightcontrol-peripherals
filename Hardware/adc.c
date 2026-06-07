/**
  * @file    adc.c
  * @brief   ADC 电池电压采样驱动（使用 DMA 循环模式）
  * @note    硬件连接：PC5 (ADC1_CH15) ← 10K+1K 分压 ← 电池
  */

#include "adc.h"
#include "stm32f4xx.h"
#include "Serial.h"
#include "Delay.h"

/* ==================== 全局变量 ==================== */

/**
  * @brief  ADC 采样值（由 DMA 自动更新）
  * @note   取值范围：0 ~ 4095
  */
volatile uint32_t adc_value = 0;

/**
  * @brief  新数据标志位
  * @note   在 DMA 中断中置 1，主循环中清零
  */
volatile uint8_t adc_ready = 0;

/**
  * @brief  DMA 中断计数器（调试用）
  */
volatile uint32_t dma_interrupt_count = 0;

/**
  * @brief  ADC EOC 中断计数器（调试用）
  */
volatile uint32_t adc_eoc_count = 0;


/* ==================== 内部变量 ==================== */

/**
 * @brief  校准后的实际 VDDA 电压 (V)
 * @note   上电时通过 VREFINT 工厂校准值自动计算，默认 3.3V
 */
static float g_VDDA = 3.3f;

/* ==================== 函数实现 ==================== */

/**
  * @brief  ADC + DMA 初始化函数
  * @note   配置 PC5 为模拟输入，ADC1 连续转换，DMA2 循环搬运
  * @retval 无
  */
void ADC_Config(void)
{
    /* ========== 1. GPIO 初始化 ========== */
    /* 使能 GPIOC 时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;          /* PC5 */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;       /* 模拟输入模式 */
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;   /* 无上下拉 */
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* ========== 2. ADC 和 DMA 时钟使能 ========== */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);  /* ADC1 时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);  /* DMA2 时钟 */

    /* ========== 3. ADC 公共配置 ========== */
    ADC_CommonInitTypeDef ADC_CommonInitStruct;
    ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;           /* 独立模式 */
    ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div4;        /* 预分频 4 */
    ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; /* 单 ADC 模式 */
    ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStruct);

    /* ========== 4. ADC 初始化 ========== */
    ADC_InitTypeDef ADC_InitStruct;
    ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;        /* 12 位分辨率 */
    ADC_InitStruct.ADC_ScanConvMode = DISABLE;                 /* 非扫描模式（单通道）*/
    ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;            /* 连续转换模式（关键！）*/
    ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; /* 软件触发 */
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;        /* 数据右对齐 */
    ADC_InitStruct.ADC_NbrOfConversion = 1;                    /* 1 个转换通道 */
    ADC_Init(ADC1, &ADC_InitStruct);

    /* ========== 5. 配置 ADC 通道 ========== */
    /* PC5 = ADC1 通道 15，采样时间 56 周期 */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_56Cycles);

    /* ========== 6. DMA 配置 ========== */
    DMA_InitTypeDef DMA_InitStruct;
    DMA_InitStruct.DMA_Channel = DMA_Channel_0;                           /* 通道 0 */
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);        /* 外设地址：ADC 数据寄存器 */
    DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&adc_value;            /* 内存地址：adc_value 变量 */
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;                  /* 外设 → 内存 */
    DMA_InitStruct.DMA_BufferSize = 1;                                    /* 传输数量：1 */
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;         /* 外设地址不变 */
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Disable;                 /* 内存地址不变 */
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;  /* 外设数据宽度：32 位（关键！）*/
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;          /* 内存数据宽度：32 位 */
    DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;                          /* 循环模式（关键！）*/
    DMA_InitStruct.DMA_Priority = DMA_Priority_Low;                       /* 低优先级（不影响 DShot）*/
    DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;                   /* 禁用 FIFO */
    DMA_Init(DMA2_Stream0, &DMA_InitStruct);                              /* 使用 DMA2 数据流 0 */

    /* ========== 7. 中断配置 ========== */
    /* 使能 DMA 传输完成中断 */
    DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
    /* 使能 ADC EOC 中断（调试用）*/
    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

    /* DMA 中断优先级配置 */
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream0_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;   // 电池采样，最低抢占优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    
    /* ADC EOC 中断优先级配置（调试用）*/
    NVIC_InitStruct.NVIC_IRQChannel = ADC_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    /* ========== 8. 启动 DMA 和 ADC ========== */
    DMA_Cmd(DMA2_Stream0, ENABLE);        /* 使能 DMA 数据流 */
    ADC_DMACmd(ADC1, ENABLE);             /* ADC 使能 DMA 请求 */
    ADC1->CR2 |= ADC_CR2_DDS;             /* 使能 DMA 循环模式（关键！）*/
    ADC_Cmd(ADC1, ENABLE);                /* 使能 ADC */

    /* ========== 9. ADC 校准 ========== */
    ADC1->CR2 |= (1 << 30);               /* 设置 CAL 位，开始校准 */
    while(ADC1->CR2 & (1 << 30));         /* 等待校准完成 */

    /* ========== 10. VREFINT 校准 —— 计算实际 VDDA ========== */
    /* 原理: VREFINT 内部基准 ~1.21V 不随 VDDA 变化
     *       工厂校准时 VDDA=3.3V, VREFINT 读数为 VREFINT_CAL_VALUE
     *       实际 VDDA = 3.3V * VREFINT_CAL_VALUE / 实际 VREFINT 读数 */
    ADC_TempSensorVrefintCmd(ENABLE);                    /* 使能内部 VREFINT */
    Delay_ms(2);                                         /* 等待 VREFINT 稳定 */

    ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 1,    /* 临时切换到 VREFINT 通道 */
                             ADC_SampleTime_480Cycles);

    ADC_SoftwareStartConv(ADC1);                         /* 启动单次转换 */
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));       /* 等待转换完成 */
    uint16_t vrefint_reading = ADC_GetConversionValue(ADC1);

    if (vrefint_reading > 0) {
        g_VDDA = 3.3f * VREFINT_CAL_VALUE / (float)vrefint_reading;
    }
    /* 注意: 若 VREFINT_CAL_VALUE 出厂未烧录 (0xFFFF), g_VDDA 保持默认 3.3f */

    /* ========== 11. 切回电池通道，启动 DMA 连续转换 ========== */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1,    /* 切回电池检测通道 */
                             ADC_SampleTime_56Cycles);
    ADC_SoftwareStartConv(ADC1);                         /* 启动连续转换 */

}


/**
  * @brief  获取电池电压
  * @retval 电池电压值（单位：伏特）
  * @note   ========== 24V 电池采样电路说明 ==========
  *         电路拓扑：24V ──[10KΩ]──┬──[1KΩ]── GND
  *                                │
  *                              ADC_IN
  *         分压比 = (10K + 1K) / 1K = 11
  *
  *         ========== 电压换算公式 ==========
  *         ADC 满量程 = g_VDDA（由工厂 VREFINT 校准自动计算）
  *         采样点电压 = adc_value × (g_VDDA / 4096)
  *         电池电压   = 采样点电压 × 11
  *
  *         ========== 示例计算 ==========
  *         假设 adc_value = 2700 (24V电池), g_VDDA = 3.3V
  *         采样点电压 = 2700 × 3.3 / 4096 = 2.175V
  *         电池电压   = 2.175 × 11 = 23.93V ≈ 24V ✓
  */
float ADC_GetBatteryVoltage(void)
{
    const float DIVIDER_RATIO = 7.91f;    /* 电压分压比：实测反推 */

    /* 第1步：计算 ADC 采样点的实际电压（使用校准后的 VDDA）*/
    float voltage = (adc_value * g_VDDA) / 4096.0f;

    /* 第2步：根据分压比还原电池电压 */
    return voltage * DIVIDER_RATIO;
}


/**
  * @brief  获取 ADC 原始值
  * @retval ADC 采样原始值（0 ~ 4095）
  */
uint16_t ADC_GetRaw(void)
{
    return (uint16_t)adc_value;
}


/**
 * @brief  获取校准后的 VDDA 电压值
 * @retval VDDA 电压 (V)，由 VREFINT 工厂校准自动计算
 */
float ADC_GetVDDA(void)
{
    return g_VDDA;
}

/**
 * @brief  打印 ADC 原始值、电压值和 VDDA（调试用）
 */
void ADC_PrintRaw(void)
{
    Serial_Printf("ADC Raw: %lu, VDDA: %.3fV, Battery: %.2fV\r\n",
                  adc_value, g_VDDA, ADC_GetBatteryVoltage());
}


/**
  * @brief  DMA2 数据流 0 中断服务函数
  * @note   每次 DMA 传输完成时触发，约 10Hz
  */
void DMA2_Stream0_IRQHandler(void)
{
    /* 检查传输完成中断标志 */
    if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))
    {
        DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);  /* 清除中断标志 */
        dma_interrupt_count++;                              /* 调试计数 */
        adc_ready = 1;                                      /* 通知主循环有新数据 */
    }
}


/**
  * @brief  ADC 中断服务函数（调试用）
  * @note   每次 ADC 转换完成时触发
  */
void ADC_IRQHandler(void)
{
    /* 检查 EOC（转换完成）中断标志 */
    if(ADC_GetITStatus(ADC1, ADC_IT_EOC))
    {
        ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);  /* 清除中断标志 */
        adc_eoc_count++;                          /* 调试计数 */
    }
}
