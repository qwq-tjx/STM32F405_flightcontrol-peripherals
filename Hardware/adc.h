/**
  * @file    adc.h
  * @brief   ADC 电池电压采样驱动头文件
  */

#ifndef __ADC_H
#define __ADC_H

#include "stm32f4xx.h"

/* ==================== 宏定义 ==================== */

/**
  * @brief  分压电阻比例
  * @note   实测反推：电池 23.7V 时 ADC 引脚约 2.996V，分压比 ≈ 7.91
  *         (理论值 11 不匹配，实际电阻非标准 10K+1K)
  */
#define VOLTAGE_DIVIDER_RATIO    7.91f

/**
  * @brief  内部 VREFINT 工厂校准值地址
  * @note   该值是 VDDA = 3.3V 时 VREFINT 的 ADC 读数
  */
#define VREFINT_CAL_ADDR         ((uint16_t*)0x1FFF7A2A)
#define VREFINT_CAL_VALUE        (*VREFINT_CAL_ADDR)


/* ==================== 外部变量声明 ==================== */

/**
  * @brief  ADC 采样值（由 DMA 自动更新）
  */
extern volatile uint16_t adc_value;

/**
  * @brief  新数据标志位
  */
extern volatile uint8_t adc_ready;

/**
  * @brief  DMA 中断计数器（调试用）
  */
extern volatile uint32_t dma_interrupt_count;

/**
  * @brief  ADC EOC 中断计数器（调试用）
  */
extern volatile uint32_t adc_eoc_count;


/* ==================== 函数声明 ==================== */

void ADC_Config(void);                      /* ADC + DMA 初始化 */
float ADC_GetBatteryVoltage(void);          /* 获取电池电压（V）*/
float ADC_GetVDDA(void);                    /* 获取校准后 VDDA 电压（V）*/
uint16_t ADC_GetRaw(void);                  /* 获取 ADC 原始值 */
void ADC_PrintRaw(void);                    /* 打印调试信息 */

#endif /* __ADC_H */
