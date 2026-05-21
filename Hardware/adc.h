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
  * @note   电路：电池 --- 10KΩ --- PC5 --- 1KΩ --- GND
  *         分压比 = (10K + 1K) / 1K = 11
  */
#define VOLTAGE_DIVIDER_RATIO    11.0f

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
extern volatile uint32_t adc_value;

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
uint16_t ADC_GetRaw(void);                  /* 获取 ADC 原始值 */
void ADC_PrintRaw(void);                    /* 打印调试信息 */

#endif /* __ADC_H */
