#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx.h"

/* 辅助函数: 16位值转二进制字符串 (static 每个编译单元持有独立副本) */
static void fmt_bin16(uint16_t v, char *buf)
{
    for (int8_t i = 15; i >= 0; i--) {
        buf[15 - i] = (v & ((uint16_t)1 << i)) ? '1' : '0';
    }
    buf[16] = '\0';
}

void USART1_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

void SendThrottleValues(void);

void USART2_Init(void);
void uart2_send_byte(uint8_t data);
void USART2_DMA_Send(uint8_t *data, uint16_t length);

uint8_t Serial_GetRxFlag(void);
uint8_t Serial_GetRxData(void);
void UART5_Init(void);
void UART5_SendData(uint8_t *p_data, uint32_t uiSize);

/* ---- UART3 光流传感器 (MTF-01) ---- */
typedef struct {
    volatile int16_t  flow_vel_x;     /* 光流 X 速度 (cm/s @ 1m) */
    volatile int16_t  flow_vel_y;     /* 光流 Y 速度 (cm/s @ 1m) */
    volatile uint32_t distance;       /* 激光测距 (mm), 0=不可用 */
    volatile uint8_t  flow_quality;   /* 光流质量 (0~100) */
    volatile uint8_t  fresh;          /* 新数据标志: UART3 ISR 置1, TIM7 ISR 清0 */
} optflow_raw_t;

extern volatile optflow_raw_t g_optflow;

void UART3_Init(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);

#endif
