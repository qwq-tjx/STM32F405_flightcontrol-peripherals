#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>

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

#endif
