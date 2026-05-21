// debug_output.c
#include "debug_output.h"
#include "Serial.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h> 
static RingBuffer tx_ring;
static char print_buffer[256];

void Debug_Init(void)
{
    tx_ring.head = 0;
    tx_ring.tail = 0;
    memset(tx_ring.buffer, 0, DEBUG_BUF_SIZE);
}

void Debug_Printf(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    int len = vsnprintf(print_buffer, sizeof(print_buffer), format, args);
    va_end(args);
    
    if(len <= 0) return;
    
    // 写入环形缓冲区
    for(int i = 0; i < len; i++) {
        uint16_t next_head = (tx_ring.head + 1) % DEBUG_BUF_SIZE;
        if(next_head != tx_ring.tail) {  // 缓冲区未满
            tx_ring.buffer[tx_ring.head] = print_buffer[i];
            tx_ring.head = next_head;
        } else {
            break;  // 缓冲区满，丢弃数据
        }
    }
}

void Debug_Process(void)
{
    while(tx_ring.head != tx_ring.tail) {
        if(USART_GetFlagStatus(USART1, USART_FLAG_TXE)) {
            USART_SendData(USART1, tx_ring.buffer[tx_ring.tail]);
            tx_ring.tail = (tx_ring.tail + 1) % DEBUG_BUF_SIZE;
        } else {
            break;  // 发送寄存器忙，下次再发
        }
    }
}
