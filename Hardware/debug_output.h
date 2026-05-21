#ifndef DEBUG_OUTPUT_H
#define DEBUG_OUTPUT_H

#include "stm32f4xx.h"

#define DEBUG_BUF_SIZE 1024

typedef struct {
    char buffer[DEBUG_BUF_SIZE];
    uint16_t head;
    uint16_t tail;
} RingBuffer;

void Debug_Init(void);
void Debug_Printf(const char* format, ...);
void Debug_Process(void);  // 在 main 循环中调用

#endif
