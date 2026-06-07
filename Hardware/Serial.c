#include "stm32f4xx.h"
#include <stdio.h>
#include <stdarg.h>
#include "Serial.h"
#include "DSHOT.h"
#include "mavlink.h"
#include "wit_c_sdk.h"

//******************USART1*******************
static uint8_t Serial_RxData;
static uint8_t Serial_RxFlag;

void USART1_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &USART_InitStructure);
    
    USART_Cmd(USART1, ENABLE);
  
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  // 调试串口，最低抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
}

void Serial_SendByte(uint8_t Byte)
{
    USART_SendData(USART1, Byte);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i ++)
    {
        Serial_SendByte(Array[i]);
    }
}

void Serial_SendString(char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i ++)
    {
        Serial_SendByte(String[i]);
    }
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y --)
    {
        Result *= X;
    }
    return Result;
}

void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i ++)
    {
        Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
    }
}

int fputc(int ch, FILE *f)
{
    Serial_SendByte(ch);
    return ch;
}

void Serial_Printf(char *format, ...)
{
    char String[100];
    va_list arg;
    va_start(arg, format);
    vsnprintf(String, sizeof(String), format, arg);  // 防栈溢出
    va_end(arg);
    Serial_SendString(String);
}

uint8_t Serial_GetRxFlag(void)
{
    if (Serial_RxFlag == 1)
    {
        Serial_RxFlag = 0;
        return 1;
    }
    return 0;
}

uint8_t Serial_GetRxData(void)
{
    return Serial_RxData;
}

void CopeCmdData(unsigned char ucData);
void WitImu_CmdProcess(uint8_t ucData);
void WitImu_DataIn(uint8_t ucData);

// ========== 串口油门调试解析器 ==========
// 协议帧格式: @xxx,xxx,xxx,xxx\r\n  (4个油门值，逗号分隔，范围0-4095)
#define THROTTLE_PARSE_IDLE       0
#define THROTTLE_PARSE_COLLECT    1
#define THROTTLE_PARSE_BUF_MAX   32

volatile uint8_t serial_throttle_updated = 0;  // 主循环检测并清零

static uint8_t  throttle_parse_state = THROTTLE_PARSE_IDLE;
static char     throttle_rx_buf[THROTTLE_PARSE_BUF_MAX];
static uint8_t  throttle_rx_idx = 0;

static void Serial_ParseThrottle(uint8_t byte)
{
    switch (throttle_parse_state)
    {
        case THROTTLE_PARSE_IDLE:
            if (byte == '@')
            {
                throttle_parse_state = THROTTLE_PARSE_COLLECT;
                throttle_rx_idx = 0;
            }
            break;

        case THROTTLE_PARSE_COLLECT:
            // 遇到换行符 → 结束帧
            if (byte == '\n')
            {
                throttle_rx_buf[throttle_rx_idx] = '\0';
                throttle_parse_state = THROTTLE_PARSE_IDLE;

                // 解析 4 个逗号分隔值: t1,t2,t3,t4
                if (throttle_rx_idx > 0)
                {
                    uint16_t vals[4] = {0, 0, 0, 0};
                    uint8_t  seg = 0;
                    uint16_t num = 0;
                    uint8_t  has_digit = 0;

                    for (uint8_t i = 0; i < throttle_rx_idx; i++)
                    {
                        char c = throttle_rx_buf[i];

                        if (c == '\r')
                            continue;       // 忽略回车

                        if (c >= '0' && c <= '9')
                        {
                            num = num * 10 + (c - '0');
                            has_digit = 1;
                        }
                        else if (c == ',')
                        {
                            if (seg < 4)
                            {
                                vals[seg] = num;
                                seg++;
                            }
                            num = 0;
                            has_digit = 0;
                        }
                        // 其他字符忽略（如回车 \r）
                    }

                    // 最后一个值（没有尾随逗号）
                    if (has_digit && seg < 4)
                    {
                        vals[seg] = num;
                        seg++;
                    }

                    // 必须收到恰好 4 个值
                    if (seg == 4)
                    {
                        // 限幅 0-4095
                        for (uint8_t k = 0; k < 4; k++)
                        {
                            if (vals[k] > 4095) vals[k] = 4095;
                        }

                        // 设置油门值（临界区保护）
                        DSHOT_ENTER_CRITICAL();
                        current_throttle[0] = vals[0];
                        current_throttle[1] = vals[1];
                        current_throttle[2] = vals[2];
                        current_throttle[3] = vals[3];
                        DSHOT_EXIT_CRITICAL();

                        // 通知主循环更新DMA缓冲区
                        serial_throttle_updated = 1;

                        // 打印油门值、DSHOT帧及二进制到串口助手
                        {
                            uint16_t f0, f1, f2, f3;
                            char b0[17], b1[17], b2[17], b3[17];
                            f0 = (vals[0] << 4) | (((vals[0]>>8)&0x0F) + ((vals[0]>>4)&0x0F) + (vals[0]&0x0F)) & 0x0F;
                            f1 = (vals[1] << 4) | (((vals[1]>>8)&0x0F) + ((vals[1]>>4)&0x0F) + (vals[1]&0x0F)) & 0x0F;
                            f2 = (vals[2] << 4) | (((vals[2]>>8)&0x0F) + ((vals[2]>>4)&0x0F) + (vals[2]&0x0F)) & 0x0F;
                            f3 = (vals[3] << 4) | (((vals[3]>>8)&0x0F) + ((vals[3]>>4)&0x0F) + (vals[3]&0x0F)) & 0x0F;
                            fmt_bin16(f0, b0); fmt_bin16(f1, b1); fmt_bin16(f2, b2); fmt_bin16(f3, b3);
                            Serial_Printf("[THR] SER: %d %d %d %d\r\n", vals[0], vals[1], vals[2], vals[3]);
                            Serial_Printf("          BIN: %s %s %s %s\r\n", b0, b1, b2, b3);
                        }
                    }
                }
            }
            // 缓冲区溢出 → 丢弃帧
            else if (throttle_rx_idx >= THROTTLE_PARSE_BUF_MAX - 1)
            {
                throttle_parse_state = THROTTLE_PARSE_IDLE;
            }
            else
            {
                throttle_rx_buf[throttle_rx_idx++] = byte;
            }
            break;
    }
}

void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        uint8_t ucTemp = USART_ReceiveData(USART1);  // 读取数据自动清除RXNE标志
        Serial_RxData = ucTemp;
        Serial_RxFlag = 1;
        WitImu_CmdProcess(ucTemp);
        Serial_ParseThrottle(ucTemp);  // 并联油门调试解析，不干扰IMU
    }
}

// 添加：发送四个油门值到串口助手
void SendThrottleValues(void)
{
    Serial_Printf("CH0:%d, CH1:%d, CH2:%d, CH3:%d\r\n", 
                  current_throttle[0],
                  current_throttle[1],
                  current_throttle[2],
                  current_throttle[3]);
}

//******************USART2 DMA发送（带队列）*******************
// DMA发送缓冲区
static uint8_t uart2_tx_buffer[256];
// 发送忙标志: 0=空闲, 1=正在发送
static volatile uint8_t uart2_tx_busy = 0;

// ========== 发送队列 ==========
#define UART2_TX_QUEUE_SIZE  8
static volatile uint32_t uart2_tx_queue_overflow = 0;          // 诊断：队列满丢弃计数
static struct {
    uint8_t data[256];
    uint16_t length;
} uart2_tx_queue[UART2_TX_QUEUE_SIZE];
static volatile uint8_t uart2_tx_queue_head = 0;
static volatile uint8_t uart2_tx_queue_tail = 0;
static volatile uint8_t uart2_tx_queue_count = 0;

// ========== 队列临界区保护（防止多ISR竞态） ==========
// USART2_IRQ(pri0) 可抢占 TIM7/DMA1_Stream6(pri2)、TIM6(pri4)
// ，这些ISR都可能调用 USART2_DMA_Send，必须互斥
static uint32_t uart2_tx_crit_primask;
#define UART2_TX_ENTER_CRITICAL()  do { uart2_tx_crit_primask = __get_PRIMASK(); __disable_irq(); } while(0)
#define UART2_TX_EXIT_CRITICAL()   do { if (!uart2_tx_crit_primask) __enable_irq(); } while(0)

// 前向声明
static void USART2_DMA_Send_Next(void);

void USART2_DMA_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStruct;
    
    // 1. 使能DMA1时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    
    // 2. 复位DMA数据流
    DMA_DeInit(DMA1_Stream6);
    while(DMA_GetCmdStatus(DMA1_Stream6) != DISABLE);
    
    // 3. 配置DMA结构体
    DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;                              // USART2_TX对应Channel4
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);         // USART2数据寄存器地址
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)uart2_tx_buffer;          // 内存缓冲区地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                     // 内存到外设
    DMA_InitStructure.DMA_BufferSize = 0;                                       // 数据长度稍后设置
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;            // 外设地址不递增
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                    // 内存地址递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     // 外设数据宽度: 8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;            // 内存数据宽度: 8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                               // 正常模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                       // 中等优先级
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                      // 禁用FIFO
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream6, &DMA_InitStructure);
    
    // 4. 使能DMA传输完成中断
    DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);
    
    // 5. 配置NVIC中断优先级
    NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream6_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;  // USART2 DMA TX, 最低抢占优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

/**
  * @brief  USART2 DMA发送函数（队列缓冲，非阻塞）
  * @param  data: 数据指针
  * @param  length: 数据长度
  * @note   消息进入队列，DMA空闲时自动发送下一条
  */
void USART2_DMA_Send(uint8_t *data, uint16_t length)
{
    UART2_TX_ENTER_CRITICAL();
    
    // 队列满，丢弃数据（记录溢出次数用于诊断）
    if(uart2_tx_queue_count >= UART2_TX_QUEUE_SIZE) {
        uart2_tx_queue_overflow++;
        UART2_TX_EXIT_CRITICAL();
        return;
    }
    
    if(length > 256) {
        length = 256;
    }
    
    // 复制数据到队列
    for(uint16_t i = 0; i < length; i++) {
        uart2_tx_queue[uart2_tx_queue_head].data[i] = data[i];
    }
    uart2_tx_queue[uart2_tx_queue_head].length = length;
    
    // 移动队列头
    uart2_tx_queue_head = (uart2_tx_queue_head + 1) % UART2_TX_QUEUE_SIZE;
    uart2_tx_queue_count++;
    
    // 如果DMA空闲，立即开始发送
    if(!uart2_tx_busy) {
        USART2_DMA_Send_Next();
    }
    
    UART2_TX_EXIT_CRITICAL();
}

/**
  * @brief  从队列发送下一条数据
  */
static void USART2_DMA_Send_Next(void)
{
    if(uart2_tx_queue_count == 0 || uart2_tx_busy) {
        return;
    }
    
    uint16_t length = uart2_tx_queue[uart2_tx_queue_tail].length;
    
    // 复制数据到发送缓冲区
    for(uint16_t i = 0; i < length; i++) {
        uart2_tx_buffer[i] = uart2_tx_queue[uart2_tx_queue_tail].data[i];
    }
    
    // 移动队列尾
    uart2_tx_queue_tail = (uart2_tx_queue_tail + 1) % UART2_TX_QUEUE_SIZE;
    uart2_tx_queue_count--;
    
    // 禁止DMA
    DMA_Cmd(DMA1_Stream6, DISABLE);
    
    // 设置传输数据长度
    DMA_SetCurrDataCounter(DMA1_Stream6, length);
    
    // 使能USART2的DMA发送请求
    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
    
    // 使能DMA
    DMA_Cmd(DMA1_Stream6, ENABLE);
    
    // 设置忙标志
    uart2_tx_busy = 1;
}

/**
  * @brief  DMA1 Stream6 中断处理函数（发送完成）
  */
void DMA1_Stream6_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6)) {
        DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);
        
        // 清除发送忙标志
        uart2_tx_busy = 0;
        
        // 失能USART2的DMA发送请求
        USART_DMACmd(USART2, USART_DMAReq_Tx, DISABLE);
        
        // 发送队列中的下一条消息（临界区保护，防止USART2 ISR抢占破坏队列）
        UART2_TX_ENTER_CRITICAL();
        USART2_DMA_Send_Next();
        UART2_TX_EXIT_CRITICAL();
    }
}

//******************USART2*******************
void USART2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
    
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStruct);
    
    USART_Cmd(USART2, ENABLE);
    
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    
    NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  // 遥控信号，最高优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    
    // 初始化USART2 DMA发送
    USART2_DMA_Init();
}

/* 发送一个字节（阻塞方式） */
void uart2_send_byte(uint8_t data)
{
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    USART_SendData(USART2, data);
}

void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        uint8_t data = USART_ReceiveData(USART2);
        mavlink_parse_byte(data);
    }
}

/**
  * @brief  UART5 初始化
  * @note   引脚：PC12 (TX), PD2 (RX)
  *         波特率：115200
  *         数据位：8
  *         停止位：1
  *         无校验
  */
void UART5_Init(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
    
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
    
    GPIO_InitTypeDef GPIO_InitStruct;
    
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStruct);
    
    USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART5, &USART_InitStruct);
    
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
    
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  // IMU数据，最高优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    
    USART_Cmd(UART5, ENABLE);
    
    while(USART_GetFlagStatus(UART5, USART_FLAG_RXNE) == SET) {
        (void)USART_ReceiveData(UART5);
    }
}

void UART5_IRQHandler(void)
{
    if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
    {
        uint8_t ucTemp = USART_ReceiveData(UART5);  // 读取数据自动清除RXNE标志
        WitImu_DataIn(ucTemp);
    }
}

/**
  * @brief  UART5 发送数据（阻塞方式）
  * @param  p_data: 数据指针
  * @param  uiSize: 数据长度
  */
void UART5_SendData(uint8_t *p_data, uint32_t uiSize)
{
    uint32_t i;
    
    for(i = 0; i < uiSize; i++)
    {
        while(USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);
        USART_SendData(UART5, *p_data++);
    }
    
    while(USART_GetFlagStatus(UART5, USART_FLAG_TC) == RESET);
}
