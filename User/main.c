#include "stm32f4xx.h"
#include "Delay.h"
#include "Serial.h"
#include "mavlink.h"
#include "DSHOT.h"
#include "debug_output.h"
#include "adc.h"
#include "string.h"
#include "wit_c_sdk.h"
#include "TIM.h"
#include "LED.h"

// ==================== TIM6_DAC_IRQn 中断处理 (电池检测 - 1秒周期) ====================

// ==================== TIM7_IRQHandler 中断处理 (MAVLink发送 - 10ms周期) ====================
//MAVLink包括IMU,心跳包

//mavlink.c,364~374行关于DSHOT油门值的打印调试

///添加：发送四个油门值到串口助手
//SendThrottleValues()

//打印 ADC 原始值和电压值（调试用）
//ADC_PrintRaw()

// 定义圆周率常量
#ifndef M_PI
#define M_PI  3.14159265358979323846f
#endif

// DMA缓冲区声明已在DSHOT.h中

int main(void)
{
    // ========== 初始化阶段 ==========
    systick_init();
    USART1_Init();
    Debug_Init();

    // 初始化 IMU
    UART5_Init();
    WitImu_Init();

    // 初始化 UART2 (MAVLink，含DMA)
    USART2_Init();

    // 初始化 DSHOT
    DShot_Init();

    // 初始化 LED (PB6)
    LED_init();

    // ========== 正常模式 ==========
    
    // 初始化 ADC
    ADC_Config();

    // 启动定时器
    TIM7_Init(10000 - 1, 84 - 1);   // MAVLink 100Hz
    TIM6_Init(10000 - 1, 8400 - 1); // 电池 1Hz
	    // ========== 系统初始化完成 ==========
    Serial_Printf("\r\n");
    Serial_Printf("========================================\r\n");
    Serial_Printf("       FLYCONTROL STM32F405 System      \r\n");
    Serial_Printf("========================================\r\n");
    Serial_Printf("System Clock: 168 MHz\r\n");
    Serial_Printf("DSHOT Mode: DSHOT150 (PC6-PC9)\r\n");
    Serial_Printf("MAVLink: Ready (USART2)\r\n");
    Serial_Printf("IMU: WitMotion (UART5)\r\n");
    Serial_Printf("========================================\r\n");
    Serial_Printf("Waiting for RC commands...\r\n");
    Serial_Printf("\r\n");

    // ========== 主循环 ==========
    uint8_t channel;
    uint16_t value;
    uint8_t any_update;

    while (1)
    {
        // 处理油门命令队列
        any_update = 0;

        DSHOT_ENTER_CRITICAL();
        while (throttle_cmd_available()) {
            throttle_cmd_dequeue(&channel, &value);
            current_throttle[channel] = value;
            any_update = 1;
        }
        DSHOT_EXIT_CRITICAL();

        // 有更新时更新DMA缓冲区
        if (any_update) {
            DShot_UpdateAllChannels();
        }

        // LED 1Hz闪烁处理 (与MAVLink心跳同频)
        LED_blink_handler();

        Debug_Process();
    }
}


