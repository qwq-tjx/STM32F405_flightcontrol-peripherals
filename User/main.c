#include "stm32f4xx.h"
#include "Delay.h"
#include "Serial.h"
#include "mavlink.h"
#include "DSHOT.h"
#include "adc.h"
#include "string.h"
#include "wit_c_sdk.h"
#include "TIM.h"
#include "LED.h"
#include "control.h"

//@brief  打印 ADC 原始值和电压值（调试用）
//void ADC_PrintRaw(void)

//mavlink441~450，586.593油门值调试
//mavlink534~537目标姿态角，角速度调试
//serial 221,230
//// 添加：发送四个油门值到串口助手
//void SendThrottleValues(void)
// ==================== TIM6_DAC_IRQn 中断处理 (电池检测 - 1秒周期) ====================

// ==================== TIM7_IRQHandler 中断处理 (MAVLink发送 - 10ms周期) ====================
//MAVLink包括IMU,心跳包

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
	


    // 初始化 IMU
    UART5_Init();
    WitImu_Init();

    // 初始化 UART2 (MAVLink，含DMA)
    USART2_Init();
	    
    Serial_Printf("USART_Init\r\n");
    // 初始化 DSHOT
    DShot_Init();
	   
    Serial_Printf("DShot_Init\r\n");
    // 初始化 LED (PB6)
    LED_init();

    // 初始化 ADC
    ADC_Config();
    Serial_Printf("ADC_Config\r\n");

    // 启动定时器
    TIM7_Init(10000 - 1, 84 - 1);   // MAVLink 100Hz
    TIM6_Init(10000 - 1, 8400 - 1); // 电池 1Hz

    // ========== 主循环 ==========
    uint8_t channel;
    uint16_t value;
    uint8_t any_update;

    while (1)
    {
        // 处理油门命令队列
        any_update = 0;

        DSHOT_ENTER_CRITICAL();
        while (throttle_cmd_available_unsafe()) {
            // 使用非临界区版本，避免嵌套 EXIT_CRITICAL 过早恢复中断
            throttle_cmd_dequeue_unsafe(&channel, &value);
            current_throttle[channel] = value;
            any_update = 1;
        }
        DSHOT_EXIT_CRITICAL();

        // 串口调试油门更新
        if (serial_throttle_updated) {
            serial_throttle_updated = 0;
            any_update = 1;
        }

        // 有更新时更新DMA缓冲区
        if (any_update) {
            DShot_UpdateAllChannels();
        }

        // LED 1Hz闪烁处理 (与MAVLink心跳同频)
        LED_blink_handler();
    }
}


