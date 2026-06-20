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
#include "drone_control.h"      /* 飞控算法库 */


// 定义圆周率常量
#ifndef M_PI
#define M_PI  3.14159265358979323846f
#endif

// DMA缓冲区声明已在DSHOT.h中

/******************************************************************************
 *                         串口调试功能使用说明

 *
 * ──────────────────────────────────────────────────────────────────────────
 *  一、USART1 串口油门直驱协议
 * ──────────────────────────────────────────────────────────────────────────
 *  格式: @t1,t2,t3,t4\r\n   (4个电机油门值，逗号分隔，范围 0~4095)
 *  示例: 在串口助手中发送  @1000,1000,800,800  + 回车
 *
 *  飞控收到后会回显确认:
 *    [THR] SER: 1000 1000 800 800
 *              BIN: 0000001111101000 0000001111101000 0000001100100000 0000001100100000
 *
 *  说明:
 *    - 回车 \r 会被忽略，必须 \n 结束帧
 *    - 超范围值自动限幅到 4095
 *    - 实现位置: Hardware/Serial.c → Serial_ParseThrottle()
 *    - 无需任何上位机软件，任何串口助手均可使用
 *
 * ──────────────────────────────────────────────────────────────────────────
 *  二、ADC 电池电压调试打印
 * ──────────────────────────────────────────────────────────────────────────
 *  函数: ADC_PrintRaw()    (定义于 Hardware/adc.c:217)
 *  调用: 在 main.c 主循环中添加以下代码即可每周期打印:
 *          ADC_PrintRaw();  // 输出 ADC原始值、VDDA电压、电池电压
 *
 *  输出示例:
 *    ADC Raw: 2048, VDDA: 3.296V, Battery: 22.13V
 *
 *  注意: 调用频率过高会阻塞 USART1，建议加延时或降频打印。
 *
 * ──────────────────────────────────────────────────────────────────────────
 *  三、已禁用的串口阻塞打印 (P0修复)
 * ──────────────────────────────────────────────────────────────────────────
 *  由于 ISR 上下文中的阻塞 Serial_Printf 会抢占 UART5，导致 IMU 传感器
 *  丢包和飞控失控，以下打印已被注释禁用。如需调试，请在主循环或低优
 *  先级 ISR 中重新启用，绝对不要在 USART2_IRQ 或 UART5_IRQ 中解注。
 *
 *  ┌───────┬──────────────┬──────────────────────────────────────────────┐
 *  │ 文件  │   行号       │  原打印内容                                  │
 *  ├───────┼──────────────┼──────────────────────────────────────────────┤
 *  │ mavlink_c/mavlink.c  │                                              │
 *  │       │ 532          │ Serial_Printf("[THR] throttle:%.2f N")       │
 *  │       │ 538          │ Serial_Printf("[CTRL] DISABLED")             │
 *  │       │ 555-556      │ Serial_Printf("[RATE] gyr(deg/s):...")       │
 *  │       │ 580-582      │ Serial_Printf("[ATTI] ang(deg):...")         │
 *  │       │ 607-611      │ Serial_Printf("[PIDP] T=...")                │
 *  │       │ 666,674      │ Serial_Printf("[THR] SERVO ch%d=%d")         │
 *  ├───────┼──────────────┼──────────────────────────────────────────────┤
 *  │ Hardware/TIM.c       │                                              │
 *  │       │ 297-298      │ Serial_Printf("[DEBUG] target_throttle=...") │
 *  └───────┴──────────────┴──────────────────────────────────────────────┘
 *
 *  重新启用方法: 取消对应行的 // 注释即可。建议只在 USART1(pri=3) 的
 *  ISR 上下文或主循环中使用，避免高优先级 ISR 因打印卡死。
 * ──────────────────────────────────────────────────────────────────────────
 ******************************************************************************/

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
    
    // 初始化 UART3 (MTF-01 光流传感器)
    UART3_Init();
    Serial_Printf("UART3_Init\r\n");
	    
    Serial_Printf("USART_Init\r\n");
    // 初始化 DSHOT
    DShot_Init();
	   
    Serial_Printf("DShot_Init\r\n");
    // 初始化 LED (PB6)
    LED_init();

    // 初始化 ADC
    ADC_Config();
    Serial_Printf("ADC_Config\r\n");

    // 飞控算法初始化 (PID 全零，物理参数预设，上电仅此一次)
    drone_control_algorithm_init();
    Serial_Printf("DroneControl_Init\r\n");

    // 启动定时器
    TIM7_Init(10000 - 1, 84 - 1);   // 飞控 + MAVLink 100Hz
    TIM6_Init(10000 - 1, 8400 - 1); // 电池 1Hz

    // ========== 主循环 ==========
    uint8_t channel;
    uint16_t value;
    uint8_t any_update;

    while (1)
    {
        any_update = 0;

        /* 非 DISABLED 模式下，飞控 PID 输出独占电机控制权；
           仅在 DISABLED 模式下才允许 MAVLink 手动油门命令 (防止覆盖 PID 输出) */
        if (control_mode == CTRL_MODE_DISABLED) {
            DSHOT_ENTER_CRITICAL();
            while (throttle_cmd_available_unsafe()) {
                throttle_cmd_dequeue_unsafe(&channel, &value);
                current_throttle[channel] = value;
                any_update = 1;
            }
            DSHOT_EXIT_CRITICAL();
        }

        // 串口调试油门更新 (始终允许，DISABLED 下手动控制可用)
        if (serial_throttle_updated) {
            serial_throttle_updated = 0;
            any_update = 1;
        }

        // 飞控算法油门输出更新 (TIM7 ISR 在 control_mode != DISABLED 时写入)
        if (drone_throttle_updated) {
            drone_throttle_updated = 0;
            any_update = 1;
        }

        // 有更新时刷新 DMA 缓冲区
        if (any_update) {
            DShot_UpdateAllChannels();
        }

        // LED 1Hz闪烁处理 (与MAVLink心跳同频)
        LED_blink_handler();
    }
}


