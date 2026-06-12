#pragma once
// control.h - 目标值存储 & 控制模式声明
// 变量定义在 mavlink.c 中，此处只做 extern 声明

#include <stdint.h>

// ===== 控制模式枚举 =====
typedef enum {
    CTRL_MODE_DISABLED = 0,  // 无控制，使用串口油门 (name="DISA")
    CTRL_MODE_ATTITUDE = 1,  // 姿态控制，外环角度+内环角速度 (name="ATTI")
    CTRL_MODE_RATE     = 2,  // 角速度控制，仅内环 (name="RATE")
    CTRL_MODE_VH       = 3,  // 速度+高度闭环 (name="VELH")
} control_mode_t;

// ===== 目标值 (MAVLink 接收中断更新) =====
// data[0..2] → target_angle[0..2]  (弧度, ±π)
// data[3..5] → target_gyro[0..2]   (rad/s, ±34.9)
extern volatile float  target_angle[3];      // roll, pitch, yaw
extern volatile float  target_gyro[3];       // x, y, z
extern volatile float  target_throttle;      // 目标总推力 (N)
extern volatile float  target_velocity[3];   // 目标速度 (m/s, 世界坐标系)
extern volatile float  target_altitude;      // 目标高度 (m)
extern volatile uint8_t control_mode;

