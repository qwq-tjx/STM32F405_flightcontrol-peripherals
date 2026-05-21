#ifndef __IMU_SIMULATOR_H
#define __IMU_SIMULATOR_H

#include "stm32f4xx.h"

#define GRAVITY_MSS    9.80665f   // 重力加速度 (m/s^2)
// IMU 数据结构体
typedef struct {
    float accel_x;   // 加速度 X (m/s^2)
    float accel_y;   // 加速度 Y (m/s^2)
    float accel_z;   // 加速度 Z (m/s^2)
    float gyro_x;    // 陀螺仪 X (rad/s)
    float gyro_y;    // 陀螺仪 Y (rad/s)
    float gyro_z;    // 陀螺仪 Z (rad/s)
    float mag_x;     // 磁力计 X (uT)
    float mag_y;     // 磁力计 Y (uT)
    float mag_z;     // 磁力计 Z (uT)
    int16_t temperature;  // 温度 (cdegC, 即 0.01 度)
} imu_data_t;

// 初始化 IMU 模拟器
void imu_simulator_init(void);

// 更新 IMU 数据（基于正弦波模拟）
void imu_simulator_update(void);

// 获取当前 IMU 数据
imu_data_t* imu_simulator_get_data(void);

// 获取时间偏移（毫秒）
uint32_t imu_simulator_get_time_us(void);

#endif
