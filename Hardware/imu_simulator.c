#include "imu_simulator.h"
#include "Delay.h"
#include <math.h>
// 定义数学常量
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// IMU 数据结构体实例
static imu_data_t imu_data = {0};
static uint32_t start_time_ms = 0;

// 标准范围参数
#define GRAVITY_MSS    9.80665f   // 重力加速度 (m/s^2)
#define GYRO_SCALE     0.0174533f // 度转弧度 (pi/180)
#define MAG_SCALE      1.0f       // 磁力计缩放

// 模拟参数
#define SIM_AMPLITUDE_ACCEL  2.0f     // 加速度振幅 (±2g)
#define SIM_AMPLITUDE_GYRO   1.0f     // 陀螺仪振幅 (±1 rad/s ≈ ±57 deg/s)
#define SIM_AMPLITUDE_MAG    50.0f    // 磁力计振幅 (±50 uT)
#define SIM_FREQUENCY_ACCEL  0.5f     // 加速度变化频率 (Hz)
#define SIM_FREQUENCY_GYRO   0.3f     // 陀螺仪变化频率 (Hz)
#define SIM_FREQUENCY_MAG    0.2f     // 磁力计变化频率 (Hz)

void imu_simulator_init(void)
{
    start_time_ms = delay_ms_count_get();
    
    // 初始化所有数据为0
    imu_data.accel_x = 0;
    imu_data.accel_y = 0;
    imu_data.accel_z = GRAVITY_MSS;  // Z轴初始为重力加速度
    imu_data.gyro_x = 0;
    imu_data.gyro_y = 0;
    imu_data.gyro_z = 0;
    imu_data.mag_x = 0;
    imu_data.mag_y = 0;
    imu_data.mag_z = 0;
    imu_data.temperature = 2500;  // 25.00 度
}

static uint32_t update_counter = 0;

void imu_simulator_update(void)
{
	  update_counter++;
    float time_sec = update_counter * 0.01f;  // 每 10ms 更新一次
	
 
    
  // 计算正弦波值
    float sin_accel = sinf(2.0f * M_PI * SIM_FREQUENCY_ACCEL * time_sec);
    float sin_accel_y = sinf(2.0f * M_PI * SIM_FREQUENCY_ACCEL * time_sec + M_PI / 4);
    float sin_accel_z = sinf(2.0f * M_PI * SIM_FREQUENCY_ACCEL * time_sec + M_PI / 2);
    
    float sin_gyro = sinf(2.0f * M_PI * SIM_FREQUENCY_GYRO * time_sec);
    float sin_gyro_y = sinf(2.0f * M_PI * SIM_FREQUENCY_GYRO * time_sec + M_PI / 3);
    float sin_gyro_z = sinf(2.0f * M_PI * SIM_FREQUENCY_GYRO * time_sec + 2 * M_PI / 3);
    
    float sin_mag = sinf(2.0f * M_PI * SIM_FREQUENCY_MAG * time_sec);
    float sin_mag_y = sinf(2.0f * M_PI * SIM_FREQUENCY_MAG * time_sec + M_PI / 6);
    float sin_mag_z = sinf(2.0f * M_PI * SIM_FREQUENCY_MAG * time_sec + M_PI / 3);
    
    // 更新加速度计数据 (振幅 ±2g，加上重力分量)
    imu_data.accel_x = SIM_AMPLITUDE_ACCEL * GRAVITY_MSS * sin_accel;
    imu_data.accel_y = SIM_AMPLITUDE_ACCEL * GRAVITY_MSS * sin_accel_y;
    imu_data.accel_z = GRAVITY_MSS + SIM_AMPLITUDE_ACCEL * GRAVITY_MSS * sin_accel_z;
    
    // 更新陀螺仪数据 (振幅 ±1 rad/s)
    imu_data.gyro_x = SIM_AMPLITUDE_GYRO * sin_gyro;
    imu_data.gyro_y = SIM_AMPLITUDE_GYRO * sin_gyro_y;
    imu_data.gyro_z = SIM_AMPLITUDE_GYRO * sin_gyro_z;
    
    // 更新磁力计数据 (振幅 ±50 uT)
    imu_data.mag_x = SIM_AMPLITUDE_MAG * sin_mag;
    imu_data.mag_y = SIM_AMPLITUDE_MAG * sin_mag_y;
    imu_data.mag_z = SIM_AMPLITUDE_MAG * sin_mag_z;
    
    // 模拟温度轻微变化
    imu_data.temperature = 2500 + (int16_t)(100 * sinf(0.1f * time_sec));
}

imu_data_t* imu_simulator_get_data(void)
{
    return &imu_data;
}

uint32_t imu_simulator_get_time_us(void)
{
    return (delay_ms_count_get() - start_time_ms) * 1000;
}
