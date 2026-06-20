/**
 * @file drone_control.h
 * @brief 飞控算法库 - 角度外环(20Hz) + 角速度内环(100Hz)
 *
 * 本文件仅包含算法相关的类型定义与函数声明，
 * 数据调度（IMU读取、时序控制、通信等）由外部模块负责。
 */

#ifndef __DRONE_CONTROL_H__
#define __DRONE_CONTROL_H__

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "wit_c_sdk.h"          /* WitImuData_t 结构体定义 */

// ============================================================
// 浮点 NaN/Inf 防护工具
// ============================================================
// NaN 满足 (v != v)，Inf 满足 (v*0.0f != 0.0f)
// 两者任一为真时 is_good_float 返回 false
static inline int is_good_float(float v)
{
    return (v == v) && (v * 0.0f == 0.0f);
}

// ============================================================
// 油门单位转换: 算法输出 rad/s → DSHOT 原始值 (0~4095)
// ============================================================
// 典型 6S 电机空载转速 ≈ 800KV × 24V ≈ 19200 RPM ≈ 2011 rad/s
// 带桨最高有效转速约为空载的 60%，即 1200 rad/s
// 转换系数 = 4095 / 1200 ≈ 3.4
#define MAX_MOTOR_RAD_S     1200.0f   /* 电机最大有效角速度 (rad/s) */
#define RAD_S_TO_DSHOT      3.4125f   /* 转换系数: 4095 / 1200 */


// ============================================================
// 基础类型定义
// ============================================================

typedef struct {
    float w;
    float x;
    float y;
    float z;
} quaternion_t;

typedef struct {
    float x;
    float y;
    float z;
} vector_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} Eulerangle_t;

// ============================================================
// 四元数工具函数
// ============================================================

void quaternion_multiply(const quaternion_t *a, const quaternion_t *b, quaternion_t *result);
void euler_to_quaternion(const Eulerangle_t *euler, quaternion_t *quat);
void quaternion_to_euler(const quaternion_t *quat, Eulerangle_t *euler);
void quaternion_inverse(const quaternion_t *q, quaternion_t *result);
void quaternion_to_rotation_matrix(const quaternion_t *q, float R[3][3]);

// ============================================================
// 三维 Kalman 滤波 - 高度估计
// ============================================================

typedef struct {
    float z;            // 融合高度 (m)
    float vz;           // 垂直速度 (m/s)
    float ba;           // 加速度计 Z 轴偏置 (m/s?)

    float P[3][3];      // 协方差矩阵

    // 可调参数（传感器可靠性决定了以下噪声方差的相对大小）
    float q_z;          // 位置过程噪声 (accel 动态可靠性中等)
    float q_vz;         // 速度过程噪声
    float q_ba;         // 偏置过程噪声 (accel 偏置漂移率)
    float r_baro;       // 气压计观测噪声方差 (静态中等 / 动态最差)
    float r_laser;      // 激光 TOF 观测噪声方差 (静态/动态均最高)
} kalman_3d_t;

// ============================================================
// PID 控制器
// ============================================================

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integrate;   //积分累积
    float integrate_limit;  //积分限幅
    float last_error;  //上次误差
} PID_data_t;

// ============================================================
// 控制参数结构体
// ============================================================

typedef struct {
    float T;                             // 姿态收敛时间常数 (s)
    PID_data_t pidx;                     // X轴(横滚)角速度 PID
    PID_data_t pidy;                     // Y轴(俯仰)角速度 PID
    PID_data_t pidz;                     // Z轴(偏航)角速度 PID

    float arm_length;                    // 轴距: 中心到电机距离 (m)
    float thrust_coeff;                  // 1/kt, 存倒数
    float counter_torque_coeff;          // 1/km, 存倒数
    float inertia[3][3];                 // 惯性张量 (kg.m^2)
    float mass;                          // 无人机总质量 (kg)
    vector_t optflow_position;           // 光流模块在机体坐标系下的安装位置 (m)

    // 速度与高度控制 PID
    PID_data_t pid_vx;                   // 世界 X 轴水平速度 PID (加速度输出)
    PID_data_t pid_vy;                   // 世界 Y 轴水平速度 PID (加速度输出)
    PID_data_t pid_vz;                   // [未使用] 垂直速度已合并到 pid_alt 的 D 项
    PID_data_t pid_alt;                  // 高度 PID (单级, 位置误差 → 加速度)
                                         // PID_update 的 D 项根据垂直速度提供阻尼
    float max_tilt_angle;                // 最大倾斜角 (rad)，推荐 0.4 (≈23°)
} drone_control_param_t;

// ============================================================
// 全局算法状态（供外部读取/写入）
// ============================================================

typedef struct {
    // IMU 原始数据（由外部通过 IMU_Data_update 写入）
    quaternion_t quaternion;           // 当前四元数
    Eulerangle_t euler;                // 当前欧拉角 (rad)
    vector_t angular_velocity;         // 当前角速度 (rad/s)
    vector_t linear_acceleration;      // 当前加速度 (m/s^2)
    float altitude;                    // 当前高度 (m)

    // 光流与激光数据（由外部通过 Optflow_Data_update 写入）
    float flow_vel_x;                  // 光流原始X速度 (m/s, 机体坐标系)
    float flow_vel_y;                  // 光流原始Y速度 (m/s, 机体坐标系)
    float laser_range;                 // 激光测距距离 (m)
    uint16_t flow_quality;             // 光流质量 (0~100, >=60 可信)

    // 传感器融合数据（由 Optflow_Data_update 内部计算）
    vector_t velocity;                 // 机体速度 (m/s)
    float height;                      // 无人机高度 (m)

    quaternion_t reliable_quat;        // 可靠姿态四元数（外环从欧拉角转换得出，IMU 原始四元数异常替代方案）
    kalman_3d_t kf_height;             // 高度 KF 状态

    // 目标姿态与油门（由外部根据 MAVLink / 遥控器数据写入）
    Eulerangle_t target_euler;         // 目标欧拉角 (rad)
    float target_throttle;             // 目标总推力 (N)，开环测试用，直接传入混控分配

    // 目标速度与高度（由外部根据 MAVLink / 遥控器数据写入）
    vector_t target_velocity;          // 目标速度 (m/s, 世界坐标系 NED/ENU)
    float target_altitude;             // 目标高度 (m)

    // 世界坐标系速度（由 drone_control_velocity_altitude_control 从机体速度转换得出）
    vector_t velocity_world;           // 世界坐标系速度 (m/s)

    // 角度外环输出（由 drone_control_angle_outer_loop 写入）
    vector_t omega_ref;                // 参考角速度 (rad/s)

    // 角速度内环输出（由 drone_control_rate_inner_loop 写入）
    float motor_throttle[4];           // 四电机油门值 (rad/s)

    // 控制参数（可由外部实时调参）
    drone_control_param_t param;
} drone_control_algorithm_t;

// ============================================================
// 全局实例
// ============================================================

extern drone_control_algorithm_t gDroneControlAlgo;

// ============================================================
// 算法模块入口（2 个模块，按调用频率排列）
// ============================================================

// 模块 1: 角度外环 —— 基于 target_euler 与 euler 的轴角误差，计算 omega_ref（20Hz）
void drone_control_angle_outer_loop(void);

// 模块 2: 角速度内环 —— omega_ref 与 angular_velocity 的 PID 控制 + 动力学 + 混控（100Hz）
void drone_control_rate_inner_loop(void);

// 初始化算法参数（PID 全零，物理参数预设）
void drone_control_algorithm_init(void);

// IMU 数据同步: 将 WitImuData_t (deg/s, deg, g) → gDroneControlAlgo (rad/s, rad, m/s2)
// 调用频率: 100Hz, 必须在 angle_outer_loop / rate_inner_loop 之前调用
void IMU_Data_update(const WitImuData_t *imu);

// 光流数据同步: 将已转换的传感器数据写入算法实例
// 调用频率: 100Hz, 在 IMU_Data_update 之后调用
// 注意: flow_vel_x/y 需在外部完成单位转换 (m/s) 和坐标系对齐,
//       此处仅做存储, 不再做任何单位/坐标转换
// 参数:
//   vx       - 光流 X 速度 (m/s, 机体坐标系)
//   vy       - 光流 Y 速度 (m/s, 机体坐标系)
//   range    - TOF 激光测距 (m)
//   quality  - 光流质量 (0~100, MTF-01 传感器上报)
void Optflow_Data_update(float vx, float vy, float range, uint16_t quality);

// 速度与高度融合: 基于光流 + IMU 补偿计算机体速度与高度
// 调用频率: 100Hz, 在 Optflow_Data_update 之后、控制环之前调用
void velocity_height_Data_update(void);

// 模块 3: 速度与高度控制 —— 世界坐标系 PID → 期望加速度 → 姿态角 + 推力（10Hz）
// 依赖: velocity_world / height 必须在调用前更新（由 velocity_height_Data_update 保证）
// 输出: 写入 target_euler.roll/pitch 和 target_throttle
// 航向保持（yaw 不受此模块影响，由遥控器/上位机直接指定）
void drone_control_velocity_altitude_control(void);

#endif /* __DRONE_CONTROL_H__ */
