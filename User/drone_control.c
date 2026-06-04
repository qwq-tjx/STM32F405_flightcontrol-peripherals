/**
 * @file drone_control.c
 * @brief 飞控算法库 - 角度外环(20Hz) + 角速度内环(100Hz)
 *
 * 调用时序（由 TIM7 ISR 调度，100Hz）：
 *   1. IMU_Data_update()               ← 100Hz, 将 IMU 数据同步到算法状态
 *   2. drone_control_angle_outer_loop() ← 20Hz,  在内环前（每 5 次调用 1 次）
 *   3. drone_control_rate_inner_loop()  ← 100Hz, 最后执行
 */

#include "drone_control.h"
#include <math.h>

#ifndef M_PI
#define M_PI  3.14159265358979323846f
#endif

// ============================================================
// 全局算法实例
// ============================================================

drone_control_algorithm_t gDroneControlAlgo;

// ============================================================
// 四元数工具函数
// ============================================================

void quaternion_multiply(const quaternion_t *a, const quaternion_t *b, quaternion_t *result)
{
    result->w = a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z;
    result->x = a->w * b->x + a->x * b->w + a->y * b->z - a->z * b->y;
    result->y = a->w * b->y - a->x * b->z + a->y * b->w + a->z * b->x;
    result->z = a->w * b->z + a->x * b->y - a->y * b->x + a->z * b->w;
}

void quaternion_inverse(const quaternion_t *q, quaternion_t *result)
{
    float norm_sq = q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z;
    if (norm_sq > 1e-12f) {
        float inv_norm_sq = 1.0f / norm_sq;
        result->w =  q->w * inv_norm_sq;
        result->x = -q->x * inv_norm_sq;
        result->y = -q->y * inv_norm_sq;
        result->z = -q->z * inv_norm_sq;
    } else {
        result->w = 1.0f;
        result->x = 0.0f;
        result->y = 0.0f;
        result->z = 0.0f;
    }
}

// ============================================================
// 欧拉角 <-> 四元数
// ============================================================

void euler_to_quaternion(const Eulerangle_t *euler, quaternion_t *quat)
{
    float cr = cosf(euler->roll  * 0.5f);
    float sr = sinf(euler->roll  * 0.5f);
    float cp = cosf(euler->pitch * 0.5f);
    float sp = sinf(euler->pitch * 0.5f);
    float cy = cosf(euler->yaw   * 0.5f);
    float sy = sinf(euler->yaw   * 0.5f);

    quat->w = cr * cp * cy + sr * sp * sy;
    quat->x = sr * cp * cy - cr * sp * sy;
    quat->y = cr * sp * cy + sr * cp * sy;
    quat->z = cr * cp * sy - sr * sp * cy;
}

void quaternion_to_euler(const quaternion_t *quat, Eulerangle_t *euler)
{
    float w = quat->w, x = quat->x, y = quat->y, z = quat->z;

    euler->roll = atan2f(2.0f * (w*x + y*z), 1.0f - 2.0f * (x*x + y*y));

    float sin_pitch = 2.0f * (w*y - z*x);
    if (sin_pitch > 1.0f)  sin_pitch = 1.0f;
    if (sin_pitch < -1.0f) sin_pitch = -1.0f;
    euler->pitch = asinf(sin_pitch);

    euler->yaw = atan2f(2.0f * (w*z + x*y), 1.0f - 2.0f * (y*y + z*z));
}

// ============================================================
// PID 控制器
// ============================================================

float PID_update(float error, PID_data_t *p, float dt)
{
    float output = 0.0f;

    output += p->Kp * error;                                           // 比例项（与 dt 无关）

    p->integrate += error * dt;                                        // 积分项：e * dt 累加
    if (p->integrate >  p->integrate_limit) p->integrate =  p->integrate_limit;
    if (p->integrate < -p->integrate_limit) p->integrate = -p->integrate_limit;
    output += p->Ki * p->integrate;

    float derivative = (error - p->last_error) / dt;                   // 微分项：差分 / dt
    output += p->Kd * derivative;

    p->last_error = error;
    return output;
}

// ============================================================
// 内环辅助函数：三轴 PID 计算参考角加速度
// ============================================================

static vector_t compute_omega_control(const vector_t *omega_ref, const vector_t *angular_velocity,
                                       const drone_control_param_t *param, float dt)
{
    vector_t output;

    float error_x = omega_ref->x - angular_velocity->x;                // 三轴角速度误差
    float error_y = omega_ref->y - angular_velocity->y;
    float error_z = omega_ref->z - angular_velocity->z;

    output.x = PID_update(error_x, &((drone_control_param_t *)param)->pidx, dt);
    output.y = PID_update(error_y, &((drone_control_param_t *)param)->pidy, dt);
    output.z = PID_update(error_z, &((drone_control_param_t *)param)->pidz, dt);

    return output;
}

// ============================================================
// 内环辅助函数：欧拉动力学 tau = J * alpha + omega x (J * omega)
// ============================================================

static vector_t compute_torque(vector_t alpha_ref, const vector_t *angular_velocity,
                                const drone_control_param_t *param)
{
    const float (*J)[3] = param->inertia;
    vector_t w = *angular_velocity;

    float Jw_x = J[0][0] * w.x + J[0][1] * w.y + J[0][2] * w.z;     // J * omega
    float Jw_y = J[1][0] * w.x + J[1][1] * w.y + J[1][2] * w.z;
    float Jw_z = J[2][0] * w.x + J[2][1] * w.y + J[2][2] * w.z;

    float Jax = J[0][0] * alpha_ref.x + J[0][1] * alpha_ref.y + J[0][2] * alpha_ref.z;  // J * alpha
    float Jay = J[1][0] * alpha_ref.x + J[1][1] * alpha_ref.y + J[1][2] * alpha_ref.z;
    float Jaz = J[2][0] * alpha_ref.x + J[2][1] * alpha_ref.y + J[2][2] * alpha_ref.z;

    float gx = w.y * Jw_z - w.z * Jw_y;                               // omega x (J * omega)
    float gy = w.z * Jw_x - w.x * Jw_z;
    float gz = w.x * Jw_y - w.y * Jw_x;

    vector_t torque;
    torque.x = Jax + gx;
    torque.y = Jay + gy;
    torque.z = Jaz + gz;
    return torque;
}

// ============================================================
// 内环辅助函数：混控分配（力矩 + 推力 → 四电机转速）
// ============================================================

static void allocate_motors(float thrust_ref, vector_t torque_ref, const drone_control_param_t *param,
                             const float *prev_motor_throttle, float *out_motor_throttle)
{
    float arm = param->arm_length;
    float c   = 0.70710678f * arm;                                     // 等效力臂 = arm / sqrt(2)
    float inv_4c  = 1.0f / (4.0f * c);
    float inv_4km = param->counter_torque_coeff * 0.25f;               // 1 / (4 * km)
    float F4 = 0.25f * thrust_ref;

    float thrust[4];                                                    // 四电机推力 (N)
    thrust[0] = F4 - torque_ref.x * inv_4c + torque_ref.y * inv_4c + torque_ref.z * inv_4km;
    thrust[1] = F4 + torque_ref.x * inv_4c + torque_ref.y * inv_4c - torque_ref.z * inv_4km;
    thrust[2] = F4 + torque_ref.x * inv_4c - torque_ref.y * inv_4c + torque_ref.z * inv_4km;
    thrust[3] = F4 - torque_ref.x * inv_4c - torque_ref.y * inv_4c - torque_ref.z * inv_4km;

    // 推力下标 → PWM 引脚映射: PC6→M1, PC7→M4, PC8→M2, PC9→M3
    static const uint8_t motor_to_pwm[4] = {0, 2, 3, 1};
    for (int i = 0; i < 4; i++) {
        float t = thrust[i];
        if (t < 0.0f) t = 0.0f;
        out_motor_throttle[motor_to_pwm[i]] = sqrtf(t * param->thrust_coeff);  // omega = sqrt(T * inv_kt)
    }
    (void)prev_motor_throttle;
}



// ============================================================
// 模块 1: 角度外环 —— 轴角误差法计算参考角速度（20Hz）
// ============================================================

void drone_control_angle_outer_loop(void)
{
    drone_control_param_t *param = &gDroneControlAlgo.param;

    // 当前姿态四元数直接从 IMU 实例读取，无需欧拉角转换
    quaternion_t q_c = gDroneControlAlgo.quaternion;

    // 目标欧拉角 → 四元数（仅此处需要转换）
    quaternion_t q_d;
    euler_to_quaternion(&gDroneControlAlgo.target_euler, &q_d);

    // 误差四元数 q_e = q_c^{-1} * q_d
    quaternion_t q_c_inv;
    quaternion_inverse(&q_c, &q_c_inv);

    quaternion_t q_e;
    quaternion_multiply(&q_c_inv, &q_d, &q_e);

    // 归一化
    float norm = sqrtf(q_e.w * q_e.w + q_e.x * q_e.x + q_e.y * q_e.y + q_e.z * q_e.z);
    if (norm > 1e-12f) {
        float inv_norm = 1.0f / norm;
        q_e.w *= inv_norm;  q_e.x *= inv_norm;
        q_e.y *= inv_norm;  q_e.z *= inv_norm;
    }

    // 提取轴角: 误差角 alpha_e = 2 * acos(q_e.w)
    float cos_half_alpha = q_e.w;
    if (cos_half_alpha > 1.0f)  cos_half_alpha = 1.0f;
    if (cos_half_alpha < -1.0f) cos_half_alpha = -1.0f;

    float half_alpha     = acosf(cos_half_alpha);
    float sin_half_alpha = sinf(half_alpha);

    float error_angle;
    vector_t error_axis;

    if (sin_half_alpha > 1e-10f) {
        float inv_sin = 1.0f / sin_half_alpha;
        error_angle = 2.0f * half_alpha;
        error_axis.x = q_e.x * inv_sin;
        error_axis.y = q_e.y * inv_sin;
        error_axis.z = q_e.z * inv_sin;
    } else {
        error_angle = 0.0f;
        error_axis.x = 0.0f;
        error_axis.y = 0.0f;
        error_axis.z = 0.0f;
    }

    // 映射到参考角速度: omega_ref = (error_angle / T) * error_axis
    float inv_T = 1.0f / param->T;
    gDroneControlAlgo.omega_ref.x = error_angle * error_axis.x * inv_T;
    gDroneControlAlgo.omega_ref.y = error_angle * error_axis.y * inv_T;
    gDroneControlAlgo.omega_ref.z = error_angle * error_axis.z * inv_T;
}

// ============================================================
// 模块 2: 角速度内环 —— PID + 动力学 + 混控（100Hz）
// ============================================================

void drone_control_rate_inner_loop(void)
{
    const float dt = 0.01f;  // 100Hz 更新周期
    drone_control_param_t *param = &gDroneControlAlgo.param;

    // 1. 角速度 PID 内环: omega_ref vs angular_velocity -> alpha_ref
    vector_t alpha_ref = compute_omega_control(
        &gDroneControlAlgo.omega_ref,
        &gDroneControlAlgo.angular_velocity,
        param, dt);

    // 2. 欧拉动力学: alpha_ref -> torque
    vector_t torque = compute_torque(alpha_ref, &gDroneControlAlgo.angular_velocity, param);

    // 3. 从 target_throttle 直接获取总推力参考（开环油门）
    float thrust_ref = gDroneControlAlgo.target_throttle;
    //if (thrust_ref < 1e-6f) {
        //thrust_ref = param->mass * 9.80665f;                           // 默认悬停推力
    //}

    // 4. 混控分配: torque + thrust -> 四电机转速
    allocate_motors(thrust_ref, torque, param, gDroneControlAlgo.motor_throttle,
                    gDroneControlAlgo.motor_throttle);
}

// ============================================================
// 初始化（PID 全零，物理参数预设）
// ============================================================

void drone_control_algorithm_init(void)
{
    drone_control_param_t *p = &gDroneControlAlgo.param;

    gDroneControlAlgo.quaternion.w = 1.0f;  gDroneControlAlgo.quaternion.x = 0.0f;
    gDroneControlAlgo.quaternion.y = 0.0f;  gDroneControlAlgo.quaternion.z = 0.0f;

    gDroneControlAlgo.euler.roll  = 0.0f;  gDroneControlAlgo.euler.pitch = 0.0f;  gDroneControlAlgo.euler.yaw = 0.0f;
    gDroneControlAlgo.angular_velocity.x = 0.0f;  gDroneControlAlgo.angular_velocity.y = 0.0f;  gDroneControlAlgo.angular_velocity.z = 0.0f;
    gDroneControlAlgo.linear_acceleration.x = 0.0f;  gDroneControlAlgo.linear_acceleration.y = 0.0f;
    gDroneControlAlgo.linear_acceleration.z = 0.0f;
    gDroneControlAlgo.altitude = 0.0f;

    gDroneControlAlgo.target_euler.roll  = 0.0f;  gDroneControlAlgo.target_euler.pitch = 0.0f;  gDroneControlAlgo.target_euler.yaw = 0.0f;
    gDroneControlAlgo.target_throttle = 0.0f;
    gDroneControlAlgo.omega_ref.x = 0.0f;  gDroneControlAlgo.omega_ref.y = 0.0f;  gDroneControlAlgo.omega_ref.z = 0.0f;

    for (int i = 0; i < 4; i++) {
        gDroneControlAlgo.motor_throttle[i] = 0.0f;
    }

    // PID 参数先全零，实际飞行时通过上位机调参写入
    // 推荐起始值（连续时间型，内环 dt=0.01 已计入 PID_update）：
    //   roll/pitch: Kp=20, Ki=200, Kd=0, integrate_limit=0.15
    //   yaw:        Kp=25, Ki=250, Kd=0, integrate_limit=0.15
    p->pidx.Kp = 0.0f;  p->pidx.Ki = 0.0f;  p->pidx.Kd = 0.0f;
    p->pidx.integrate = 0.0f;  p->pidx.integrate_limit = 0.0f;  p->pidx.last_error = 0.0f;
    p->pidy.Kp = 0.0f;  p->pidy.Ki = 0.0f;  p->pidy.Kd = 0.0f;
    p->pidy.integrate = 0.0f;  p->pidy.integrate_limit = 0.0f;  p->pidy.last_error = 0.0f;
    p->pidz.Kp = 0.0f;  p->pidz.Ki = 0.0f;  p->pidz.Kd = 0.0f;
    p->pidz.integrate = 0.0f;  p->pidz.integrate_limit = 0.0f;  p->pidz.last_error = 0.0f;

    // 姿态收敛时间常数 T 先置零，推荐起始值 T=0.5s
    p->T = 0.0f;
    p->arm_length           = 0.425f;
    p->thrust_coeff         = 1.7129458e8f;
    p->counter_torque_coeff = 4.1795f;
    p->mass                 = 4.5f;
    p->inertia[0][0] = 0.015f;   p->inertia[0][1] = 0.0f;   p->inertia[0][2] = 0.0f;
    p->inertia[1][0] = 0.0f;    p->inertia[1][1] = 0.015f;  p->inertia[1][2] = 0.0f;
    p->inertia[2][0] = 0.0f;    p->inertia[2][1] = 0.0f;    p->inertia[2][2] = 0.025f;
}

// ============================================================
// IMU 数据同步: 将原始传感器数据转换为算法所需的 SI 单位
// ============================================================
// 调用频率: 100Hz, 每次飞控内环迭代前必须调用
//
// 单位转换说明:
//   WitImuData_t.acc[3]   → g (重力加速度) → m/s²
//   WitImuData_t.gyro[3]  → deg/s           → rad/s
//   WitImuData_t.angle[3] → deg             → rad
//   WitImuData_t.quat[4]  → 归一化值         直接复制
//   WitImuData_t.altitude → m               直接复制

void IMU_Data_update(const WitImuData_t *imu)
{
    static const float DEG_TO_RAD = M_PI / 180.0f;   /* 度转弧度 */

    /* ---- 1. 四元数: 直接复制 ---- */
    gDroneControlAlgo.quaternion.w = imu->quat[0];
    gDroneControlAlgo.quaternion.x = imu->quat[1];
    gDroneControlAlgo.quaternion.y = imu->quat[2];
    gDroneControlAlgo.quaternion.z = imu->quat[3];

    /* ---- 2. 欧拉角: deg → rad ---- */
    gDroneControlAlgo.euler.roll  = imu->angle[0] * DEG_TO_RAD;
    gDroneControlAlgo.euler.pitch = imu->angle[1] * DEG_TO_RAD;
    gDroneControlAlgo.euler.yaw   = imu->angle[2] * DEG_TO_RAD;

    /* ---- 3. 角速度: deg/s → rad/s ---- */
    gDroneControlAlgo.angular_velocity.x = imu->gyro[0] * DEG_TO_RAD;
    gDroneControlAlgo.angular_velocity.y = imu->gyro[1] * DEG_TO_RAD;
    gDroneControlAlgo.angular_velocity.z = imu->gyro[2] * DEG_TO_RAD;

    /* ---- 4. 加速度: g → m/s² ---- */
    gDroneControlAlgo.linear_acceleration.x = imu->acc[0] * 9.80665f;
    gDroneControlAlgo.linear_acceleration.y = imu->acc[1] * 9.80665f;
    gDroneControlAlgo.linear_acceleration.z = imu->acc[2] * 9.80665f;

    /* ---- 5. 高度: 直接复制 ---- */
    gDroneControlAlgo.altitude = imu->altitude;
}
