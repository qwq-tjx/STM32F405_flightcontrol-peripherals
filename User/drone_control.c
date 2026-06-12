/**
 * @file drone_control.c
 * @brief 飞控算法库 - 速度/高度外环(10Hz) + 角度外环(20Hz) + 角速度内环(100Hz)
 *
 * 调用时序（由 TIM7 ISR 调度，100Hz）：
 *   1. IMU_Data_update()                            ← 100Hz, 将 IMU 数据同步到算法状态
 *   2. drone_control_velocity_altitude_control()     ← 10Hz,  速度高度外环（每 10 次调用 1 次）
 *   3. drone_control_angle_outer_loop()              ← 20Hz,  角度外环（每 5 次调用 1 次）
 *   4. drone_control_rate_inner_loop()               ← 100Hz, 角速度内环（最后执行）
 */

#include "drone_control.h"
#include <math.h>

// 无头模式: 定义后航向角不受外环控制，由内环直接接收遥控器指令
// 注释掉此行则恢复航向角受外环角度闭环控制
#define DRONE_HEADLESS

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
// 四元数 → 旋转矩阵 (大地→机体: v_body = R @ v_earth)
// ============================================================

void quaternion_to_rotation_matrix(const quaternion_t *q, float R[3][3])
{
    float w = q->w, x = q->x, y = q->y, z = q->z;

    float xx = x * x, yy = y * y, zz = z * z;
    float xy = x * y, xz = x * z, yz = y * z;
    float wx = w * x, wy = w * y, wz = w * z;

    R[0][0] = 1.0f - 2.0f * (yy + zz);
    R[0][1] = 2.0f * (xy - wz);
    R[0][2] = 2.0f * (xz + wy);

    R[1][0] = 2.0f * (xy + wz);
    R[1][1] = 1.0f - 2.0f * (xx + zz);
    R[1][2] = 2.0f * (yz - wx);

    R[2][0] = 2.0f * (xz - wy);
    R[2][1] = 2.0f * (yz + wx);
    R[2][2] = 1.0f - 2.0f * (xx + yy);
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
    float F4 = 0.025f * thrust_ref;  //地面站控件端不知道出了什么问题，发送的力比期望大10倍，这里补偿回来

    float thrust[4];                                                    // 四电机推力 (N)
    thrust[0] = F4 - torque_ref.x * inv_4c + torque_ref.y * inv_4c + torque_ref.z * inv_4km;
    thrust[1] = F4 + torque_ref.x * inv_4c + torque_ref.y * inv_4c - torque_ref.z * inv_4km;
    thrust[2] = F4 + torque_ref.x * inv_4c - torque_ref.y * inv_4c + torque_ref.z * inv_4km;
    thrust[3] = F4 - torque_ref.x * inv_4c - torque_ref.y * inv_4c - torque_ref.z * inv_4km;

    // 推力下标 → PWM 引脚映射: PC6→M2, PC7→M3, PC8→M1, PC9→M4
    static const uint8_t motor_to_pwm[4] = {2, 0, 1, 3};
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

    /* ---- 守卫: T 未配置或过小时跳过外环，避免 1/0 → NaN 导致电机失控 ---- */
    if (param->T <= 1e-3f) {
        gDroneControlAlgo.omega_ref.x = 0.0f;
        gDroneControlAlgo.omega_ref.y = 0.0f;
        gDroneControlAlgo.omega_ref.z = 0.0f;
        return;
    }

    // IMU 四元数异常，改由欧拉角转换得到当前姿态四元数，存入 reliable_quat
    euler_to_quaternion(&gDroneControlAlgo.euler, &gDroneControlAlgo.reliable_quat);

    // 目标欧拉角 → 四元数（仅此处需要转换）
    quaternion_t q_d;
    euler_to_quaternion(&gDroneControlAlgo.target_euler, &q_d);

    // 误差四元数 q_e = q_c^{-1} * q_d
    quaternion_t q_c_inv;
    quaternion_inverse(&gDroneControlAlgo.reliable_quat, &q_c_inv);

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
#ifdef DRONE_HEADLESS
    gDroneControlAlgo.omega_ref.z = 0.0f;  // 无头模式: 航向不受外环控制
#else
    gDroneControlAlgo.omega_ref.z = error_angle * error_axis.z * inv_T;
#endif
}

// ============================================================
// 模块 2: 角速度内环 —— PID + 动力学 + 混控（100Hz）
// ============================================================

void drone_control_rate_inner_loop(void)
{
    const float dt = 0.01f;  // 100Hz 更新周期
    drone_control_param_t *param = &gDroneControlAlgo.param;

    // 0. 传感器融合（速度/高度 KF）：与内环同步 100Hz 更新
    velocity_height_Data_update();

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

// KF 前向声明（实现在文件末尾，但 algorithm_init 需要调用 kalman_3d_init）
static void kalman_3d_init(kalman_3d_t *kf, float init_z);
static void kalman_3d_predict(kalman_3d_t *kf, float acc_z_world);
static void kalman_3d_update(kalman_3d_t *kf, float z_meas, float r);

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

    gDroneControlAlgo.flow_vel_x = 0.0f;  gDroneControlAlgo.flow_vel_y = 0.0f;
    gDroneControlAlgo.laser_range = 0.0f;
    gDroneControlAlgo.flow_quality = 0;
    gDroneControlAlgo.velocity.x = 0.0f;  gDroneControlAlgo.velocity.y = 0.0f;  gDroneControlAlgo.velocity.z = 0.0f;
    gDroneControlAlgo.height = 0.0f;

    gDroneControlAlgo.target_euler.roll  = 0.0f;  gDroneControlAlgo.target_euler.pitch = 0.0f;  gDroneControlAlgo.target_euler.yaw = 0.0f;
    gDroneControlAlgo.target_throttle = 0.0f;
    gDroneControlAlgo.target_velocity.x = 0.0f;  gDroneControlAlgo.target_velocity.y = 0.0f;  gDroneControlAlgo.target_velocity.z = 0.0f;
    gDroneControlAlgo.target_altitude = 0.0f;
    gDroneControlAlgo.velocity_world.x = 0.0f;  gDroneControlAlgo.velocity_world.y = 0.0f;  gDroneControlAlgo.velocity_world.z = 0.0f;
    gDroneControlAlgo.omega_ref.x = 0.0f;  gDroneControlAlgo.omega_ref.y = 0.0f;  gDroneControlAlgo.omega_ref.z = 0.0f;

    for (int i = 0; i < 4; i++) {
        gDroneControlAlgo.motor_throttle[i] = 0.0f;
    }

    // PID 参数先全零，实际飞行时通过上位机调参写入
    // 推荐起始值（连续时间型，内环 dt=0.01 已计入 PID_update）：
    //   roll/pitch: Kp=20, Ki=200, Kd=0, integrate_limit=0.15
    //   yaw:        Kp=25, Ki=250, Kd=0, integrate_limit=0.15
	//实际对带宽要求似乎没那么高
    p->pidx.Kp = 24.0f;  p->pidx.Ki = 0.0f;  p->pidx.Kd = 0.0f;
    p->pidx.integrate = 0.0f;  p->pidx.integrate_limit = 0.0f;  p->pidx.last_error = 0.0f;
    p->pidy.Kp = 24.0f;  p->pidy.Ki = 0.0f;  p->pidy.Kd = 0.0f;
    p->pidy.integrate = 0.0f;  p->pidy.integrate_limit = 0.0f;  p->pidy.last_error = 0.0f;
    p->pidz.Kp = 3.0f;  p->pidz.Ki = 0.0f;  p->pidz.Kd = 0.0f;
    p->pidz.integrate = 0.0f;  p->pidz.integrate_limit = 0.0f;  p->pidz.last_error = 0.0f;

    // 速度与高度 PID 参数（推荐起始值，飞控上电后可通过上位机调参覆盖）
    //   水平速度: Kp=2.0, Ki=0.1, integrate_limit=5.0  (对应 ~0.5g 加速度)
    //   高度 PID (单级, 10Hz, dt=0.1s):
    //     仿真测试结论 (a_star ∈ [-2, 2] m/s? 范围内验证):
    //       Kp=4   Ki=1   Kd=4   integrate_limit=2.0
    //     说明: |a*|<2 时表现良好, 略有超调但尚可接受, 建议作为首次试飞起始值。
    //     垂直速度: 已合并到高度单级 PID 中, 不再独立使用
    p->pid_vx.Kp = 2.0f;  p->pid_vx.Ki = 0.1f;  p->pid_vx.Kd = 0.0f;
    p->pid_vx.integrate = 0.0f;  p->pid_vx.integrate_limit = 5.0f;  p->pid_vx.last_error = 0.0f;
    p->pid_vy.Kp = 2.0f;  p->pid_vy.Ki = 0.1f;  p->pid_vy.Kd = 0.0f;
    p->pid_vy.integrate = 0.0f;  p->pid_vy.integrate_limit = 5.0f;  p->pid_vy.last_error = 0.0f;
    p->pid_vz.Kp = 0.0f;  p->pid_vz.Ki = 0.0f;  p->pid_vz.Kd = 0.0f;
    p->pid_vz.integrate = 0.0f;  p->pid_vz.integrate_limit = 0.0f;  p->pid_vz.last_error = 0.0f;
    p->pid_alt.Kp = 0.0f;  p->pid_alt.Ki = 0.0f;  p->pid_alt.Kd = 0.0f;
    p->pid_alt.integrate = 0.0f;  p->pid_alt.integrate_limit = 0.0f;  p->pid_alt.last_error = 0.0f;

    // 姿态收敛时间常数 T 先置零，推荐起始值 T=0.5s
    p->T = 0.5f;
    p->arm_length           = 0.425f;
    p->thrust_coeff         = 3.56473192e5f;
    p->counter_torque_coeff = 40.98774809f;
    p->mass                 = 4.5f;
    p->inertia[0][0] = 0.015f;   p->inertia[0][1] = 0.0f;   p->inertia[0][2] = 0.0f;
    p->inertia[1][0] = 0.0f;    p->inertia[1][1] = 0.015f;  p->inertia[1][2] = 0.0f;
    p->inertia[2][0] = 0.0f;    p->inertia[2][1] = 0.0f;    p->inertia[2][2] = 0.025f;

    p->optflow_position.x = -0.15f;  p->optflow_position.y = 0.0f;  p->optflow_position.z = 0.0f;

    p->max_tilt_angle = 15.0f * (M_PI / 180.0f);  // 速度控制倾角限制在15°

    // 可靠姿态四元数初始化（单位四元数）
    gDroneControlAlgo.reliable_quat.w = 1.0f;
    gDroneControlAlgo.reliable_quat.x = 0.0f;
    gDroneControlAlgo.reliable_quat.y = 0.0f;
    gDroneControlAlgo.reliable_quat.z = 0.0f;

    // 高度 KF 初始化（用气压计初始高度）
    kalman_3d_init(&gDroneControlAlgo.kf_height, gDroneControlAlgo.altitude);
}

// ============================================================
// IMU 数据同步: 将原始传感器数据转换为算法所需的 SI 单位
// ============================================================
// 调用频率: 100Hz, 每次飞控内环迭代前必须调用
//
// 单位转换说明:
//   WitImuData_t.acc[3]   → g (重力加速度) → m/s?
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
    // 归一化 yaw 到 [-π, π]，与控件 target 范围对齐 (IMU 原始范围 [0°, 360°])
    if (gDroneControlAlgo.euler.yaw > M_PI)
        gDroneControlAlgo.euler.yaw -= 2.0f * M_PI;

    /* ---- 3. 角速度: deg/s → rad/s ---- */
    gDroneControlAlgo.angular_velocity.x = imu->gyro[0] * DEG_TO_RAD;
    gDroneControlAlgo.angular_velocity.y = imu->gyro[1] * DEG_TO_RAD;
    gDroneControlAlgo.angular_velocity.z = imu->gyro[2] * DEG_TO_RAD;

    /* ---- 4. 加速度: g → m/s? ---- */
    gDroneControlAlgo.linear_acceleration.x = imu->acc[0] * 9.80665f;
    gDroneControlAlgo.linear_acceleration.y = imu->acc[1] * 9.80665f;
    gDroneControlAlgo.linear_acceleration.z = imu->acc[2] * 9.80665f;

    /* ---- 5. 高度: 直接复制 ---- */
    gDroneControlAlgo.altitude = imu->altitude;
}

// ============================================================
// 光流数据同步: 将传感器数据做单位转换和坐标对齐后写入
// ============================================================
// 调用频率: 100Hz, IMU_Data_update 之后立即调用
//
// 此函数完成:
//   1. 坐标系对齐 (X 取反 → flow_vel_x 已为机体坐标系)
//   2. 单位转换: cm/s → m/s, mm → m
//
// velocity_height_Data_update 直接使用 flow_vel_x/y 做陀螺仪补偿，
// 不再重复做坐标/单位转换。
//
// 参数:
//   vx       - 光流 X 速度 (cm/s, 外部坐标系)
//   vy       - 光流 Y 速度 (cm/s, 外部坐标系)
//   range    - TOF 激光测距 (mm)
//   quality  - 光流质量 (0~100, MTF-01 传感器上报)

void Optflow_Data_update(float vx, float vy, float range, uint16_t quality)
{
    gDroneControlAlgo.flow_vel_x = -vx * 0.01f;      // X 取反 + cm→m/s
    gDroneControlAlgo.flow_vel_y =  vy * 0.01f;      // cm→m/s
    gDroneControlAlgo.laser_range = range * 0.001f;  // mm→m
    gDroneControlAlgo.flow_quality = quality;
    //考虑到安装方向与单位差异之后，在这里完成了SI单位制的统一转换
}

// ============================================================
// 三维 Kalman 滤波 - 高度估计
// ============================================================
// 状态向量: [z, vz, ba]^T
//  dt = 0.01s @ 100Hz
//
// 预测模型:
//   z(k+1)  = z(k)  + dt * vz(k)
//   vz(k+1) = vz(k) + dt * (acc_z_world - GRAVITY + ba(k))
//   ba(k+1) = ba(k)
//
// 观测量: 气压计高度 或 激光 TOF 高度
//
// 传感器可靠性（决定了噪声参数设置）:
//   静态: 激光 >>> 气压计 > 加速度计
//   动态: 激光 > 加速度计 > 气压计

#define KF_GRAVITY   9.80665f
#define KF_DT        0.01f

static void kalman_3d_init(kalman_3d_t *kf, float init_z)
{
    kf->z  = init_z;
    kf->vz = 0.0f;
    kf->ba = 0.0f;

    // 初始协方差（中等置信度）
    kf->P[0][0] = 1.0f;   kf->P[0][1] = 0.0f;   kf->P[0][2] = 0.0f;
    kf->P[1][0] = 0.0f;   kf->P[1][1] = 1.0f;   kf->P[1][2] = 0.0f;
    kf->P[2][0] = 0.0f;   kf->P[2][1] = 0.0f;   kf->P[2][2] = 0.1f;

    // --- 噪声参数设置（依据传感器可靠性调优） ---
    // 激光: 静态/动态均最高可靠 → 极小的 r_laser，KF 高度信赖激光测量
    // 气压计: 静态中等 / 动态最差 → 较大的 r_baro
    // 加速度计: 动态中等可靠 → 适中的 q_z / q_vz / q_ba
    kf->q_z     = 0.1f;          // 位置过程噪声: accel 积分有漂移
    kf->q_vz    = 0.5f;          // 速度过程噪声: accel 积分发散
    kf->q_ba    = 0.0005f;       // accel 偏置漂移: 动态可靠性中等
    kf->r_baro  = 0.5f;          // 气压计方差: 静态估计 ±0.7m，动态更差
    kf->r_laser = 0.000025f;     // 激光方差: ±5mm @ 1σ，极高信任
}

static void kalman_3d_predict(kalman_3d_t *kf, float acc_z_world)
{
    float dt = KF_DT;
    float z  = kf->z;
    float vz = kf->vz;
    float ba = kf->ba;

    // 1. 状态预测
    float acc_eff = acc_z_world - KF_GRAVITY + ba;  // 有效垂直加速度
    kf->z  = z  + dt * vz;
    kf->vz = vz + dt * acc_eff;
    // ba 保持不变

    // 2. 协方差预测: P_new = F * P * F^T + Q
    const float (*P)[3] = kf->P;

    float P11 = P[0][0] + dt * (2.0f * P[0][1] + dt * P[1][1]);
    float P12 = P[0][1] + dt * P[1][1] + dt * (P[0][2] + dt * P[1][2]);
    float P13 = P[0][2] + dt * P[1][2];
    float P22 = P[1][1] + dt * (2.0f * P[1][2] + dt * P[2][2]);
    float P23 = P[1][2] + dt * P[2][2];

    kf->P[0][0] = P11 + kf->q_z;
    kf->P[0][1] = P12;
    kf->P[0][2] = P13;
    kf->P[1][0] = P12;
    kf->P[1][1] = P22 + kf->q_vz;
    kf->P[1][2] = P23;
    kf->P[2][0] = P13;
    kf->P[2][1] = P23;
    kf->P[2][2] = P[2][2] + kf->q_ba;
}

static void kalman_3d_update(kalman_3d_t *kf, float z_meas, float r)
{
    float P00 = kf->P[0][0];
    float P01 = kf->P[0][1];
    float P02 = kf->P[0][2];

    // 创新 S = H * P * H^T + R = P00 + R
    float S = P00 + r;
    if (S < 1e-12f) S = 1e-12f;

    // Kalman 增益 K = P * H^T / S = [P00, P01, P02]^T / S
    float K0 = P00 / S;
    float K1 = P01 / S;
    float K2 = P02 / S;

    // 创新 y = z_meas - H * x_pred = z_meas - z
    float y = z_meas - kf->z;

    // 状态更新
    kf->z  += K0 * y;
    kf->vz += K1 * y;
    kf->ba += K2 * y;

    // 协方差更新: P = (I - K*H) * P
    kf->P[0][0] -= K0 * P00;
    kf->P[0][1] -= K0 * P01;
    kf->P[0][2] -= K0 * P02;
    kf->P[1][0] -= K1 * P00;
    kf->P[1][1] -= K1 * P01;
    kf->P[1][2] -= K1 * P02;
    kf->P[2][0] -= K2 * P00;
    kf->P[2][1] -= K2 * P01;
    kf->P[2][2] -= K2 * P02;
}

// ============================================================
// 速度与高度融合: 光流 XY 速度 + IMU 补偿 + KF 高度估计 + 世界速度
// ============================================================
// 调用频率: 100Hz, Optflow_Data_update 之后, 控制环之前
//
// 本函数包含五个核心步骤：
//   1. 光流 XY 速度 → 陀螺仪补偿 → 机体 XY 速度
//   2. 加速度计体轴 XYZ → 四元数旋转 → 世界系 Z 加速度
//   3. 机体 XY 速度 + R 矩阵 → 世界 XY 速度 (velocity_world.xy)
//   4. KF 预测（acc_z_world 驱动）+ 更新（气压计 + 激光双观测）
//   5. 写回: height = KF.z, velocity.z / velocity_world.z = KF.vz
//
// 补偿模型：
//   v_sensor = v_body + ω × r + ω × [0,0,-h]
//   v_body = v_flow - ω × (r + [0,0,-h])
//
// 光流输入约定:
//   flow_vel_x/y 已在 Optflow_Data_update 中完成单位转换 (cm/s → m/s)
//   和坐标系对齐, 此处直接用于陀螺仪补偿, 不做二次转换

void velocity_height_Data_update(void)
{
    // ====== 1. 光流 XY 速度 → 陀螺仪补偿 → 机体 XY 速度 ======
    float raw_vx = gDroneControlAlgo.flow_vel_x;  // m/s, 机体坐标系
    float raw_vy = gDroneControlAlgo.flow_vel_y;  // m/s, 机体坐标系
    float range  = gDroneControlAlgo.laser_range; // m

    const drone_control_param_t *p = &gDroneControlAlgo.param;
    float rx = p->optflow_position.x;
    float ry = p->optflow_position.y;
    float rz = p->optflow_position.z;
    float wx = gDroneControlAlgo.angular_velocity.x;
    float wy = gDroneControlAlgo.angular_velocity.y;
    float wz = gDroneControlAlgo.angular_velocity.z;

    float h = (range > 0.001f) ? range : (gDroneControlAlgo.height > 0.01f ? gDroneControlAlgo.height : 1.0f);

    // 陀螺仪效应补偿: v_body = v_flow - ω × (r + [0,0,-h])
    float rz_eff = rz - h;
    float rot_x = wy * rz_eff - wz * ry;
    float rot_y = wz * rx  - wx * rz_eff;

    gDroneControlAlgo.velocity.x = raw_vx - rot_x;
    gDroneControlAlgo.velocity.y = raw_vy - rot_y;

    // ====== 2. 体轴加速度 → 世界系 Z 加速度 ======
    // R: v_body = R @ v_earth（大地→机体）
    // R^T: v_earth = R^T @ v_body
    // 世界 Z = R[0][2]*ax + R[1][2]*ay + R[2][2]*az = R^T 第三行
    float R[3][3];
    quaternion_to_rotation_matrix(&gDroneControlAlgo.reliable_quat, R);

    float ax = gDroneControlAlgo.linear_acceleration.x;
    float ay = gDroneControlAlgo.linear_acceleration.y;
    float az = gDroneControlAlgo.linear_acceleration.z;
    float acc_z_world = R[0][2] * ax + R[1][2] * ay + R[2][2] * az;

    // ====== 3. 机体速度 → 世界速度 (利用步骤 2 已算好的 R) ======
    // v_earth.xy = R^T @ v_body.xy, v_earth.z 由 KF 提供
    gDroneControlAlgo.velocity_world.x = R[0][0] * gDroneControlAlgo.velocity.x
                                       + R[1][0] * gDroneControlAlgo.velocity.y;
    gDroneControlAlgo.velocity_world.y = R[0][1] * gDroneControlAlgo.velocity.x
                                       + R[1][1] * gDroneControlAlgo.velocity.y;

    // ====== 4. Kalman 滤波预测 + 更新 ======
    kalman_3d_t *kf = &gDroneControlAlgo.kf_height;

    // 预测步（加速度计驱动）
    kalman_3d_predict(kf, acc_z_world);

    // 更新步 1：气压计（始终可用，r 较大 → 权重低）
    kalman_3d_update(kf, gDroneControlAlgo.altitude, kf->r_baro);

    // 更新步 2：激光 TOF — 激光质量与光流水平速度质量无关，使用固定 r_laser
    //   range > 0.001f (=1mm) 过滤掉 0（不可用），激光最小测量值 2mm
    if (range > 0.001f) {
        kalman_3d_update(kf, range, kf->r_laser);
    }

    // ====== 5. 写回输出 ======
    gDroneControlAlgo.height         = kf->z;
    gDroneControlAlgo.velocity.z     = kf->vz;
    gDroneControlAlgo.velocity_world.z = kf->vz;  // KF 垂直速度已为世界系
}

// ============================================================
// 模块 3: 速度与高度控制 —— PID → 期望加速度 → 姿态角 + 推力（10Hz）
// ============================================================
// 算法流程:
//   1. 读取 velocity_world (已由 velocity_height_Data_update @ 100Hz 更新)
//   2. 水平速度 PID: 世界系速度误差 → 期望水平加速度 (世界系)
//   3. 高度单级 PID: 位置误差 → 期望垂直加速度
//   4. 期望世界加速度 [ax, ay, az] → 体轴 Z 方向 + 推力 T
//      (牛顿第二定律: T * z_body_world + [0,0,-mg] = m * a_des)
//   5. 体轴 Z → roll, pitch; 推力 T → target_throttle
//      yaw 不受影响, 由遥控器/上位机直接指定
//
// 被控量:
//   input:
//     velocity_world          ← 本函数 step 1 计算
//     height                  ← KF 融合高度
//     target_velocity         ← 外部写入 (世界坐标系)
//     target_altitude         ← 外部写入
//   output:
//     target_euler.roll/pitch ← 本函数写入
//     target_throttle         ← 本函数写入
//
// 坐标系约定:
//   Z = up (高度正), 重力加速度 g ≈ 9.81 m/s? 朝下
//   水平速度在世界 X/Y 平面, 航向 yaw 由外部独立控制

void drone_control_velocity_altitude_control(void)
{
    const float dt = 0.1f;                          // 10Hz 更新周期
    const float g = 9.80665f;
    drone_control_param_t *param = &gDroneControlAlgo.param;

    // ---- 守卫: 参数未配置时跳过 ----
    if (param->max_tilt_angle < 0.01f) {
        return;
    }

    // ====== 1. 读取世界速度 (已由 velocity_height_Data_update @ 100Hz 更新) ======
    vector_t *v_world = &gDroneControlAlgo.velocity_world;

    // ====== 2. 水平速度 PID → 期望水平加速度 (世界坐标系) ======
    float err_vx = gDroneControlAlgo.target_velocity.x - v_world->x;
    float err_vy = gDroneControlAlgo.target_velocity.y - v_world->y;

    float acc_x = PID_update(err_vx, &param->pid_vx, dt);
    float acc_y = PID_update(err_vy, &param->pid_vy, dt);

    // ====== 3. 高度单级 PID → 期望垂直加速度 ======
    // PID: acc_z = Kp*e + Ki*∫e·dt + Kd·de/dt
    //   比例项: 位置误差 → 回复加速度
    //   积分项: 消除静差（风、质量偏差等），对二阶系统至关重要
    //   微分项: de/dt ≈ -Vz → 提供阻尼，使系统临界阻尼 (ζ=1.0)
    float alt_err = gDroneControlAlgo.target_altitude - gDroneControlAlgo.height;
    float acc_z = PID_update(alt_err, &param->pid_alt, dt);

    // ====== 4. 期望世界加速度限幅（防止指令过激） ======
    // 水平加速度对应倾斜角: tan(tilt) = a_horiz / g
    // 直接限制水平加速度幅值 ≤ max_tilt_angle * g（小角度近似）
    float max_horiz = param->max_tilt_angle * g;
    float horiz_mag = sqrtf(acc_x * acc_x + acc_y * acc_y);
    if (horiz_mag > max_horiz) {
        float scale = max_horiz / horiz_mag;
        acc_x *= scale;
        acc_y *= scale;
    }

    // ====== 5. 世界加速度 → 体轴 Z 方向 + 推力 ======
    // 牛顿第二定律: a_des = T/m * z_body_world + [0,0,-g]  (Z=up)
    //              z_body_world = [ax, ay, az+g] / ||[ax, ay, az+g]||
    //              推力 T = m * ||[ax, ay, az+g]||
    float az_total = acc_z + g;
    float thrust_norm = sqrtf(acc_x * acc_x + acc_y * acc_y + az_total * az_total);

    if (thrust_norm < 1e-6f) {
        return;     // 零推力场景（理论上不应发生）
    }

    // 期望体轴 Z（世界坐标系）单位向量
    float inv_norm = 1.0f / thrust_norm;
    float zx = acc_x * inv_norm;
    float zy = acc_y * inv_norm;
    float zz = az_total * inv_norm;

    // ====== 6. 最大倾斜角保护 ======
    // 若期望体轴 Z 偏离垂直方向超过 max_tilt，将其投影到约束锥面上
    float cos_max_tilt = cosf(param->max_tilt_angle);
    if (zz < cos_max_tilt) {
        float horiz_body = sqrtf(zx * zx + zy * zy);
        if (horiz_body > 1e-6f) {
            float sin_max_tilt = sinf(param->max_tilt_angle);
            float scale = sin_max_tilt / horiz_body;
            zx *= scale;
            zy *= scale;
            zz = cos_max_tilt;
        }
    }

    // ====== 7. 体轴 Z → roll, pitch ======
    // z_body_world = [-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]
    //   pitch = -asin(zx)
    //   roll  =  atan2(zy, zz)
    if (zx > 1.0f)  zx = 1.0f;
    if (zx < -1.0f) zx = -1.0f;
    float pitch = -asinf(zx);
    float roll  = atan2f(zy, zz);

    // ====== 8. 写入输出 ======
    gDroneControlAlgo.target_euler.roll  = roll;
    gDroneControlAlgo.target_euler.pitch = pitch;
    // yaw 保持不变（由遥控器/上位机直接写入 target_euler.yaw）

    // 推力: T = m * ||[ax, ay, az+g]||
    // allocate_motors 内部用 0.025f * thrust_ref, 此处直接写入牛顿值
    gDroneControlAlgo.target_throttle = param->mass * thrust_norm;
}
