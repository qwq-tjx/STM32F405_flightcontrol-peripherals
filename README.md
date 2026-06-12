# FLYCONTROL STM32F405 飞控系统

基于 STM32F405 的四轴飞行器飞控系统，支持 **MAVLink 通信**、**DSHOT15 数字电调**、**WitMotion IMU 姿态传感器**、**双环 PID 姿态控制**、**串口油门调试**、**Mission Planner 上位机插件 (综合调参面板 + IMU 3D可视化)**。

---

## 硬件规格

| 项目 | 详情 |
|------|------|
| **主控芯片** | STM32F405RG (Cortex-M4, 168MHz, 带 FPU) |
| **电调协议** | DSHOT15 (4 路独立, 降速测试模式) |
| **姿态传感器** | WitMotion WT931 (UART5, 115200) |
| **通信协议** | MAVLink v2 (USART2, 115200) |
| **电池检测** | 24V 分压采样 (ADC1_CH15) |
| **开发环境** | Keil MDK-ARM v5, ARMCC V5.06 |
| **固件库** | STM32F4xx Standard Peripheral Driver |

---

## 引脚分配

| 功能 | GPIO | 复用功能 | 说明 |
|------|------|----------|------|
| USART1 TX | PA9 | AF_USART1 | 调试控制台 + 油门控制 (115200) |
| USART1 RX | PA10 | AF_USART1 | IMU 配置命令 + 油门接收 |
| USART2 TX | PA2 | AF_USART2 | MAVLink 发送 (DMA) |
| USART2 RX | PA3 | AF_USART2 | MAVLink 接收 |
| UART5 TX | PC12 | AF_UART5 | IMU 传感器 |
| UART5 RX | PD2 | AF_UART5 | IMU 数据接收 |
| **电机1** | **PC6** | TIM8_CH1 | DSHOT15 |
| **电机2** | **PC7** | TIM8_CH2 | DSHOT15 |
| **电机3** | **PC8** | TIM8_CH3 | DSHOT15 |
| **电机4** | **PC9** | TIM8_CH4 | DSHOT15 |
| 电池检测 | PC5 | ADC1_CH15 | 24V 分压 (10K+1K) |

---

## DSHOT15 协议

- **协议版本**: DSHOT15 (15kbps 位速率, 降速测试模式)
- **油门范围**: 0 ~ 4095 (12 位)
- **帧格式**: 16 位 = 12 位油门值 + 4 位 CRC
- **CRC 算法**: 4 位分组求和 mod 16
- **PWM 实现**: TIM8 (高级定时器) + DMA2 位带控制
- **DMA 通道**: DMA2_Stream2/3/4/7, Channel 7, 循环模式
- **PWM 参数**: ARR=559, PSC=19 → 位周期 66.67µs

### DSHOT 帧示例

```
油门值: 1017 (0x3F9)
CRC计算: group1=3, group2=15, group3=9, sum=27 → CRC=11 (0xB)
DSHOT帧: 0011111110011011
```

---

## MAVLink 通信

### 上行 (发送)

TIM7 ISR 100Hz 中调用 `mavlink_send_imu_periodic()`，默认 `attitude_interval_us=100000` (100ms=10Hz)，
相位交替发送：

| 周期 | 消息 | 单条频率 | 说明 |
|------|------|:--:|------|
| 1s | HEARTBEAT | 1Hz | 心跳包 |
| 500ms | SERVO_OUTPUT_RAW | 2Hz | 4路电机油门回读 |
| 10Hz 偶数轮 | SCALED_PRESSURE | 5Hz | 气压数据 |
| 10Hz 偶数轮 | ALTITUDE | 5Hz | 高度数据 |
| 10Hz 偶数轮 | ATTITUDE_QUATERNION | 5Hz | 四元数姿态 |
| 10Hz 奇数轮 | HIGHRES_IMU | 5Hz | 加速度/角速度/磁力计 |
| 10Hz 奇数轮 | ATTITUDE | 5Hz | 欧拉角+角速度 (Roll/Pitch/Yaw) |
| 1s (TIM6) | SYS_STATUS | 1Hz | 电池电压 |

> VFR_HUD 已注释禁用。姿态数据可通过 `MAV_CMD_SET_MESSAGE_INTERVAL` 动态调速。

### 下行 (接收命令)

| 命令/消息 | ID | 说明 |
|-----------|-----|------|
| MAV_CMD_DO_SET_SERVO | 183 | 设置单路/全部电机油门 |
| RC_CHANNELS_OVERRIDE | 70 | RC 通道覆盖映射 |
| DEBUG_FLOAT_ARRAY | 350 | 目标姿态/角速度 (name=ATTI) + PID参数 (name=PIDP) + PID查询 (name=PIDQ) |
| MAV_CMD_SET_MESSAGE_INTERVAL | 511 | 设置消息发送频率 |
| MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES | 520 | 返回飞控版本信息 |

### PID 参数协议

| 消息 | name | 数据 | 方向 |
|------|------|------|------|
| DEBUG_FLOAT_ARRAY | `PIDP` | data[0]=T, data[1-3]=Roll Kp/Ki/Kd, data[4-6]=Pitch, data[7-9]=Yaw | 上行 (发送PID) |
| DEBUG_FLOAT_ARRAY | `PIDQ` | 空数据(10×0) | 上行 (查询PID) |
| NAMED_VALUE_FLOAT | `PIDR0`~`PIDR9` | T + 3轴×3 = 10个值 | 下行 (飞控回传) |
| STATUSTEXT | `PIDRA\|` / `PIDRB\|` | 双段各5值，组装为10个PID参数 | 下行 (备选回传通道) |

### PID 初始化与默认值

上电时 `drone_control_algorithm_init()` 已预设**姿态内环 P 值**（电机在 ATTI/RATE 模式下可直接启动）：

| 控制器 | Kp | Ki | Kd | integrate_limit | 说明 |
|--------|-----|-----|-----|:--:|------|
| **pidx** (Roll 横滚) | **24.0** | 0 | 0 | 0 | 角速度内环 |
| **pidy** (Pitch 俯仰) | **24.0** | 0 | 0 | 0 | 角速度内环 |
| **pidz** (Yaw 偏航) | **3.0** | 0 | 0 | 0 | 角速度内环 |
| pid_vx (水平速度) | **2.0** | 0.1 | 0 | 5.0 | 速度控制 |
| pid_vy (水平速度) | **2.0** | 0.1 | 0 | 5.0 | 速度控制 |
| pid_alt (高度) | **0** | 0 | 0 | 0 | VELH 模式高度闭环 |

代码注释中的**推荐调参起始值**（需通过上位机手动写入）：

| 轴 | Kp | Ki | Kd | T |
|----|-----|-----|-----|------|
| Roll | 20 | 200 | 0 | 0.50s |
| Pitch | 20 | 200 | 0 | |
| Yaw | 25 | 250 | 0 | |
| 高度 | 4 | 1 | 4 | — |

> **注意**：高度 PID (`pid_alt`) 默认全零，切换 VELH 模式前必须通过上位机填入 Kp/Ki/Kd，否则仅有重力前馈无高度闭环修正。

### 控制模式

飞控支持四种控制模式，由 `control_mode` 变量控制（定义于 `User/control.h`），在 `TIM7_IRQHandler` (100Hz) 中调度执行。

#### 模式概览

```
DISABLED                    ATTI                          RATE                          VELH
════════                    ════                          ════                          ════
IMU读取 ✓                   IMU读取 ✓                      IMU读取 ✓                      IMU读取 ✓
数据同步 ✓                  数据同步 ✓                     数据同步 ✓                     数据同步 ✓
                            target_euler ← 插件            (不写 target_euler)            target_velocity/altitude ← 插件
                            outer loop 20Hz:               outer loop: 跳过 ✗            velocity_altitude_control 10Hz:
                              q_e → error_angle            omega_ref ← 插件直接注入        世界速度PID → 期望加速度
                              omega_ref = f(error_angle)   ─────────────↓                 加速度 → 姿态角 + 推力
                            ──────────────────────────→   内环 100Hz:                    ──────────────────────────→
飞控跳过 ✗                  内环 100Hz:                      PID(omega_ref vs gyro)        内环 100Hz + 混控 → DSHOT
                              PID(omega_ref vs gyro)        动力学补偿
                              动力学补偿                     混控 → DSHOT
                              混控 → DSHOT
                            
油门来源: 串口手动            油门来源: 插件 target_throttle   油门来源: 插件 target_throttle  油门来源: 高度速度PID输出
角度控制: 无                 角度控制: 自动回平              角度控制: 无 (松杆不停)         角度控制: 倾斜角限幅 ±15°
```

#### CTRL_MODE_DISABLED (0) — 应急/调试模式

TIM7 中断中整个飞控模块被跳过：

```
if (control_mode != CTRL_MODE_DISABLED) {
    // 外环 + 内环 + 混控 → 都不执行
}
// DISABLED 时油门由主循环的串口/DSHOT 队列手动控制
```

- 上电默认状态（安全，不会自动启动电机）
- 串口命令 `@motor0,motor1,motor2,motor3\r\n` 手动调速
- 上位机发 `name="DISA"` 可紧急切回

#### CTRL_MODE_ATTITUDE (1) — 姿态自稳模式 (双环)

完整的外环+内环控制链路：

```
目标角度 (target_angle[3])
    │
    ▼  外环 20Hz
euler_to_quaternion(target_euler) → q_d
q_e = q_c⁻¹ * q_d                      ← 误差四元数
提取轴角: error_angle, error_axis
omega_ref = (error_angle / T) * error_axis   ← T 为收敛时间常数
    │
    ▼  内环 100Hz
PID(omega_ref vs angular_velocity) → alpha_ref
tau = J·alpha + ω×(J·ω)               ← 欧拉动力学
混控分配: thrust + torque → 四电机转速
    │
    ▼
转速 → DSHOT (0~4095) → DMA 输出
```

- 指定目标 Roll/Pitch/Yaw → 飞控自动解算角速度 → 达到期望姿态
- 松杆 → 回平 (目标角度=0)
- 通过 `DEBUG_FLOAT_ARRAY` (name=`ATTI`) 接收：data[0-2]=目标角度(rad), data[3-5]=目标角速度(rad/s)

#### CTRL_MODE_RATE (2) — 角速度模式 (仅内环)

跳过外环，插件直接下发目标角速度：

```
target_gyro[3] (由插件直接指定)
    │
    ▼ (跳过外环, omega_ref = target_gyro)
内环 100Hz:
  PID(omega_ref vs angular_velocity) → alpha_ref
  tau = J·alpha + ω×(J·ω)
  混控分配 → 四电机转速
    │
    ▼
转速 → DSHOT (0~4095) → DMA 输出
```

- 仅内环运行，无外环角度闭环
- 不自动回平，松杆后不会回到水平
- 通过 `DEBUG_FLOAT_ARRAY` (name=`RATE`) 接收：data[0-2]=目标角速度(rad/s)

#### CTRL_MODE_VH (3) — 速度与高度闭环

世界坐标系速度+高度 PID 控制，上位机下发目标速度和高度的完整闭环：

```
target_velocity[3] + target_altitude (由插件指定)
    │
    ▼  速度高度控制 10Hz
世界坐标系速度 PID → 期望加速度 (ax, ay, az)
az_total = az_pid + g (重力前馈)
thrust_norm = sqrt(ax² + ay² + az_total²)
期望倾角 = atan2(sqrt(ax²+ay²), az_total)
    │
    ▼
target_euler.roll/pitch ← 期望倾角 (限幅 ±15°)
target_throttle ← mass × thrust_norm
    │
    ▼  内环 100Hz + 混控 → DSHOT
```

- 水平速度 PID + 高度 PID 双闭环
- 最大倾斜角限幅 15° (安全保护)
- 通过 `DEBUG_FLOAT_ARRAY` (name=`VELH`) 接收：data[0-2]=目标速度(m/s), data[3]=目标高度(m)
- **重要**：高度 PID (`pid_alt`) 默认全零，切换前必须填入参数，否则仅重力前馈无闭环修正
- **地面解锁 VELH 时电机将输出悬停级推力 (~2100 DSHOT)**，必须注意安全

#### 模式切换

通过 `DEBUG_FLOAT_ARRAY` 消息的 `name` 字段切换：

| name | 对应模式 | 说明 |
|------|:---:|------|
| `DISA` | DISABLED | 立即停止飞控，退回手动油门 |
| `ATTI` | ATTITUDE | 双环姿态控制 |
| `RATE` | RATE | 仅内环角速度控制 |
| `VELH` | VH | 速度+高度闭环控制 |

#### 对比总结

| | DISABLED | ATTI | RATE | VELH |
|---|:---:|:---:|:---:|:---:|
| 外环 (20Hz) | ❌ | ✅ 轴角误差法 | ❌ | ❌ |
| 速度高度环 (10Hz) | ❌ | ❌ | ❌ | ✅ |
| 内环 PID (100Hz) | ❌ | ✅ | ✅ | ✅ |
| 动力学补偿 | ❌ | ✅ | ✅ | ✅ |
| omega_ref 来源 | — | 外环计算 | 插件直接赋值 | 速度环解算 |
| target_euler 写入 | ❌ | ✅ | ❌ | ✅ (速度环) |
| 自动回平 | — | ✅ | ❌ | — |
| DSHOT 输出 | 串口手动 | PID+混控 | PID+混控 | PID+混控 |
| 重力前馈 | ❌ | ✅ | ✅ | ✅ |

---

## 控制算法

飞控采用**双环串级 PID** 控制架构 (`User/drone_control.c`)：

```
                         目标姿态角 (20Hz)
                              │
                              ▼
┌─────────────────────────────────────────────┐
│  外环 (角度环): 角度误差 → PID → 目标角速度  │
│     输入: target_angle[3] (Roll/Pitch/Yaw)  │
│     输出: target_gyro[3] (期望角速度)        │
└──────────────────┬──────────────────────────┘
                   │
                   ▼  当前角速度 (100Hz)
┌─────────────────────────────────────────────┐
│  内环 (角速度环): 角速度误差 → PID → 油门   │
│     输入: target_gyro[3] + 当前角速度        │
│     输出: motor_throttle[4] (0~4095)        │
└─────────────────────────────────────────────┘
```

- **IMU 数据更新**: 100Hz (`TIM7_IRQHandler`)
- **外环频率**: 20Hz (每5次执行1次)
- **内环频率**: 100Hz (每次执行)
- **PID 参数**: T(时间常数) + 三轴 Kp/Ki/Kd，可通过 MAVLink 在线调参
- **控制输出**: `control_mode ≠ 0` 时覆写 `motor_throttle[4]`，否则使用串口油门

### 通道映射

| MAVLink 通道 | 电机索引 | 说明 |
|-------------|----------|------|
| 1 ~ 4 | 0 ~ 3 | RC 通道直接映射 |
| 9 ~ 12 | 0 ~ 3 | ThrottleControl 插件通道 |

---

## 系统架构

```
初始化流程:
  systick → USART1(调试) → UART5 + WitMotion IMU
         → USART2 + MAVLink
         → DSHOT(TIM8+DMA)
         → ADC 电池检测
         → 飞控算法初始化
         → TIM7 (100Hz 飞控+MAVLink)
         → TIM6 (1Hz 电池电压)

主循环:
  while(1):
    1. 处理 MAVLink 油门命令队列
    2. 检测串口油门更新标志 (USART1)
    3. 检测飞控算法油门更新标志 (TIM7 ISR)
    4. 任一更新 → 刷新 DSHOT DMA 缓冲区
    5. LED 1Hz 闪烁 (与 MAVLink 心跳同步)
```

---

## 编译与烧录

### 环境要求

- Keil MDK-ARM v5 或更高
- STM32F4xx DFP 包
- ARM Compiler v5.06

### 编译步骤

1. 打开 `Project/Template.uvprojx`
2. 选择 Target: **Template**
3. 点击 Build (F7)
4. 输出文件: `Objects/Template.axf` / `Objects/Template.hex`

### 烧录

使用 ST-Link 或 J-Link 烧录 `Objects/Template.hex` 到 STM32F405。

---

## 上位机插件

四个 Mission Planner 插件，位于工程根目录，使用 .NET Framework 4.7.2 编译。

**编译**：
```powershell
msbuild ThrottleControlPlugin\ThrottleControlPlugin.csproj /p:Configuration=Release
msbuild ImuVisualizationPlugin\ImuVisualizationPlugin.csproj /p:Configuration=Release
msbuild TargetControlPlugin\TargetControlPlugin.csproj /p:Configuration=Release
msbuild TargetThrottlePlugin\TargetThrottlePlugin.csproj /p:Configuration=Release
```

### ThrottleControlPlugin — 飞控综合调参面板

Mission Planner 插件，一站式飞控调参工具：

**油门控制**
- 4 路电机独立 TrackBar + 数值输入框 (0~4095)
- 飞控油门回读显示 (SERVO_OUTPUT_RAW / RC_CHANNELS)
- 同步所有电机 / 紧急停止按钮
- 使用 `MAV_CMD_DO_SET_SERVO` (通道 9-12)

**目标姿态 / 角速度**
- 三轴目标角度 (Roll/Pitch/Yaw, ±3.14 rad) 滑条 + 度数显示
- 三轴目标角速度 (X/Y/Z, ±34.9 rad/s) 滑条
- 通过 `DEBUG_FLOAT_ARRAY` (name=ATTI) 发送

**PID 参数调谐**
- 三轴 Kp/Ki/Kd (0.0~300.0) + 时间常数 T (0.00~1.00s)
- 「发送 PID」按钮 → 手动推送到飞控 (滑条不自动发送)
- 「查询 PID」按钮 → 从飞控回读当前 PID 值并同步控件
- 「推荐值」按钮 → 恢复代码默认 PID (仅改 UI，不发送)
- **PID2 调参**：第二组 PID 参数界面，支持多控制器独立调参

**电池电压显示**
- 实时显示母线电压 (V)、单节电压 (6S, 保留2位小数)、剩余电量 (%)
- 数据来源: `SYS_STATUS` (msgid=1) 的 `voltage_battery` + `battery_remaining`
- 单节 < 3.65V → 醒目红色低电压预警
- 超时 5 秒无数据 → 灰色提示

### TargetControlPlugin — 目标控制面板

综合模式切换 + 目标值下发：

- **模式切换**：DISABLED / ATTI / RATE / VELH 四模式按钮
- **姿态控制** (ATTI)：目标角度 (rad) + 目标角速度 (rad/s) + 推力 (N)
- **角速度控制** (RATE)：目标角速度 (rad/s)
- **速度高度控制** (VELH)：目标水平速度 (m/s) + 目标高度 (m)
- 通过 `DEBUG_FLOAT_ARRAY` (name=DISA/ATTI/RATE/VELH) 发送

### TargetThrottlePlugin — 目标油门控件

独立目标油门输入窗口，范围为 0~10 kg。功能已被 `TargetControlPlugin` 的推力滑块覆盖，保留作为精简备用。

### ImuVisualizationPlugin — IMU 数据可视化

Mission Planner 插件，实时显示飞控传感器数据：
- 欧拉角 / 角速度 / 加速度数值面板
- 四元数 / 磁力计 / 气压高度面板
- 姿态 / IMU / 高度 时间序列曲线图
- 暂停刷新开关

### 直接控制 (无插件)

在 Mission Planner 的 `SERVO` 页面设置通道 9-12 的 PWM 值。

---

## 串口调试

飞控板提供 3 个串口：USART1 (PA9/PA10) 为调试串口，USART2 (PA2/PA3) 为 MAVLink，UART5 (PC12/PD2) 为 IMU。

### 一、USART1 串口油门直驱协议

通过串口助手（115200 8N1）发送油门指令，帧格式：

```
@t1,t2,t3,t4\r\n
```

| 字段 | 说明 |
|------|------|
| `@` | 帧头标记 |
| `t1~t4` | 4 个电机油门值，逗号分隔，范围 0~4095 |
| `\r\n` | 帧尾（`\r` 可省略，`\n` 必须） |

**示例**：发送 `@1000,1000,800,800` + 回车

飞控收到后回显确认：
```
[THR] SER: 1000 1000 800 800
          BIN: 0000001111101000 0000001111101000 0000001100100000 0000001100100000
```

> **注意**：超范围值自动限幅到 4095；无需任何上位机软件，任意串口助手均可使用。
> 实现位置：`Hardware/Serial.c` → `Serial_ParseThrottle()`

### 二、ADC 电池电压调试打印

函数 `ADC_PrintRaw()` (定义于 `Hardware/adc.c`)，输出 ADC 原始值、VDDA 参考电压和电池电压。

**使用方法**：在 `main.c` 主循环中取消 `ADC_PrintRaw()` 的注释即可：
```c
// main.c 主循环中
ADC_PrintRaw();  // 取消注释可打印电池电压/VDDA/ADC原始值到USART1
```

**输出示例**：
```
ADC Raw: 2048, VDDA: 3.296V, Battery: 22.13V
```

> **注意**：调用频率过高会阻塞 USART1，建议加延时或降频打印。

### 三、启动信息

上电后 USART1 串口助手会依次输出：
```
USART_Init
DShot_Init
ADC_Config
DroneControl_Init
```

### 四、已禁用的串口阻塞打印 (P0 修复)

由于 ISR 上下文中的阻塞 `Serial_Printf` 会抢占 UART5，导致 IMU 传感器丢包和飞控失控，
以下打印已被注释禁用。如需调试，在**主循环或低优先级 ISR** 中取消注释即可，
**绝对不要**在 `USART2_IRQ` 或 `UART5_IRQ` 中解注。

| 文件 | 行号 | 原打印内容 | 用途 |
|------|------|-----------|------|
| `mavlink_c/mavlink.c` | 532 | `[THR] throttle:%.2f N` | MAVLink 目标油门 |
| | 538 | `[CTRL] DISABLED` | 控制模式禁用通知 |
| | 555-556 | `[RATE] gyr(deg/s):...` | 角速度模式目标值 |
| | 580-582 | `[ATTI] ang(deg):...` | 姿态模式目标值 |
| | 607-611 | `[PIDP] T=...` | PID 参数写入确认 |
| | 666,674 | `[THR] SERVO ch%d=%d` | 舵机/电机设置确认 |
| `Hardware/TIM.c` | 297-298 | `[DEBUG] target_throttle=...` | 油门链路验证 |

---

## 工程目录结构

```
├── CMSIS/               CMSIS 内核头文件 + STM32F4xx 头文件
├── Hardware/            硬件驱动模块
│   ├── DSHOT.c/.h       DSHOT 电调驱动
│   ├── Serial.c/.h      串口驱动 (USART1/2, UART5)
│   ├── TIM.c/.h         定时器 + 飞控控制调度
│   ├── PWM.c/.h         PWM 基础驱动
│   ├── adc.c/.h         ADC 电池检测 (12-bit, DMA HalfWord)
│   ├── Delay.c/.h       SysTick 延时
│   ├── LED.c/.h         LED 指示
│   ├── mtf01.c/.h       MTF-01 光流传感器驱动
│   ├── debug_output.c/.h  调试输出模块
│   ├── imu_simulator.c     IMU 数据模拟器
│   ├── wit_c_sdk.c/.h   WitMotion IMU SDK
│   └── REG.h            IMU 寄存器映射
├── Library/             STM32F4xx 标准外设库
├── mavlink_c/           MAVLink v2 协议栈
│   ├── mavlink.c        自定义 MAVLink 处理逻辑
│   └── common.h          所有 MAVLink 消息头文件
├── Project/             Keil 工程文件
├── Startup/             startup_stm32f405xx.s
├── User/                用户应用
│   ├── main.c / main.h   主程序
│   ├── control.c / control.h  控制模式 & 目标值
│   ├── drone_control.c / drone_control.h  双环 PID 姿态 + 速度高度控制
│   └── stm32f4xx_it.c   中断服务函数
├── ThrottleControlPlugin/   Mission Planner 综合调参插件源码
├── TargetControlPlugin/     Mission Planner 目标控制插件源码
├── TargetThrottlePlugin/    目标油门独立插件源码
├── ImuVisualizationPlugin/  Mission Planner IMU 可视化插件源码
├── ThrottleControlPlugin.dll   综合调参插件
├── TargetControlPlugin.dll     目标控制插件
├── TargetThrottlePlugin.dll    目标油门插件 (精简备用)
├── ImuVisualizationPlugin.dll  IMU 可视化插件
└── README.md
```

---

## 关键技术细节

### DMA 循环模式

DSHOT 使用 DMA 循环模式 (`DMA_Mode_Circular`)，每个 TIM8 PWM 周期自动触发一次 DMA 传输，将缓冲区数据写入 CCR 寄存器，实现连续 DSHOT 脉冲输出。

### 临界区保护

油门值更新使用 `__set_PRIMASK(1/0)` 关全局中断保护，防止主循环和 MAVLink 中断同时写入 `current_throttle[]`。

### 电池电压计算

24V 电池经过 10K/1K 分压 (11:1) 后接入 PC5，ADC 配置要点：

| 参数 | 值 | 说明 |
|------|------|------|
| ADC 分辨率 | 12-bit | 0~4095 |
| 数据对齐 | **右对齐** (显式强制) | `ADC1->CR2 &= ~(1<<11)` |
| DMA 传输宽度 | **HalfWord (16-bit)** | 防止 32-bit 读取高 16 位垃圾 |
| 分压比 | 1/11 | 10K + 1K 分压电路 |
| VDDA | 3.3V (默认) | VREFINT 校准暂禁用 |

```
Vbat = ADC值 × VDDA / 4096 × 11
示例: adc=2708 → 2708×3.3/4096×11 = 24.0V
```

> **修复记录**：DMA 原先使用 Word (32-bit) 传输，高 16 位保留区读到 0xFFxx 垃圾值导致原始值异常 (4672~65520)。
> 同时 ADC_Init 可能未正确清除 ALIGN 位，需显式强制右对齐。修复后正常范围 0~4095。

---

## 备注

- DSHOT 测试模式代码已清理，当前为正式运行版本
- DMA 采用循环模式，上电后即持续输出 DSHOT 信号
- MAVLink 发送使用 DMA 队列缓冲，非阻塞
