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

| 消息 | 频率 | 说明 |
|------|------|------|
| HEARTBEAT | 1Hz | 心跳包 |
| ATTITUDE | ~50Hz | 姿态数据 (Roll/Pitch/Yaw) |
| ATTITUDE_QUATERNION | ~50Hz | 四元数姿态 |
| HIGHRES_IMU | ~50Hz | 加速度/角速度/磁力计 |
| SCALED_PRESSURE | ~50Hz | 气压数据 |
| ALTITUDE | ~50Hz | 高度数据 |
| SERVO_OUTPUT_RAW | 2Hz | 4路电机油门回读 |
| SYS_STATUS | 1Hz | 电池电压 |

> 消息交错发送 (偶数轮发气压+高度+四元数, 奇数轮发IMU+姿态), 每组约70B,
> 115200下 6ms → 100Hz 循环有充足余量。

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

### PID 默认值

| 轴 | Kp | Ki | Kd | T |
|----|-----|-----|-----|------|
| Roll | 20 | 200 | 0 | 0.50s |
| Pitch | 20 | 200 | 0 | |
| Yaw | 25 | 250 | 0 | |

### 控制模式与目标值

接收 `DEBUG_FLOAT_ARRAY` 时自动进入姿态控制模式：

| 数据索引 | 对应变量 | 含义 | 限幅范围 |
|----------|----------|------|----------|
| data[0] | target_angle[0] | Roll 目标角 (rad) | ±3.15 (±180.5°) |
| data[1] | target_angle[1] | Pitch 目标角 (rad) | ±3.15 (±180.5°) |
| data[2] | target_angle[2] | Yaw 目标角 (rad) | ±3.15 (±180.5°) |
| data[3] | target_gyro[0] | X 轴目标角速度 (rad/s) | ±34.9 (±2000°/s) |
| data[4] | target_gyro[1] | Y 轴目标角速度 (rad/s) | ±34.9 (±2000°/s) |
| data[5] | target_gyro[2] | Z 轴目标角速度 (rad/s) | ±34.9 (±2000°/s) |

控制模式枚举 (`control.h`)：
- `CTRL_MODE_DISABLED (0)` — 无控制，使用串口油门
- `CTRL_MODE_ATTITUDE (1)` — 姿态控制 (外环角度+内环角速度)
- `CTRL_MODE_RATE (2)` — 角速度控制 (仅内环)

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
  systick → USART1(调试) → Debug_Init
         → UART5 + WitMotion IMU
         → USART2 + MAVLink
         → DSHOT(TIM8+DMA)
         → ADC 电池检测
         → TIM7 (100Hz MAVLink 发送)
         → TIM6 (1Hz 电池电压)

主循环:
  while(1):
    1. 处理油门命令缓冲区
    2. 检测串口油门更新标志
    3. 更新 DSHOT DMA 缓冲区
    4. 输出调试信息
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

**电池电压显示**
- 实时显示母线电压 (V)、单节电压 (6S, 保留2位小数)、剩余电量 (%)
- 数据来源: `SYS_STATUS` (msgid=1) 的 `voltage_battery` + `battery_remaining`
- 单节 < 3.65V → 醒目红色低电压预警
- 超时 5 秒无数据 → 灰色提示

### TargetThrottlePlugin — 目标油门控件

独立目标油门输入窗口，范围为 0~10 kg。

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

连接 PA9 (TX) / PA10 (RX)，波特率 115200。

### 串口油门控制

通过串口助手发送油门指令，帧格式如下：

```
@xxx,xxx,xxx,xxx\r\n
```

- 帧头 `@`，帧尾 `\r\n`
- 4 个逗号分隔值，范围 0~4095（超过自动限幅）
- 示例：`@1000,2000,500,0\r\n`

### 启动信息示例

```
========================================
       FLYCONTROL STM32F405 System      
========================================
System Clock: 168 MHz
DSHOT Mode: DSHOT15 (PC6-PC9)
MAVLink: Ready (USART2)
IMU: WitMotion (UART5)
========================================
Waiting for RC commands...
```

---

## 工程目录结构

```
├── CMSIS/               CMSIS 内核头文件 + STM32F4xx 头文件
├── Hardware/            硬件驱动模块
│   ├── DSHOT.c/.h       DSHOT 电调驱动
│   ├── Serial.c/.h      串口驱动 (USART1/2, UART5)
│   ├── TIM.c/.h         定时器 + DMA 配置
│   ├── adc.c/.h         ADC 电池检测
│   ├── Delay.c/.h       SysTick 延时
│   ├── debug_output.c/.h 调试输出
│   ├── wit_c_sdk.c/.h   WitMotion IMU SDK
│   ├── REG.h            IMU 寄存器映射
│   └── imu_simulator.c/.h IMU 模拟器 (测试用)
├── Library/             STM32F4xx 标准外设库
├── mavlink_c/           MAVLink v2 协议栈
│   ├── mavlink.c        自定义 MAVLink 处理逻辑
│   └── common/          所有 MAVLink 消息头文件
├── Project/             Keil 工程文件
├── Startup/             startup_stm32f405xx.s
├── User/                用户应用
│   ├── main.c / main.h   主程序
│   ├── control.c / control.h  控制模式 & 目标值
│   ├── drone_control.c / drone_control.h  双环 PID 姿态控制算法
├── ThrottleControlPlugin.cs   Mission Planner 综合调参插件
├── TargetThrottlePlugin/      目标油门独立插件
└── ImuVisualizationPlugin.cs   Mission Planner IMU 可视化插件
```

---

## 关键技术细节

### DMA 循环模式

DSHOT 使用 DMA 循环模式 (`DMA_Mode_Circular`)，每个 TIM8 PWM 周期自动触发一次 DMA 传输，将缓冲区数据写入 CCR 寄存器，实现连续 DSHOT 脉冲输出。

### 临界区保护

油门值更新使用 `__set_PRIMASK(1/0)` 关全局中断保护，防止主循环和 MAVLink 中断同时写入 `current_throttle[]`。

### 电池电压计算

24V 电池经过 10K/1K 分压 (11:1) 后接入 PC5：
```
Vbat = ADC值 × Vref / 4096 × 11
```

---

## 备注

- DSHOT 测试模式代码已清理，当前为正式运行版本
- DMA 采用循环模式，上电后即持续输出 DSHOT 信号
- MAVLink 发送使用 DMA 队列缓冲，非阻塞
