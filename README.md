# FLYCONTROL STM32F405 飞控系统

基于 STM32F405 的四轴飞行器飞控系统，支持 **MAVLink 通信**、**DSHOT15 数字电调**、**WitMotion IMU 姿态传感器**、**串口油门调试**、**Mission Planner 上位机插件 (油门控制 + IMU 3D可视化)**。

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
| DEBUG_FLOAT_ARRAY | 350 | 接收目标姿态角 (rad) + 目标角速度 (rad/s) |
| MAV_CMD_SET_MESSAGE_INTERVAL | 511 | 设置消息发送频率 |
| MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES | 520 | 返回飞控版本信息 |

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

### ThrottleControlPlugin — 电机油门控制

Mission Planner 插件，提供可视化油门控制面板：

- 4 个独立 TrackBar + 数值输入框 (0~4095)
- 同步所有电机 / 紧急停止按钮
- 使用 `MAV_CMD_DO_SET_SERVO` (通道 9-12)

### ImuVisualizationPlugin — IMU 3D可视化

Mission Planner 插件，实时渲染飞控姿态：

- 3D 飞控模型实时旋转 (基于四元数)
- 自动请求 IMU 数据流 (ATTITUDE/HIGHRES_IMU/ALTITUDE 等)
- 航向角/高度/油门状态面板
- 读取 `SERVO_OUTPUT_RAW` 显示电机油门状态

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
│   ├── main.c           主程序
│   ├── main.h
│   └── control.h        控制模式 & 目标值声明
├── ThrottleControlPlugin.cs   Mission Planner 油门控制插件
└── ImuVisualizationPlugin.cs   Mission Planner IMU 3D可视化插件
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
