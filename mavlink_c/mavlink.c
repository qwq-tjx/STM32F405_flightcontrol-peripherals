#include "mavlink.h"
#include "Serial.h"
#include "Delay.h"
#include "DSHOT.h"
#include "wit_c_sdk.h"
#include "LED.h"

// 临界区保护宏（与 DSHOT 保持一致）
#define THROTTLE_ENTER_CRITICAL()  do { __set_PRIMASK(1); } while(0)
#define THROTTLE_EXIT_CRITICAL()   do { __set_PRIMASK(0); } while(0)

// 定义圆周率常量
#ifndef M_PI
#define M_PI  3.14159265358979323846f
#endif

// ========== 油门命令环形缓冲区 ==========
#define THROTTLE_CMD_QUEUE_SIZE  32

typedef struct {
    uint8_t channel;
    uint16_t value;
} throttle_cmd_t;

static throttle_cmd_t cmd_queue[THROTTLE_CMD_QUEUE_SIZE];
static volatile uint16_t cmd_head = 0;
static volatile uint16_t cmd_tail = 0;

uint8_t throttle_cmd_enqueue(uint8_t channel, uint16_t value)
{
    uint16_t next_head;
    uint8_t result;
    
    THROTTLE_ENTER_CRITICAL();
    
    next_head = (cmd_head + 1) % THROTTLE_CMD_QUEUE_SIZE;
    
    if (next_head == cmd_tail) {
        result = 0;
    } else {
        cmd_queue[cmd_head].channel = channel;
        cmd_queue[cmd_head].value = value;
        cmd_head = next_head;
        result = 1;
    }
    
    THROTTLE_EXIT_CRITICAL();
    
    return result;
}

uint8_t throttle_cmd_dequeue(uint8_t *channel, uint16_t *value)
{
    uint8_t result;
    
    THROTTLE_ENTER_CRITICAL();
    
    if (cmd_head == cmd_tail) {
        result = 0;
    } else {
        *channel = cmd_queue[cmd_tail].channel;
        *value = cmd_queue[cmd_tail].value;
        cmd_tail = (cmd_tail + 1) % THROTTLE_CMD_QUEUE_SIZE;
        result = 1;
    }
    
    THROTTLE_EXIT_CRITICAL();
    
    return result;
}

uint8_t throttle_cmd_available(void)
{
    int16_t count;
    
    THROTTLE_ENTER_CRITICAL();
    count = (cmd_head - cmd_tail + THROTTLE_CMD_QUEUE_SIZE) % THROTTLE_CMD_QUEUE_SIZE;
    THROTTLE_EXIT_CRITICAL();
    
    return (uint8_t)count;
}

// MAVLink 系统ID配置
mavlink_system_t mavlink_system = {
    .sysid = 1,
    .compid = 1
};

// ========== IMU 数据发送间隔配置 ==========
static uint32_t attitude_interval_us = 100000;  // ATTITUDE 默认 10Hz
static uint32_t imu_hres_interval_us = 100000; // HIGHRES_IMU 默认 10Hz
static volatile uint8_t attitude_stream_enabled = 1;

// ========== 发送 IMU 姿态数据 (ATTITUDE 消息) ==========
void mavlink_send_imu_attitude(WitImuData_t *imu_data)
{
    mavlink_message_t msg;

    float roll_rad = imu_data->angle[0] * (M_PI / 180.0f);
    float pitch_rad = imu_data->angle[1] * (M_PI / 180.0f);
    float yaw_rad = imu_data->angle[2] * (M_PI / 180.0f);

    uint32_t time_ms = delay_ms_count_get();

    // ATTITUDE消息也包含角速度（rollspeed, pitchspeed, yawspeed）
    // 单位: rad/s
    float rollspeed = imu_data->gyro[0];
    float pitchspeed = imu_data->gyro[1];
    float yawspeed = imu_data->gyro[2];

    mavlink_msg_attitude_pack(
        mavlink_system.sysid,
        mavlink_system.compid,  // 恢复为1，保持与心跳包一致
        &msg,
        time_ms,
        roll_rad,
        pitch_rad,
        yaw_rad,
        rollspeed,
        pitchspeed,
        yawspeed
    );

    mavlink_send_message(&msg);
}

// ========== 发送 IMU 传感器原始数据 (HIGHRES_IMU) ==========
void mavlink_send_scaled_imu(WitImuData_t *imu_data)
{
    mavlink_message_t msg;
    uint64_t time_usec = delay_ms_count_get() * 1000ULL;

    // 加速度: g -> m/s^2
    float acc_x = imu_data->acc[0] * 9.80665f;
    float acc_y = imu_data->acc[1] * 9.80665f;
    float acc_z = imu_data->acc[2] * 9.80665f;

    // 角速度: rad/s
    float gyro_x = imu_data->gyro[0];
    float gyro_y = imu_data->gyro[1];
    float gyro_z = imu_data->gyro[2];

    // 磁场: uT -> Gauss
    float mag_x = imu_data->mag[0] / 100.0f;
    float mag_y = imu_data->mag[1] / 100.0f;
    float mag_z = imu_data->mag[2] / 100.0f;

    // 芯片温度 (°C)
    float temperature = imu_data->temperature;

    // 发送HIGHRES_IMU消息 (ID=105)
    mavlink_msg_highres_imu_pack(
        mavlink_system.sysid,
        mavlink_system.compid,  // 使用系统默认compid=1
        &msg,
        time_usec,
        acc_x, acc_y, acc_z,
        gyro_x, gyro_y, gyro_z,
        mag_x, mag_y, mag_z,
        0, 0, 0,
        temperature,
        0x1FFF
    );

    mavlink_send_message(&msg);
}

// ========== 发送四元数姿态数据 (ATTITUDE_QUATERNION 消息) ==========
void mavlink_send_imu_quaternion(WitImuData_t *imu_data)
{
    mavlink_message_t msg;
    uint32_t time_ms = delay_ms_count_get();

    // 四元数: quat[0]=w, quat[1]=x, quat[2]=y, quat[3]=z
    // MAVLink ATTITUDE_QUATERNION: q1=w, q2=x, q3=y, q4=z
    float qw = imu_data->quat[0];
    float qx = imu_data->quat[1];
    float qy = imu_data->quat[2];
    float qz = imu_data->quat[3];

    // 角速度 (deg/s, 直接使用原始值)
    float rollspeed = imu_data->gyro[0];
    float pitchspeed = imu_data->gyro[1];
    float yawspeed = imu_data->gyro[2];

    mavlink_msg_attitude_quaternion_pack(
        mavlink_system.sysid,
        mavlink_system.compid,
        &msg,
        time_ms,
        qw, qx, qy, qz,
        rollspeed, pitchspeed, yawspeed
    );

    mavlink_send_message(&msg);
}

// ========== 发送气压数据 (SCALED_PRESSURE 消息) ==========
void mavlink_send_imu_pressure(WitImuData_t *imu_data)
{
    mavlink_message_t msg;
    uint32_t time_ms = delay_ms_count_get();

    mavlink_msg_scaled_pressure_pack(
        mavlink_system.sysid,
        mavlink_system.compid,
        &msg,
        time_ms,
        imu_data->pressure,  // 绝对气压 (hPa)
        0.0f,                // 差压 (无此传感器)
        0                    // 温度 (0.01°C, 未知)
    );

    mavlink_send_message(&msg);
}

// ========== 发送 VFR_HUD (Mission Planner 主屏HUD高度、航向、速度) ==========
void mavlink_send_vfr_hud(WitImuData_t *imu_data)
{
    mavlink_message_t msg;
    float alt = imu_data->altitude;          // 气压高度 (m)
    int16_t heading = (int16_t)(imu_data->angle[2]); // 偏航角 (0~360)

    // 包裹航向到 0~360
    while (heading < 0) heading += 360;
    heading = heading % 360;

    mavlink_msg_vfr_hud_pack(
        mavlink_system.sysid,
        mavlink_system.compid,
        &msg,
        0.0f,           // airspeed (无空速)
        0.0f,           // groundspeed (无GPS)
        heading,        // 偏航角
        0,              // throttle (百分比, 无此数据)
        alt,            // 高度 (MSL, 米)
        0.0f            // climb rate (无垂直速度传感器)
    );

    mavlink_send_message(&msg);
}

// ========== 发送高度数据 (ALTITUDE 消息) ==========
void mavlink_send_imu_altitude(WitImuData_t *imu_data)
{
    mavlink_message_t msg;
    uint64_t time_us = delay_ms_count_get() * 1000ULL;

    mavlink_msg_altitude_pack(
        mavlink_system.sysid,
        mavlink_system.compid,
        &msg,
        time_us,
        imu_data->altitude,  // altitude_monotonic (气压高度)
        imu_data->altitude,  // altitude_amsl (AMSL海拔)
        imu_data->altitude,  // altitude_local (本地高度)
        imu_data->altitude,  // altitude_relative (相对起飞点高度)
        -1000.0f,            // altitude_terrain (无地形数据)
        0.0f                 // bottom_clearance (无激光测距)
    );

    mavlink_send_message(&msg);
}

// ========== 主动发送 IMU 数据的主函数 (定时器中断中调用) ==========
// 交错发送: 偶数轮发气压+高度, 奇数轮发ScaledIMU+姿态
// 每组 ~70B, 115200下 6ms → 100Hz循环有充足余量, 每条消息有效50Hz
void mavlink_send_imu_periodic(void)
{
    static uint32_t last_attitude_time = 0;
    static uint32_t last_heartbeat_time = 0;
    static uint8_t  msg_phase = 0;
    uint32_t current_time = delay_ms_count_get();

    // ========== 心跳包：每秒1次 ==========
    if (current_time - last_heartbeat_time >= 1000) {
        last_heartbeat_time = current_time;
        mavlink_send_heartbeat();
        LED_heartbeat_sync();  // LED闪烁与心跳同频同步
    }

    if (attitude_stream_enabled) {
        WitImuData_t imu_data;
        WitImu_GetData(&imu_data);

        uint32_t attitude_interval_ms = attitude_interval_us / 1000;
        if (current_time - last_attitude_time >= attitude_interval_ms) {
            last_attitude_time = current_time;

            if (msg_phase == 0) {
               // mavlink_send_vfr_hud(&imu_data);        // HUD (航向+高度)
                mavlink_send_imu_pressure(&imu_data);   // 气压
                mavlink_send_imu_altitude(&imu_data);   // 高度
				mavlink_send_imu_quaternion(&imu_data);
                msg_phase = 1;
            } else {
                mavlink_send_scaled_imu(&imu_data);
                mavlink_send_imu_attitude(&imu_data);
                msg_phase = 0;
            }
        }
    }
}

// ========== 底层串口发送字节（MAVLink 库回调用，使用DMA）==========
void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, int length)
{
    (void)chan;
    USART2_DMA_Send((uint8_t*)ch, (uint16_t)length);  // 非阻塞DMA发送
}

// ========== 发送 MAVLink 消息（通用）==========
void mavlink_send_message(const mavlink_message_t *msg)
{
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, msg);
    mavlink_send_uart_bytes(MAVLINK_COMM_0, buffer, len);
}

// ========== 发送心跳包 ==========
void mavlink_send_heartbeat(void)
{
    mavlink_message_t msg;

    mavlink_msg_heartbeat_pack_chan(
        mavlink_system.sysid,
        mavlink_system.compid,
        MAVLINK_COMM_0,
        &msg,
        MAV_TYPE_GENERIC,
        MAV_AUTOPILOT_GENERIC,
        0,
        0,
        MAV_STATE_ACTIVE
    );

    mavlink_send_message(&msg);
}

// ========== 处理接收到的MAVLink消息 ==========
static void process_mavlink_message(mavlink_message_t *msg)
{
    switch (msg->msgid) {
        
        case MAVLINK_MSG_ID_HEARTBEAT:
        {
            break;
        }
        
        // ========== 直接设置电机输出 (SET_ACTUATOR_CONTROL_TARGET) ==========
        case MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET:
        {
            mavlink_set_actuator_control_target_t act;
            mavlink_msg_set_actuator_control_target_decode(msg, &act);
            
            // 检查目标是否匹配
            if (act.target_system != mavlink_system.sysid && act.target_system != 0) {
                break;
            }
            
            // controls[0-3]: roll, pitch, yaw, throttle (归一化 -1 ~ 1)
            // controls[4-7]: 其他通道
            for (uint8_t i = 0; i < 4; i++) {
                float v = act.controls[i];
                if (v < 0) v = 0;  // 负值设为0 (电机不支持反转)
                uint16_t throttle = (uint16_t)(v * 4095);
                if (throttle > 4095) throttle = 4095;
                throttle_cmd_enqueue(i, throttle);
            }
            break;
        }
        
        // ========== RC通道覆盖消息 (Mission Planner手动控制) ==========
        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
        {
            mavlink_rc_channels_override_t rc;
            mavlink_msg_rc_channels_override_decode(msg, &rc);
            
            // 检查是否针对本系统 (target_system=0 表示广播, 所有系统都接收)
            if (rc.target_system != mavlink_system.sysid && rc.target_system != 0) {
                break;
            }
            
            // rc.chan1_raw ~ rc.chan8_raw 是PWM值 (通常 1000-2000)
            // 将其转换为 DShot 油门值 (0-4095)
            
            // 通道1-4 对应电机0-3
            uint8_t updated = 0;
            if (rc.chan1_raw != UINT16_MAX && rc.chan1_raw > 900) {
                uint16_t t = (rc.chan1_raw - 1000) * 4096 / 1000;
                if (t > 4095) t = 4095;
                throttle_cmd_enqueue(0, t);
                updated = 1;
            }
            if (rc.chan2_raw != UINT16_MAX && rc.chan2_raw > 900) {
                uint16_t t = (rc.chan2_raw - 1000) * 4096 / 1000;
                if (t > 4095) t = 4095;
                throttle_cmd_enqueue(1, t);
                updated = 1;
            }
            if (rc.chan3_raw != UINT16_MAX && rc.chan3_raw > 900) {
                uint16_t t = (rc.chan3_raw - 1000) * 4096 / 1000;
                if (t > 4095) t = 4095;
                throttle_cmd_enqueue(2, t);
                updated = 1;
            }
            if (rc.chan4_raw != UINT16_MAX && rc.chan4_raw > 900) {
                uint16_t t = (rc.chan4_raw - 1000) * 4096 / 1000;
                if (t > 4095) t = 4095;
                throttle_cmd_enqueue(3, t);
                updated = 1;
            }
            
            // 调试输出 (可以用串口查看)
            if (updated) {
                Serial_Printf("[RC] RC: %d %d %d %d\r\n", 
                    rc.chan1_raw, rc.chan2_raw, rc.chan3_raw, rc.chan4_raw);
            }
            break;
        }
        
        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
        {
            mavlink_request_data_stream_t req;
            mavlink_msg_request_data_stream_decode(msg, &req);
            
            if(req.target_system == mavlink_system.sysid || req.target_system == 0) {
                if(req.start_stop == 1) {
                    attitude_stream_enabled = 1;
                } else {
                    attitude_stream_enabled = 0;
                }
            }
            break;
        }
        
        case MAVLINK_MSG_ID_COMMAND_LONG:
        {
            mavlink_command_long_t cmd;
            mavlink_msg_command_long_decode(msg, &cmd);
            
            if (cmd.target_system == mavlink_system.sysid || cmd.target_system == 0) {
                
                switch (cmd.command) {
                    
                    case 183:  // MAV_CMD_DO_SET_SERVO
                    {
                        uint8_t servo_channel = (uint8_t)cmd.param1;
                        uint16_t servo_value = (uint16_t)cmd.param2;
                        uint8_t result = MAV_RESULT_ACCEPTED;
                        
                        uint8_t motor_idx = 0xFF;
                        
                        // 支持两种通道映射：1-4 或 9-12
                        if (servo_channel >= 1 && servo_channel <= 4) {
                            motor_idx = servo_channel - 1;  // 通道1-4 → 电机0-3
                        } else if (servo_channel >= 9 && servo_channel <= 12) {
                            motor_idx = servo_channel - 9;  // 通道9-12 → 电机0-3
                        }
                        
                        if (motor_idx < 4) {
                            if (servo_value > 4095) servo_value = 4095;
                            throttle_cmd_enqueue(motor_idx, servo_value);
                            // 计算DSHOT CRC
                            uint8_t g1 = (servo_value >> 8) & 0x0F;
                            uint8_t g2 = (servo_value >> 4) & 0x0F;
                            uint8_t g3 = servo_value & 0x0F;
                            uint8_t crc = (g1 + g2 + g3) & 0x0F;
                            uint16_t frame = (servo_value << 4) | crc;
                            // 打印信息
                            Serial_Printf("[RCV] ch=%d throttle=%d motor=%d\r\n", servo_channel, servo_value, motor_idx);
                          //  Serial_Printf("[RCV] DSHOT frame: ");
							//打印油门值二进制
                            for (int i = 15; i >= 0; i--) Serial_Printf("%d", (frame >> i) & 1);
                            Serial_Printf("\r\n");
							
                        } else if (servo_channel == 0xFF) {
                            for (uint8_t i = 0; i < 4; i++) {
                                throttle_cmd_enqueue(i, servo_value);
                            }
                            Serial_Printf("[RCV] all sync=%d\r\n", servo_value);
                        }
                        
                        {
                            mavlink_message_t ack_msg;
                            mavlink_msg_command_ack_pack_chan(
                                mavlink_system.sysid, mavlink_system.compid,
                                MAVLINK_COMM_0, &ack_msg, cmd.command, result,
                                0, 0, cmd.target_system, cmd.target_component);
                            mavlink_send_message(&ack_msg);
                        }
                        break;
                    }
                    
//                    case 512:  // MAV_CMD_REQUEST_MESSAGE
//                    {
//                        uint16_t requested_msg_id = (uint16_t)cmd.param1;
//                        WitImuData_t imu_data;
//                        WitImu_GetData(&imu_data);
//                        uint8_t result = MAV_RESULT_ACCEPTED;

//                        switch (requested_msg_id) {
//                            case MAVLINK_MSG_ID_ATTITUDE:
//                                mavlink_send_imu_attitude(&imu_data);
//                                break;
//                            case MAVLINK_MSG_ID_HIGHRES_IMU:
//                            case MAVLINK_MSG_ID_SCALED_IMU:
//                                mavlink_send_scaled_imu(&imu_data);
//                                break;
//                            case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
//                                mavlink_send_imu_quaternion(&imu_data);
//                                break;
//                            case MAVLINK_MSG_ID_SCALED_PRESSURE:
//                                mavlink_send_imu_pressure(&imu_data);
//                                break;
//                            case MAVLINK_MSG_ID_ALTITUDE:
//                                mavlink_send_imu_altitude(&imu_data);
//                                break;
//                            case MAVLINK_MSG_ID_VFR_HUD:
//                                mavlink_send_vfr_hud(&imu_data);
//                                break;
//                            default:
//                                result = MAV_RESULT_UNSUPPORTED;
//                                break;
//                        }
//                        {
//                            mavlink_message_t ack_msg;
//                            mavlink_msg_command_ack_pack_chan(
//                                mavlink_system.sysid, mavlink_system.compid,
//                                MAVLINK_COMM_0, &ack_msg, cmd.command, result,
//                                0, 0, cmd.target_system, cmd.target_component);
//                            mavlink_send_message(&ack_msg);
//                        }
//                        break;
//                    }
                    
                    case 511:  // MAV_CMD_SET_MESSAGE_INTERVAL
                    {
                        uint16_t msg_id = (uint16_t)cmd.param1;
                        int32_t interval_us = (int32_t)cmd.param2;
                        uint8_t result = MAV_RESULT_ACCEPTED;
                        
                        if (interval_us > 0) {
                            switch (msg_id) {
                                case MAVLINK_MSG_ID_ATTITUDE:
                                case MAVLINK_MSG_ID_ALTITUDE:
                                case MAVLINK_MSG_ID_SCALED_PRESSURE:
                                case MAVLINK_MSG_ID_VFR_HUD:
                                    attitude_interval_us = (uint32_t)interval_us;
                                    attitude_stream_enabled = 1;
                                    break;
                                case MAVLINK_MSG_ID_HIGHRES_IMU:
                                case MAVLINK_MSG_ID_SCALED_IMU:
                                    imu_hres_interval_us = (uint32_t)interval_us;
                                    attitude_stream_enabled = 1;
                                    break;
                                default:
                                    result = MAV_RESULT_UNSUPPORTED;
                                    break;
                            }
                        } else if (interval_us == -1) {
                            attitude_stream_enabled = 0;
                        } else {
                            result = MAV_RESULT_DENIED;
                        }
                        {
                            mavlink_message_t ack_msg;
                            mavlink_msg_command_ack_pack_chan(
                                mavlink_system.sysid, mavlink_system.compid,
                                MAVLINK_COMM_0, &ack_msg, cmd.command, result,
                                0, 0, cmd.target_system, cmd.target_component);
                            mavlink_send_message(&ack_msg);
                        }
                        break;
                    }
                    
                    case 520:  // MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES
                    {
                        uint8_t custom_version[8] = {0};
                        uint8_t uid2[18] = {0};
                        mavlink_msg_autopilot_version_send(
                            MAVLINK_COMM_0,
                            0, 0, 0, 0, 0,
                            custom_version, custom_version, custom_version,
                            0, 0, 0, uid2);
                        {
                            mavlink_message_t ack_msg;
                            mavlink_msg_command_ack_pack_chan(
                                mavlink_system.sysid, mavlink_system.compid,
                                MAVLINK_COMM_0, &ack_msg, cmd.command,
                                MAV_RESULT_ACCEPTED,
                                0, 0, cmd.target_system, cmd.target_component);
                            mavlink_send_message(&ack_msg);
                        }
                        break;
                    }
                    
                    default:
                        break;
                }
            }
            break;
        }
        
        default:
            break;
    }
}

// ========== 逐字节解析MAVLink数据 ==========
void mavlink_parse_byte(uint8_t data)
{
    static mavlink_message_t msg;
    static mavlink_status_t status;
    
    if (mavlink_parse_char(MAVLINK_COMM_0, data, &msg, &status)) {
        process_mavlink_message(&msg);
    }
}

// ========== 发送电池电压 (SYS_STATUS 消息) ==========
void mavlink_send_battery_voltage(float voltage_V)
{
    mavlink_message_t msg;
    
    uint16_t voltage_mV = (uint16_t)(voltage_V * 1000);
    if(voltage_mV > 65535) voltage_mV = 65535;
    
    uint8_t battery_remaining = 0;
    if(voltage_V >= 24.0f) {
        battery_remaining = 100;
    } 
    else if(voltage_V <= 20.0f) {
        battery_remaining = 0;
    } 
    else {
        battery_remaining = (uint8_t)((voltage_V - 20.0f) * 100.0f / 4.0f);
    }
    
    mavlink_msg_sys_status_pack_chan(
        mavlink_system.sysid,
        mavlink_system.compid,
        MAVLINK_COMM_0,
        &msg,
        0, 0, 0,
        0,
        voltage_mV,
        0,
        battery_remaining,
        0, 0, 0, 0, 0, 0
    );
    
    mavlink_send_message(&msg);
}

uint8_t mavlink_get_system_id(void)
{
    return mavlink_system.sysid;
}

uint8_t mavlink_get_component_id(void)
{
    return mavlink_system.compid;
}
