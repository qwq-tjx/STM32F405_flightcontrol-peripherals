/** @file
 *  @brief MAVLink comm protocol built from common.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_H
#define MAVLINK_H

#define MAVLINK_PRIMARY_XML_IDX 0

#ifndef MAVLINK_STX
#define MAVLINK_STX 253
#endif

#ifndef MAVLINK_ENDIAN
#define MAVLINK_ENDIAN MAVLINK_LITTLE_ENDIAN
#endif

#ifndef MAVLINK_ALIGNED_FIELDS
#define MAVLINK_ALIGNED_FIELDS 1
#endif

#ifndef MAVLINK_CRC_EXTRA
#define MAVLINK_CRC_EXTRA 1
#endif

#ifndef MAVLINK_COMMAND_24BIT
#define MAVLINK_COMMAND_24BIT 1
#endif

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#include "mavlink_types.h"

#define MAVLINK_SEND_UART_BYTES mavlink_send_uart_bytes
void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, int length);

#include "version.h"
#include "common.h"
#include "wit_c_sdk.h"
// ========== 基础 MAVLink 函数 ==========
void mavlink_send_heartbeat(void);
void mavlink_send_message(const mavlink_message_t *msg);
void mavlink_parse_byte(uint8_t data);
uint8_t mavlink_get_system_id(void);
uint8_t mavlink_get_component_id(void);
void mavlink_send_battery_voltage(float voltage_V);
// ========== IMU 数据发送函数 ==========
void mavlink_send_imu_attitude(WitImuData_t *imu_data);    // 发送角度 (ATTITUDE)
void mavlink_send_scaled_imu(WitImuData_t *imu_data);     // 发送加速度+角速度+磁力计 (HIGHRES_IMU)
// ========== 油门命令队列函数 ==========
uint8_t throttle_cmd_available(void);
uint8_t throttle_cmd_dequeue(uint8_t *channel, uint16_t *value);

#endif // MAVLINK_H
