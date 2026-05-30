#pragma once
// MESSAGE DEBUG_FLOAT_ARRAY PACKING
// MAVLink msgid=350, CRC_EXTRA=232
// 用途: 地面站发送目标姿态角 + 目标角速度给飞控

#define MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY 350

MAVPACKED(
typedef struct __mavlink_debug_float_array_t {
 uint64_t time_usec;                  /*< [8B] Timestamp (UNIX epoch or time since system boot)*/
 char name[10];                       /*< [10B] Name: "ATTI"/"RATE"/"DISA" */
 uint16_t array_id;                   /*< [2B] Array ID (保留=0)*/
 float data[10];                      /*< [40B] data[0..2]=角度(rad), data[3..5]=角速度(rad/s)*/
}) mavlink_debug_float_array_t;

#define MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN 60
#define MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_MIN_LEN 60
#define MAVLINK_MSG_ID_350_LEN 60
#define MAVLINK_MSG_ID_350_MIN_LEN 60

#define MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_CRC 232
#define MAVLINK_MSG_ID_350_CRC 232

#define MAVLINK_MSG_DEBUG_FLOAT_ARRAY_FIELD_NAME_LEN 10
#define MAVLINK_MSG_DEBUG_FLOAT_ARRAY_FIELD_DATA_LEN 10

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DEBUG_FLOAT_ARRAY { \
    350, \
    "DEBUG_FLOAT_ARRAY", \
    4, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_debug_float_array_t, time_usec) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 10, 8, offsetof(mavlink_debug_float_array_t, name) }, \
         { "array_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_debug_float_array_t, array_id) }, \
         { "data", NULL, MAVLINK_TYPE_FLOAT, 10, 20, offsetof(mavlink_debug_float_array_t, data) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DEBUG_FLOAT_ARRAY { \
    "DEBUG_FLOAT_ARRAY", \
    4, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_debug_float_array_t, time_usec) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 10, 8, offsetof(mavlink_debug_float_array_t, name) }, \
         { "array_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_debug_float_array_t, array_id) }, \
         { "data", NULL, MAVLINK_TYPE_FLOAT, 10, 20, offsetof(mavlink_debug_float_array_t, data) }, \
         } \
}
#endif

/**
 * @brief Pack a debug_float_array message
 * @param system_id ID of this system
 * @param component_id ID of this component
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (UNIX epoch or time since system boot)
 * @param name Name, for indicating control mode ("ATTI"/"RATE"/"DISA")
 * @param array_id Array ID (reserved, put 0)
 * @param data Array of 10 float values (only data[0..5] used for target values)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_debug_float_array_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, const char *name, uint16_t array_id, const float *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint16_t(buf, 18, array_id);
    _mav_put_char_array(buf, 8, name, 10);
    _mav_put_float_array(buf, 20, data, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN);
#else
    mavlink_debug_float_array_t packet;
    packet.time_usec = time_usec;
    packet.array_id = array_id;
    mav_array_memcpy(packet.name, name, sizeof(char)*10);
    mav_array_memcpy(packet.data, data, sizeof(float)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_CRC);
}

/**
 * @brief Decode a debug_float_array message into a struct
 * @param msg The message to decode
 * @param debug_float_array C-struct to decode the message contents into
 */
static inline void mavlink_msg_debug_float_array_decode(const mavlink_message_t* msg, mavlink_debug_float_array_t* debug_float_array)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    debug_float_array->time_usec = mavlink_msg_debug_float_array_get_time_usec(msg);
    debug_float_array->array_id = mavlink_msg_debug_float_array_get_array_id(msg);
    mavlink_msg_debug_float_array_get_name(msg, debug_float_array->name);
    mavlink_msg_debug_float_array_get_data(msg, debug_float_array->data);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN? msg->len : MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN;
        memset(debug_float_array, 0, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN);
    memcpy(debug_float_array, _MAV_PAYLOAD(msg), len);
#endif
}

/**
 * @brief Get field time_usec from debug_float_array message
 */
static inline uint64_t mavlink_msg_debug_float_array_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field name from debug_float_array message
 * @return pointer to the name field (10 chars, not null-terminated)
 */
static inline uint16_t mavlink_msg_debug_float_array_get_name(const mavlink_message_t* msg, char *name)
{
    return _MAV_RETURN_char_array(msg, name, 10,  8);
}

/**
 * @brief Get field array_id from debug_float_array message
 */
static inline uint16_t mavlink_msg_debug_float_array_get_array_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field data from debug_float_array message
 * @return pointer to data array (10 floats)
 */
static inline uint16_t mavlink_msg_debug_float_array_get_data(const mavlink_message_t* msg, float *data)
{
    return _MAV_RETURN_float_array(msg, data, 10,  20);
}
