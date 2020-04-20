#pragma once
// MESSAGE SENSOR_WINGLET PACKING

#define MAVLINK_MSG_ID_SENSOR_WINGLET 302

MAVPACKED(
typedef struct __mavlink_sensor_winglet_t {
 uint64_t timestamp; /*<  Timestamp*/
 float x; /*<  X Rotational Quaternion*/
 float y; /*<  Y Rotational Quaternion*/
 float z; /*<  Z Rotational Quaternion*/
 float w; /*<  W Rotational Quaternion*/
 uint8_t id; /*<  Winglet ID*/
}) mavlink_sensor_winglet_t;

#define MAVLINK_MSG_ID_SENSOR_WINGLET_LEN 25
#define MAVLINK_MSG_ID_SENSOR_WINGLET_MIN_LEN 25
#define MAVLINK_MSG_ID_302_LEN 25
#define MAVLINK_MSG_ID_302_MIN_LEN 25

#define MAVLINK_MSG_ID_SENSOR_WINGLET_CRC 251
#define MAVLINK_MSG_ID_302_CRC 251



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SENSOR_WINGLET { \
    302, \
    "SENSOR_WINGLET", \
    6, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sensor_winglet_t, timestamp) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_sensor_winglet_t, id) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sensor_winglet_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sensor_winglet_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sensor_winglet_t, z) }, \
         { "w", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sensor_winglet_t, w) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SENSOR_WINGLET { \
    "SENSOR_WINGLET", \
    6, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sensor_winglet_t, timestamp) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_sensor_winglet_t, id) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sensor_winglet_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sensor_winglet_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sensor_winglet_t, z) }, \
         { "w", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sensor_winglet_t, w) }, \
         } \
}
#endif

/**
 * @brief Pack a sensor_winglet message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  Timestamp
 * @param id  Winglet ID
 * @param x  X Rotational Quaternion
 * @param y  Y Rotational Quaternion
 * @param z  Z Rotational Quaternion
 * @param w  W Rotational Quaternion
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sensor_winglet_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t id, float x, float y, float z, float w)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSOR_WINGLET_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 20, w);
    _mav_put_uint8_t(buf, 24, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENSOR_WINGLET_LEN);
#else
    mavlink_sensor_winglet_t packet;
    packet.timestamp = timestamp;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.w = w;
    packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENSOR_WINGLET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENSOR_WINGLET;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SENSOR_WINGLET_MIN_LEN, MAVLINK_MSG_ID_SENSOR_WINGLET_LEN, MAVLINK_MSG_ID_SENSOR_WINGLET_CRC);
}

/**
 * @brief Pack a sensor_winglet message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp  Timestamp
 * @param id  Winglet ID
 * @param x  X Rotational Quaternion
 * @param y  Y Rotational Quaternion
 * @param z  Z Rotational Quaternion
 * @param w  W Rotational Quaternion
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sensor_winglet_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t id,float x,float y,float z,float w)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSOR_WINGLET_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 20, w);
    _mav_put_uint8_t(buf, 24, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENSOR_WINGLET_LEN);
#else
    mavlink_sensor_winglet_t packet;
    packet.timestamp = timestamp;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.w = w;
    packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENSOR_WINGLET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENSOR_WINGLET;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SENSOR_WINGLET_MIN_LEN, MAVLINK_MSG_ID_SENSOR_WINGLET_LEN, MAVLINK_MSG_ID_SENSOR_WINGLET_CRC);
}

/**
 * @brief Encode a sensor_winglet struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sensor_winglet C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sensor_winglet_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sensor_winglet_t* sensor_winglet)
{
    return mavlink_msg_sensor_winglet_pack(system_id, component_id, msg, sensor_winglet->timestamp, sensor_winglet->id, sensor_winglet->x, sensor_winglet->y, sensor_winglet->z, sensor_winglet->w);
}

/**
 * @brief Encode a sensor_winglet struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sensor_winglet C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sensor_winglet_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sensor_winglet_t* sensor_winglet)
{
    return mavlink_msg_sensor_winglet_pack_chan(system_id, component_id, chan, msg, sensor_winglet->timestamp, sensor_winglet->id, sensor_winglet->x, sensor_winglet->y, sensor_winglet->z, sensor_winglet->w);
}

/**
 * @brief Send a sensor_winglet message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  Timestamp
 * @param id  Winglet ID
 * @param x  X Rotational Quaternion
 * @param y  Y Rotational Quaternion
 * @param z  Z Rotational Quaternion
 * @param w  W Rotational Quaternion
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sensor_winglet_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t id, float x, float y, float z, float w)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSOR_WINGLET_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 20, w);
    _mav_put_uint8_t(buf, 24, id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_WINGLET, buf, MAVLINK_MSG_ID_SENSOR_WINGLET_MIN_LEN, MAVLINK_MSG_ID_SENSOR_WINGLET_LEN, MAVLINK_MSG_ID_SENSOR_WINGLET_CRC);
#else
    mavlink_sensor_winglet_t packet;
    packet.timestamp = timestamp;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.w = w;
    packet.id = id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_WINGLET, (const char *)&packet, MAVLINK_MSG_ID_SENSOR_WINGLET_MIN_LEN, MAVLINK_MSG_ID_SENSOR_WINGLET_LEN, MAVLINK_MSG_ID_SENSOR_WINGLET_CRC);
#endif
}

/**
 * @brief Send a sensor_winglet message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sensor_winglet_send_struct(mavlink_channel_t chan, const mavlink_sensor_winglet_t* sensor_winglet)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sensor_winglet_send(chan, sensor_winglet->timestamp, sensor_winglet->id, sensor_winglet->x, sensor_winglet->y, sensor_winglet->z, sensor_winglet->w);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_WINGLET, (const char *)sensor_winglet, MAVLINK_MSG_ID_SENSOR_WINGLET_MIN_LEN, MAVLINK_MSG_ID_SENSOR_WINGLET_LEN, MAVLINK_MSG_ID_SENSOR_WINGLET_CRC);
#endif
}

#if MAVLINK_MSG_ID_SENSOR_WINGLET_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sensor_winglet_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t id, float x, float y, float z, float w)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 20, w);
    _mav_put_uint8_t(buf, 24, id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_WINGLET, buf, MAVLINK_MSG_ID_SENSOR_WINGLET_MIN_LEN, MAVLINK_MSG_ID_SENSOR_WINGLET_LEN, MAVLINK_MSG_ID_SENSOR_WINGLET_CRC);
#else
    mavlink_sensor_winglet_t *packet = (mavlink_sensor_winglet_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->w = w;
    packet->id = id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_WINGLET, (const char *)packet, MAVLINK_MSG_ID_SENSOR_WINGLET_MIN_LEN, MAVLINK_MSG_ID_SENSOR_WINGLET_LEN, MAVLINK_MSG_ID_SENSOR_WINGLET_CRC);
#endif
}
#endif

#endif

// MESSAGE SENSOR_WINGLET UNPACKING


/**
 * @brief Get field timestamp from sensor_winglet message
 *
 * @return  Timestamp
 */
static inline uint64_t mavlink_msg_sensor_winglet_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field id from sensor_winglet message
 *
 * @return  Winglet ID
 */
static inline uint8_t mavlink_msg_sensor_winglet_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field x from sensor_winglet message
 *
 * @return  X Rotational Quaternion
 */
static inline float mavlink_msg_sensor_winglet_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field y from sensor_winglet message
 *
 * @return  Y Rotational Quaternion
 */
static inline float mavlink_msg_sensor_winglet_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field z from sensor_winglet message
 *
 * @return  Z Rotational Quaternion
 */
static inline float mavlink_msg_sensor_winglet_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field w from sensor_winglet message
 *
 * @return  W Rotational Quaternion
 */
static inline float mavlink_msg_sensor_winglet_get_w(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a sensor_winglet message into a struct
 *
 * @param msg The message to decode
 * @param sensor_winglet C-struct to decode the message contents into
 */
static inline void mavlink_msg_sensor_winglet_decode(const mavlink_message_t* msg, mavlink_sensor_winglet_t* sensor_winglet)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sensor_winglet->timestamp = mavlink_msg_sensor_winglet_get_timestamp(msg);
    sensor_winglet->x = mavlink_msg_sensor_winglet_get_x(msg);
    sensor_winglet->y = mavlink_msg_sensor_winglet_get_y(msg);
    sensor_winglet->z = mavlink_msg_sensor_winglet_get_z(msg);
    sensor_winglet->w = mavlink_msg_sensor_winglet_get_w(msg);
    sensor_winglet->id = mavlink_msg_sensor_winglet_get_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SENSOR_WINGLET_LEN? msg->len : MAVLINK_MSG_ID_SENSOR_WINGLET_LEN;
        memset(sensor_winglet, 0, MAVLINK_MSG_ID_SENSOR_WINGLET_LEN);
    memcpy(sensor_winglet, _MAV_PAYLOAD(msg), len);
#endif
}
