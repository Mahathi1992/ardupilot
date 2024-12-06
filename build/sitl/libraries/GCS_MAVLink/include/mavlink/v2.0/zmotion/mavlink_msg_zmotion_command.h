#pragma once
// MESSAGE ZMOTION_COMMAND PACKING

#define MAVLINK_MSG_ID_ZMOTION_COMMAND 58001


typedef struct __mavlink_zmotion_command_t {
 uint32_t custom_param1; /*<  Custom Param 1 for the command.*/
 uint32_t custom_param2; /*<  Custom Param 2 for the command.*/
 uint32_t custom_param3; /*<  Custom Param 3 for the command.*/
 uint32_t custom_param4; /*<  Custom Param 4 for the command.*/
 uint16_t seq_identifier; /*<  A unique identifier to associate the command with the response.*/
 uint8_t zmotion_cmd_type; /*<  ZMotion Command Type*/
 uint8_t actual_cmd; /*<  Actual Command*/
} mavlink_zmotion_command_t;

#define MAVLINK_MSG_ID_ZMOTION_COMMAND_LEN 20
#define MAVLINK_MSG_ID_ZMOTION_COMMAND_MIN_LEN 20
#define MAVLINK_MSG_ID_58001_LEN 20
#define MAVLINK_MSG_ID_58001_MIN_LEN 20

#define MAVLINK_MSG_ID_ZMOTION_COMMAND_CRC 68
#define MAVLINK_MSG_ID_58001_CRC 68



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ZMOTION_COMMAND { \
    58001, \
    "ZMOTION_COMMAND", \
    7, \
    {  { "zmotion_cmd_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_zmotion_command_t, zmotion_cmd_type) }, \
         { "actual_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_zmotion_command_t, actual_cmd) }, \
         { "seq_identifier", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_zmotion_command_t, seq_identifier) }, \
         { "custom_param1", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_zmotion_command_t, custom_param1) }, \
         { "custom_param2", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_zmotion_command_t, custom_param2) }, \
         { "custom_param3", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_zmotion_command_t, custom_param3) }, \
         { "custom_param4", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_zmotion_command_t, custom_param4) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ZMOTION_COMMAND { \
    "ZMOTION_COMMAND", \
    7, \
    {  { "zmotion_cmd_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_zmotion_command_t, zmotion_cmd_type) }, \
         { "actual_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_zmotion_command_t, actual_cmd) }, \
         { "seq_identifier", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_zmotion_command_t, seq_identifier) }, \
         { "custom_param1", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_zmotion_command_t, custom_param1) }, \
         { "custom_param2", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_zmotion_command_t, custom_param2) }, \
         { "custom_param3", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_zmotion_command_t, custom_param3) }, \
         { "custom_param4", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_zmotion_command_t, custom_param4) }, \
         } \
}
#endif

/**
 * @brief Pack a zmotion_command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param zmotion_cmd_type  ZMotion Command Type
 * @param actual_cmd  Actual Command
 * @param seq_identifier  A unique identifier to associate the command with the response.
 * @param custom_param1  Custom Param 1 for the command.
 * @param custom_param2  Custom Param 2 for the command.
 * @param custom_param3  Custom Param 3 for the command.
 * @param custom_param4  Custom Param 4 for the command.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_zmotion_command_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t zmotion_cmd_type, uint8_t actual_cmd, uint16_t seq_identifier, uint32_t custom_param1, uint32_t custom_param2, uint32_t custom_param3, uint32_t custom_param4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZMOTION_COMMAND_LEN];
    _mav_put_uint32_t(buf, 0, custom_param1);
    _mav_put_uint32_t(buf, 4, custom_param2);
    _mav_put_uint32_t(buf, 8, custom_param3);
    _mav_put_uint32_t(buf, 12, custom_param4);
    _mav_put_uint16_t(buf, 16, seq_identifier);
    _mav_put_uint8_t(buf, 18, zmotion_cmd_type);
    _mav_put_uint8_t(buf, 19, actual_cmd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ZMOTION_COMMAND_LEN);
#else
    mavlink_zmotion_command_t packet;
    packet.custom_param1 = custom_param1;
    packet.custom_param2 = custom_param2;
    packet.custom_param3 = custom_param3;
    packet.custom_param4 = custom_param4;
    packet.seq_identifier = seq_identifier;
    packet.zmotion_cmd_type = zmotion_cmd_type;
    packet.actual_cmd = actual_cmd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ZMOTION_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ZMOTION_COMMAND;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ZMOTION_COMMAND_MIN_LEN, MAVLINK_MSG_ID_ZMOTION_COMMAND_LEN, MAVLINK_MSG_ID_ZMOTION_COMMAND_CRC);
}

/**
 * @brief Pack a zmotion_command message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param zmotion_cmd_type  ZMotion Command Type
 * @param actual_cmd  Actual Command
 * @param seq_identifier  A unique identifier to associate the command with the response.
 * @param custom_param1  Custom Param 1 for the command.
 * @param custom_param2  Custom Param 2 for the command.
 * @param custom_param3  Custom Param 3 for the command.
 * @param custom_param4  Custom Param 4 for the command.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_zmotion_command_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t zmotion_cmd_type,uint8_t actual_cmd,uint16_t seq_identifier,uint32_t custom_param1,uint32_t custom_param2,uint32_t custom_param3,uint32_t custom_param4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZMOTION_COMMAND_LEN];
    _mav_put_uint32_t(buf, 0, custom_param1);
    _mav_put_uint32_t(buf, 4, custom_param2);
    _mav_put_uint32_t(buf, 8, custom_param3);
    _mav_put_uint32_t(buf, 12, custom_param4);
    _mav_put_uint16_t(buf, 16, seq_identifier);
    _mav_put_uint8_t(buf, 18, zmotion_cmd_type);
    _mav_put_uint8_t(buf, 19, actual_cmd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ZMOTION_COMMAND_LEN);
#else
    mavlink_zmotion_command_t packet;
    packet.custom_param1 = custom_param1;
    packet.custom_param2 = custom_param2;
    packet.custom_param3 = custom_param3;
    packet.custom_param4 = custom_param4;
    packet.seq_identifier = seq_identifier;
    packet.zmotion_cmd_type = zmotion_cmd_type;
    packet.actual_cmd = actual_cmd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ZMOTION_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ZMOTION_COMMAND;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ZMOTION_COMMAND_MIN_LEN, MAVLINK_MSG_ID_ZMOTION_COMMAND_LEN, MAVLINK_MSG_ID_ZMOTION_COMMAND_CRC);
}

/**
 * @brief Encode a zmotion_command struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param zmotion_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_zmotion_command_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_zmotion_command_t* zmotion_command)
{
    return mavlink_msg_zmotion_command_pack(system_id, component_id, msg, zmotion_command->zmotion_cmd_type, zmotion_command->actual_cmd, zmotion_command->seq_identifier, zmotion_command->custom_param1, zmotion_command->custom_param2, zmotion_command->custom_param3, zmotion_command->custom_param4);
}

/**
 * @brief Encode a zmotion_command struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param zmotion_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_zmotion_command_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_zmotion_command_t* zmotion_command)
{
    return mavlink_msg_zmotion_command_pack_chan(system_id, component_id, chan, msg, zmotion_command->zmotion_cmd_type, zmotion_command->actual_cmd, zmotion_command->seq_identifier, zmotion_command->custom_param1, zmotion_command->custom_param2, zmotion_command->custom_param3, zmotion_command->custom_param4);
}

/**
 * @brief Send a zmotion_command message
 * @param chan MAVLink channel to send the message
 *
 * @param zmotion_cmd_type  ZMotion Command Type
 * @param actual_cmd  Actual Command
 * @param seq_identifier  A unique identifier to associate the command with the response.
 * @param custom_param1  Custom Param 1 for the command.
 * @param custom_param2  Custom Param 2 for the command.
 * @param custom_param3  Custom Param 3 for the command.
 * @param custom_param4  Custom Param 4 for the command.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_zmotion_command_send(mavlink_channel_t chan, uint8_t zmotion_cmd_type, uint8_t actual_cmd, uint16_t seq_identifier, uint32_t custom_param1, uint32_t custom_param2, uint32_t custom_param3, uint32_t custom_param4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZMOTION_COMMAND_LEN];
    _mav_put_uint32_t(buf, 0, custom_param1);
    _mav_put_uint32_t(buf, 4, custom_param2);
    _mav_put_uint32_t(buf, 8, custom_param3);
    _mav_put_uint32_t(buf, 12, custom_param4);
    _mav_put_uint16_t(buf, 16, seq_identifier);
    _mav_put_uint8_t(buf, 18, zmotion_cmd_type);
    _mav_put_uint8_t(buf, 19, actual_cmd);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZMOTION_COMMAND, buf, MAVLINK_MSG_ID_ZMOTION_COMMAND_MIN_LEN, MAVLINK_MSG_ID_ZMOTION_COMMAND_LEN, MAVLINK_MSG_ID_ZMOTION_COMMAND_CRC);
#else
    mavlink_zmotion_command_t packet;
    packet.custom_param1 = custom_param1;
    packet.custom_param2 = custom_param2;
    packet.custom_param3 = custom_param3;
    packet.custom_param4 = custom_param4;
    packet.seq_identifier = seq_identifier;
    packet.zmotion_cmd_type = zmotion_cmd_type;
    packet.actual_cmd = actual_cmd;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZMOTION_COMMAND, (const char *)&packet, MAVLINK_MSG_ID_ZMOTION_COMMAND_MIN_LEN, MAVLINK_MSG_ID_ZMOTION_COMMAND_LEN, MAVLINK_MSG_ID_ZMOTION_COMMAND_CRC);
#endif
}

/**
 * @brief Send a zmotion_command message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_zmotion_command_send_struct(mavlink_channel_t chan, const mavlink_zmotion_command_t* zmotion_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_zmotion_command_send(chan, zmotion_command->zmotion_cmd_type, zmotion_command->actual_cmd, zmotion_command->seq_identifier, zmotion_command->custom_param1, zmotion_command->custom_param2, zmotion_command->custom_param3, zmotion_command->custom_param4);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZMOTION_COMMAND, (const char *)zmotion_command, MAVLINK_MSG_ID_ZMOTION_COMMAND_MIN_LEN, MAVLINK_MSG_ID_ZMOTION_COMMAND_LEN, MAVLINK_MSG_ID_ZMOTION_COMMAND_CRC);
#endif
}

#if MAVLINK_MSG_ID_ZMOTION_COMMAND_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_zmotion_command_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t zmotion_cmd_type, uint8_t actual_cmd, uint16_t seq_identifier, uint32_t custom_param1, uint32_t custom_param2, uint32_t custom_param3, uint32_t custom_param4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, custom_param1);
    _mav_put_uint32_t(buf, 4, custom_param2);
    _mav_put_uint32_t(buf, 8, custom_param3);
    _mav_put_uint32_t(buf, 12, custom_param4);
    _mav_put_uint16_t(buf, 16, seq_identifier);
    _mav_put_uint8_t(buf, 18, zmotion_cmd_type);
    _mav_put_uint8_t(buf, 19, actual_cmd);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZMOTION_COMMAND, buf, MAVLINK_MSG_ID_ZMOTION_COMMAND_MIN_LEN, MAVLINK_MSG_ID_ZMOTION_COMMAND_LEN, MAVLINK_MSG_ID_ZMOTION_COMMAND_CRC);
#else
    mavlink_zmotion_command_t *packet = (mavlink_zmotion_command_t *)msgbuf;
    packet->custom_param1 = custom_param1;
    packet->custom_param2 = custom_param2;
    packet->custom_param3 = custom_param3;
    packet->custom_param4 = custom_param4;
    packet->seq_identifier = seq_identifier;
    packet->zmotion_cmd_type = zmotion_cmd_type;
    packet->actual_cmd = actual_cmd;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZMOTION_COMMAND, (const char *)packet, MAVLINK_MSG_ID_ZMOTION_COMMAND_MIN_LEN, MAVLINK_MSG_ID_ZMOTION_COMMAND_LEN, MAVLINK_MSG_ID_ZMOTION_COMMAND_CRC);
#endif
}
#endif

#endif

// MESSAGE ZMOTION_COMMAND UNPACKING


/**
 * @brief Get field zmotion_cmd_type from zmotion_command message
 *
 * @return  ZMotion Command Type
 */
static inline uint8_t mavlink_msg_zmotion_command_get_zmotion_cmd_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field actual_cmd from zmotion_command message
 *
 * @return  Actual Command
 */
static inline uint8_t mavlink_msg_zmotion_command_get_actual_cmd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  19);
}

/**
 * @brief Get field seq_identifier from zmotion_command message
 *
 * @return  A unique identifier to associate the command with the response.
 */
static inline uint16_t mavlink_msg_zmotion_command_get_seq_identifier(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field custom_param1 from zmotion_command message
 *
 * @return  Custom Param 1 for the command.
 */
static inline uint32_t mavlink_msg_zmotion_command_get_custom_param1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field custom_param2 from zmotion_command message
 *
 * @return  Custom Param 2 for the command.
 */
static inline uint32_t mavlink_msg_zmotion_command_get_custom_param2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field custom_param3 from zmotion_command message
 *
 * @return  Custom Param 3 for the command.
 */
static inline uint32_t mavlink_msg_zmotion_command_get_custom_param3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field custom_param4 from zmotion_command message
 *
 * @return  Custom Param 4 for the command.
 */
static inline uint32_t mavlink_msg_zmotion_command_get_custom_param4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Decode a zmotion_command message into a struct
 *
 * @param msg The message to decode
 * @param zmotion_command C-struct to decode the message contents into
 */
static inline void mavlink_msg_zmotion_command_decode(const mavlink_message_t* msg, mavlink_zmotion_command_t* zmotion_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    zmotion_command->custom_param1 = mavlink_msg_zmotion_command_get_custom_param1(msg);
    zmotion_command->custom_param2 = mavlink_msg_zmotion_command_get_custom_param2(msg);
    zmotion_command->custom_param3 = mavlink_msg_zmotion_command_get_custom_param3(msg);
    zmotion_command->custom_param4 = mavlink_msg_zmotion_command_get_custom_param4(msg);
    zmotion_command->seq_identifier = mavlink_msg_zmotion_command_get_seq_identifier(msg);
    zmotion_command->zmotion_cmd_type = mavlink_msg_zmotion_command_get_zmotion_cmd_type(msg);
    zmotion_command->actual_cmd = mavlink_msg_zmotion_command_get_actual_cmd(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ZMOTION_COMMAND_LEN? msg->len : MAVLINK_MSG_ID_ZMOTION_COMMAND_LEN;
        memset(zmotion_command, 0, MAVLINK_MSG_ID_ZMOTION_COMMAND_LEN);
    memcpy(zmotion_command, _MAV_PAYLOAD(msg), len);
#endif
}
