#pragma once
// MESSAGE ZMOTION_PAYLOAD2_RESPONSE PACKING

#define MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE 58003


typedef struct __mavlink_zmotion_payload2_response_t {
 uint16_t seq_identifier; /*<  A unique identifier to associate the command with the response.*/
 uint8_t zmotion_cmd_type; /*<  ZMotion Command Type*/
 uint8_t actual_cmd; /*<  Actual Command*/
 uint8_t response1; /*<  Chassis ID.*/
 uint8_t response2; /*<  Chassis Health.*/
 uint8_t response3; /*<  Payload ID.*/
 uint8_t response4; /*<  Response.*/
} mavlink_zmotion_payload2_response_t;

#define MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_LEN 8
#define MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_MIN_LEN 8
#define MAVLINK_MSG_ID_58003_LEN 8
#define MAVLINK_MSG_ID_58003_MIN_LEN 8

#define MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_CRC 203
#define MAVLINK_MSG_ID_58003_CRC 203



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ZMOTION_PAYLOAD2_RESPONSE { \
    58003, \
    "ZMOTION_PAYLOAD2_RESPONSE", \
    7, \
    {  { "zmotion_cmd_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_zmotion_payload2_response_t, zmotion_cmd_type) }, \
         { "actual_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_zmotion_payload2_response_t, actual_cmd) }, \
         { "seq_identifier", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_zmotion_payload2_response_t, seq_identifier) }, \
         { "response1", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_zmotion_payload2_response_t, response1) }, \
         { "response2", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_zmotion_payload2_response_t, response2) }, \
         { "response3", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_zmotion_payload2_response_t, response3) }, \
         { "response4", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_zmotion_payload2_response_t, response4) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ZMOTION_PAYLOAD2_RESPONSE { \
    "ZMOTION_PAYLOAD2_RESPONSE", \
    7, \
    {  { "zmotion_cmd_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_zmotion_payload2_response_t, zmotion_cmd_type) }, \
         { "actual_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_zmotion_payload2_response_t, actual_cmd) }, \
         { "seq_identifier", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_zmotion_payload2_response_t, seq_identifier) }, \
         { "response1", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_zmotion_payload2_response_t, response1) }, \
         { "response2", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_zmotion_payload2_response_t, response2) }, \
         { "response3", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_zmotion_payload2_response_t, response3) }, \
         { "response4", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_zmotion_payload2_response_t, response4) }, \
         } \
}
#endif

/**
 * @brief Pack a zmotion_payload2_response message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param zmotion_cmd_type  ZMotion Command Type
 * @param actual_cmd  Actual Command
 * @param seq_identifier  A unique identifier to associate the command with the response.
 * @param response1  Chassis ID.
 * @param response2  Chassis Health.
 * @param response3  Payload ID.
 * @param response4  Response.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_zmotion_payload2_response_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t zmotion_cmd_type, uint8_t actual_cmd, uint16_t seq_identifier, uint8_t response1, uint8_t response2, uint8_t response3, uint8_t response4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_LEN];
    _mav_put_uint16_t(buf, 0, seq_identifier);
    _mav_put_uint8_t(buf, 2, zmotion_cmd_type);
    _mav_put_uint8_t(buf, 3, actual_cmd);
    _mav_put_uint8_t(buf, 4, response1);
    _mav_put_uint8_t(buf, 5, response2);
    _mav_put_uint8_t(buf, 6, response3);
    _mav_put_uint8_t(buf, 7, response4);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_LEN);
#else
    mavlink_zmotion_payload2_response_t packet;
    packet.seq_identifier = seq_identifier;
    packet.zmotion_cmd_type = zmotion_cmd_type;
    packet.actual_cmd = actual_cmd;
    packet.response1 = response1;
    packet.response2 = response2;
    packet.response3 = response3;
    packet.response4 = response4;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_LEN, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_CRC);
}

/**
 * @brief Pack a zmotion_payload2_response message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param zmotion_cmd_type  ZMotion Command Type
 * @param actual_cmd  Actual Command
 * @param seq_identifier  A unique identifier to associate the command with the response.
 * @param response1  Chassis ID.
 * @param response2  Chassis Health.
 * @param response3  Payload ID.
 * @param response4  Response.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_zmotion_payload2_response_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t zmotion_cmd_type,uint8_t actual_cmd,uint16_t seq_identifier,uint8_t response1,uint8_t response2,uint8_t response3,uint8_t response4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_LEN];
    _mav_put_uint16_t(buf, 0, seq_identifier);
    _mav_put_uint8_t(buf, 2, zmotion_cmd_type);
    _mav_put_uint8_t(buf, 3, actual_cmd);
    _mav_put_uint8_t(buf, 4, response1);
    _mav_put_uint8_t(buf, 5, response2);
    _mav_put_uint8_t(buf, 6, response3);
    _mav_put_uint8_t(buf, 7, response4);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_LEN);
#else
    mavlink_zmotion_payload2_response_t packet;
    packet.seq_identifier = seq_identifier;
    packet.zmotion_cmd_type = zmotion_cmd_type;
    packet.actual_cmd = actual_cmd;
    packet.response1 = response1;
    packet.response2 = response2;
    packet.response3 = response3;
    packet.response4 = response4;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_LEN, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_CRC);
}

/**
 * @brief Encode a zmotion_payload2_response struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param zmotion_payload2_response C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_zmotion_payload2_response_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_zmotion_payload2_response_t* zmotion_payload2_response)
{
    return mavlink_msg_zmotion_payload2_response_pack(system_id, component_id, msg, zmotion_payload2_response->zmotion_cmd_type, zmotion_payload2_response->actual_cmd, zmotion_payload2_response->seq_identifier, zmotion_payload2_response->response1, zmotion_payload2_response->response2, zmotion_payload2_response->response3, zmotion_payload2_response->response4);
}

/**
 * @brief Encode a zmotion_payload2_response struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param zmotion_payload2_response C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_zmotion_payload2_response_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_zmotion_payload2_response_t* zmotion_payload2_response)
{
    return mavlink_msg_zmotion_payload2_response_pack_chan(system_id, component_id, chan, msg, zmotion_payload2_response->zmotion_cmd_type, zmotion_payload2_response->actual_cmd, zmotion_payload2_response->seq_identifier, zmotion_payload2_response->response1, zmotion_payload2_response->response2, zmotion_payload2_response->response3, zmotion_payload2_response->response4);
}

/**
 * @brief Send a zmotion_payload2_response message
 * @param chan MAVLink channel to send the message
 *
 * @param zmotion_cmd_type  ZMotion Command Type
 * @param actual_cmd  Actual Command
 * @param seq_identifier  A unique identifier to associate the command with the response.
 * @param response1  Chassis ID.
 * @param response2  Chassis Health.
 * @param response3  Payload ID.
 * @param response4  Response.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_zmotion_payload2_response_send(mavlink_channel_t chan, uint8_t zmotion_cmd_type, uint8_t actual_cmd, uint16_t seq_identifier, uint8_t response1, uint8_t response2, uint8_t response3, uint8_t response4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_LEN];
    _mav_put_uint16_t(buf, 0, seq_identifier);
    _mav_put_uint8_t(buf, 2, zmotion_cmd_type);
    _mav_put_uint8_t(buf, 3, actual_cmd);
    _mav_put_uint8_t(buf, 4, response1);
    _mav_put_uint8_t(buf, 5, response2);
    _mav_put_uint8_t(buf, 6, response3);
    _mav_put_uint8_t(buf, 7, response4);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE, buf, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_LEN, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_CRC);
#else
    mavlink_zmotion_payload2_response_t packet;
    packet.seq_identifier = seq_identifier;
    packet.zmotion_cmd_type = zmotion_cmd_type;
    packet.actual_cmd = actual_cmd;
    packet.response1 = response1;
    packet.response2 = response2;
    packet.response3 = response3;
    packet.response4 = response4;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE, (const char *)&packet, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_LEN, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_CRC);
#endif
}

/**
 * @brief Send a zmotion_payload2_response message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_zmotion_payload2_response_send_struct(mavlink_channel_t chan, const mavlink_zmotion_payload2_response_t* zmotion_payload2_response)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_zmotion_payload2_response_send(chan, zmotion_payload2_response->zmotion_cmd_type, zmotion_payload2_response->actual_cmd, zmotion_payload2_response->seq_identifier, zmotion_payload2_response->response1, zmotion_payload2_response->response2, zmotion_payload2_response->response3, zmotion_payload2_response->response4);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE, (const char *)zmotion_payload2_response, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_LEN, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_CRC);
#endif
}

#if MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_zmotion_payload2_response_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t zmotion_cmd_type, uint8_t actual_cmd, uint16_t seq_identifier, uint8_t response1, uint8_t response2, uint8_t response3, uint8_t response4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, seq_identifier);
    _mav_put_uint8_t(buf, 2, zmotion_cmd_type);
    _mav_put_uint8_t(buf, 3, actual_cmd);
    _mav_put_uint8_t(buf, 4, response1);
    _mav_put_uint8_t(buf, 5, response2);
    _mav_put_uint8_t(buf, 6, response3);
    _mav_put_uint8_t(buf, 7, response4);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE, buf, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_LEN, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_CRC);
#else
    mavlink_zmotion_payload2_response_t *packet = (mavlink_zmotion_payload2_response_t *)msgbuf;
    packet->seq_identifier = seq_identifier;
    packet->zmotion_cmd_type = zmotion_cmd_type;
    packet->actual_cmd = actual_cmd;
    packet->response1 = response1;
    packet->response2 = response2;
    packet->response3 = response3;
    packet->response4 = response4;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE, (const char *)packet, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_LEN, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_CRC);
#endif
}
#endif

#endif

// MESSAGE ZMOTION_PAYLOAD2_RESPONSE UNPACKING


/**
 * @brief Get field zmotion_cmd_type from zmotion_payload2_response message
 *
 * @return  ZMotion Command Type
 */
static inline uint8_t mavlink_msg_zmotion_payload2_response_get_zmotion_cmd_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field actual_cmd from zmotion_payload2_response message
 *
 * @return  Actual Command
 */
static inline uint8_t mavlink_msg_zmotion_payload2_response_get_actual_cmd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field seq_identifier from zmotion_payload2_response message
 *
 * @return  A unique identifier to associate the command with the response.
 */
static inline uint16_t mavlink_msg_zmotion_payload2_response_get_seq_identifier(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field response1 from zmotion_payload2_response message
 *
 * @return  Chassis ID.
 */
static inline uint8_t mavlink_msg_zmotion_payload2_response_get_response1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field response2 from zmotion_payload2_response message
 *
 * @return  Chassis Health.
 */
static inline uint8_t mavlink_msg_zmotion_payload2_response_get_response2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field response3 from zmotion_payload2_response message
 *
 * @return  Payload ID.
 */
static inline uint8_t mavlink_msg_zmotion_payload2_response_get_response3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field response4 from zmotion_payload2_response message
 *
 * @return  Response.
 */
static inline uint8_t mavlink_msg_zmotion_payload2_response_get_response4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Decode a zmotion_payload2_response message into a struct
 *
 * @param msg The message to decode
 * @param zmotion_payload2_response C-struct to decode the message contents into
 */
static inline void mavlink_msg_zmotion_payload2_response_decode(const mavlink_message_t* msg, mavlink_zmotion_payload2_response_t* zmotion_payload2_response)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    zmotion_payload2_response->seq_identifier = mavlink_msg_zmotion_payload2_response_get_seq_identifier(msg);
    zmotion_payload2_response->zmotion_cmd_type = mavlink_msg_zmotion_payload2_response_get_zmotion_cmd_type(msg);
    zmotion_payload2_response->actual_cmd = mavlink_msg_zmotion_payload2_response_get_actual_cmd(msg);
    zmotion_payload2_response->response1 = mavlink_msg_zmotion_payload2_response_get_response1(msg);
    zmotion_payload2_response->response2 = mavlink_msg_zmotion_payload2_response_get_response2(msg);
    zmotion_payload2_response->response3 = mavlink_msg_zmotion_payload2_response_get_response3(msg);
    zmotion_payload2_response->response4 = mavlink_msg_zmotion_payload2_response_get_response4(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_LEN? msg->len : MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_LEN;
        memset(zmotion_payload2_response, 0, MAVLINK_MSG_ID_ZMOTION_PAYLOAD2_RESPONSE_LEN);
    memcpy(zmotion_payload2_response, _MAV_PAYLOAD(msg), len);
#endif
}
