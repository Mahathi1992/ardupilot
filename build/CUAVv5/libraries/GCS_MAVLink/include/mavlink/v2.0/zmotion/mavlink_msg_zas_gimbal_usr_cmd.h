#pragma once
// MESSAGE ZAS_GIMBAL_USR_CMD PACKING

#define MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD 2372


typedef struct __mavlink_zas_gimbal_usr_cmd_t {
 uint8_t calib_stat; /*< [units]  calibration status for ZAS gimbal */
 uint8_t mode; /*<   Gimbal mode for ZAS servo gimbal */
} mavlink_zas_gimbal_usr_cmd_t;

#define MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_LEN 2
#define MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_MIN_LEN 2
#define MAVLINK_MSG_ID_2372_LEN 2
#define MAVLINK_MSG_ID_2372_MIN_LEN 2

#define MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_CRC 16
#define MAVLINK_MSG_ID_2372_CRC 16



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ZAS_GIMBAL_USR_CMD { \
    2372, \
    "ZAS_GIMBAL_USR_CMD", \
    2, \
    {  { "calib_stat", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_zas_gimbal_usr_cmd_t, calib_stat) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_zas_gimbal_usr_cmd_t, mode) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ZAS_GIMBAL_USR_CMD { \
    "ZAS_GIMBAL_USR_CMD", \
    2, \
    {  { "calib_stat", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_zas_gimbal_usr_cmd_t, calib_stat) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_zas_gimbal_usr_cmd_t, mode) }, \
         } \
}
#endif

/**
 * @brief Pack a zas_gimbal_usr_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param calib_stat [units]  calibration status for ZAS gimbal 
 * @param mode   Gimbal mode for ZAS servo gimbal 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_zas_gimbal_usr_cmd_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t calib_stat, uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_LEN];
    _mav_put_uint8_t(buf, 0, calib_stat);
    _mav_put_uint8_t(buf, 1, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_LEN);
#else
    mavlink_zas_gimbal_usr_cmd_t packet;
    packet.calib_stat = calib_stat;
    packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_CRC);
}

/**
 * @brief Pack a zas_gimbal_usr_cmd message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param calib_stat [units]  calibration status for ZAS gimbal 
 * @param mode   Gimbal mode for ZAS servo gimbal 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_zas_gimbal_usr_cmd_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t calib_stat,uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_LEN];
    _mav_put_uint8_t(buf, 0, calib_stat);
    _mav_put_uint8_t(buf, 1, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_LEN);
#else
    mavlink_zas_gimbal_usr_cmd_t packet;
    packet.calib_stat = calib_stat;
    packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_CRC);
}

/**
 * @brief Encode a zas_gimbal_usr_cmd struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param zas_gimbal_usr_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_zas_gimbal_usr_cmd_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_zas_gimbal_usr_cmd_t* zas_gimbal_usr_cmd)
{
    return mavlink_msg_zas_gimbal_usr_cmd_pack(system_id, component_id, msg, zas_gimbal_usr_cmd->calib_stat, zas_gimbal_usr_cmd->mode);
}

/**
 * @brief Encode a zas_gimbal_usr_cmd struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param zas_gimbal_usr_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_zas_gimbal_usr_cmd_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_zas_gimbal_usr_cmd_t* zas_gimbal_usr_cmd)
{
    return mavlink_msg_zas_gimbal_usr_cmd_pack_chan(system_id, component_id, chan, msg, zas_gimbal_usr_cmd->calib_stat, zas_gimbal_usr_cmd->mode);
}

/**
 * @brief Send a zas_gimbal_usr_cmd message
 * @param chan MAVLink channel to send the message
 *
 * @param calib_stat [units]  calibration status for ZAS gimbal 
 * @param mode   Gimbal mode for ZAS servo gimbal 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_zas_gimbal_usr_cmd_send(mavlink_channel_t chan, uint8_t calib_stat, uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_LEN];
    _mav_put_uint8_t(buf, 0, calib_stat);
    _mav_put_uint8_t(buf, 1, mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD, buf, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_CRC);
#else
    mavlink_zas_gimbal_usr_cmd_t packet;
    packet.calib_stat = calib_stat;
    packet.mode = mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD, (const char *)&packet, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_CRC);
#endif
}

/**
 * @brief Send a zas_gimbal_usr_cmd message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_zas_gimbal_usr_cmd_send_struct(mavlink_channel_t chan, const mavlink_zas_gimbal_usr_cmd_t* zas_gimbal_usr_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_zas_gimbal_usr_cmd_send(chan, zas_gimbal_usr_cmd->calib_stat, zas_gimbal_usr_cmd->mode);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD, (const char *)zas_gimbal_usr_cmd, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_CRC);
#endif
}

#if MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_zas_gimbal_usr_cmd_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t calib_stat, uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, calib_stat);
    _mav_put_uint8_t(buf, 1, mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD, buf, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_CRC);
#else
    mavlink_zas_gimbal_usr_cmd_t *packet = (mavlink_zas_gimbal_usr_cmd_t *)msgbuf;
    packet->calib_stat = calib_stat;
    packet->mode = mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD, (const char *)packet, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_CRC);
#endif
}
#endif

#endif

// MESSAGE ZAS_GIMBAL_USR_CMD UNPACKING


/**
 * @brief Get field calib_stat from zas_gimbal_usr_cmd message
 *
 * @return [units]  calibration status for ZAS gimbal 
 */
static inline uint8_t mavlink_msg_zas_gimbal_usr_cmd_get_calib_stat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field mode from zas_gimbal_usr_cmd message
 *
 * @return   Gimbal mode for ZAS servo gimbal 
 */
static inline uint8_t mavlink_msg_zas_gimbal_usr_cmd_get_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a zas_gimbal_usr_cmd message into a struct
 *
 * @param msg The message to decode
 * @param zas_gimbal_usr_cmd C-struct to decode the message contents into
 */
static inline void mavlink_msg_zas_gimbal_usr_cmd_decode(const mavlink_message_t* msg, mavlink_zas_gimbal_usr_cmd_t* zas_gimbal_usr_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    zas_gimbal_usr_cmd->calib_stat = mavlink_msg_zas_gimbal_usr_cmd_get_calib_stat(msg);
    zas_gimbal_usr_cmd->mode = mavlink_msg_zas_gimbal_usr_cmd_get_mode(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_LEN? msg->len : MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_LEN;
        memset(zas_gimbal_usr_cmd, 0, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_LEN);
    memcpy(zas_gimbal_usr_cmd, _MAV_PAYLOAD(msg), len);
#endif
}
