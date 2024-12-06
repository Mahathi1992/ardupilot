#pragma once
// MESSAGE ZAS_GIMBAL_USR_CMD_GEOPOINT PACKING

#define MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT 2374


typedef struct __mavlink_zas_gimbal_usr_cmd_geopoint_t {
 int32_t target_latitude; /*< [degE7]  Target Latitude for gimbal tracking */
 int32_t target_longitude; /*< [degE7]  Target Longitude for gimbal tracking */
 int32_t target_altitude; /*< [m]  Target Altitude AMSL cm for gimbal tracking */
 uint8_t calib_stat; /*< [units]  calibration status for ZAS gimbal */
 uint8_t mode; /*<   Gimbal mode for ZAS servo gimbal */
} mavlink_zas_gimbal_usr_cmd_geopoint_t;

#define MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_LEN 14
#define MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_MIN_LEN 14
#define MAVLINK_MSG_ID_2374_LEN 14
#define MAVLINK_MSG_ID_2374_MIN_LEN 14

#define MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_CRC 46
#define MAVLINK_MSG_ID_2374_CRC 46



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ZAS_GIMBAL_USR_CMD_GEOPOINT { \
    2374, \
    "ZAS_GIMBAL_USR_CMD_GEOPOINT", \
    5, \
    {  { "target_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_zas_gimbal_usr_cmd_geopoint_t, target_latitude) }, \
         { "target_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_zas_gimbal_usr_cmd_geopoint_t, target_longitude) }, \
         { "target_altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_zas_gimbal_usr_cmd_geopoint_t, target_altitude) }, \
         { "calib_stat", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_zas_gimbal_usr_cmd_geopoint_t, calib_stat) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_zas_gimbal_usr_cmd_geopoint_t, mode) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ZAS_GIMBAL_USR_CMD_GEOPOINT { \
    "ZAS_GIMBAL_USR_CMD_GEOPOINT", \
    5, \
    {  { "target_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_zas_gimbal_usr_cmd_geopoint_t, target_latitude) }, \
         { "target_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_zas_gimbal_usr_cmd_geopoint_t, target_longitude) }, \
         { "target_altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_zas_gimbal_usr_cmd_geopoint_t, target_altitude) }, \
         { "calib_stat", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_zas_gimbal_usr_cmd_geopoint_t, calib_stat) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_zas_gimbal_usr_cmd_geopoint_t, mode) }, \
         } \
}
#endif

/**
 * @brief Pack a zas_gimbal_usr_cmd_geopoint message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_latitude [degE7]  Target Latitude for gimbal tracking 
 * @param target_longitude [degE7]  Target Longitude for gimbal tracking 
 * @param target_altitude [m]  Target Altitude AMSL cm for gimbal tracking 
 * @param calib_stat [units]  calibration status for ZAS gimbal 
 * @param mode   Gimbal mode for ZAS servo gimbal 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_zas_gimbal_usr_cmd_geopoint_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t target_latitude, int32_t target_longitude, int32_t target_altitude, uint8_t calib_stat, uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_LEN];
    _mav_put_int32_t(buf, 0, target_latitude);
    _mav_put_int32_t(buf, 4, target_longitude);
    _mav_put_int32_t(buf, 8, target_altitude);
    _mav_put_uint8_t(buf, 12, calib_stat);
    _mav_put_uint8_t(buf, 13, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_LEN);
#else
    mavlink_zas_gimbal_usr_cmd_geopoint_t packet;
    packet.target_latitude = target_latitude;
    packet.target_longitude = target_longitude;
    packet.target_altitude = target_altitude;
    packet.calib_stat = calib_stat;
    packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_CRC);
}

/**
 * @brief Pack a zas_gimbal_usr_cmd_geopoint message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_latitude [degE7]  Target Latitude for gimbal tracking 
 * @param target_longitude [degE7]  Target Longitude for gimbal tracking 
 * @param target_altitude [m]  Target Altitude AMSL cm for gimbal tracking 
 * @param calib_stat [units]  calibration status for ZAS gimbal 
 * @param mode   Gimbal mode for ZAS servo gimbal 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_zas_gimbal_usr_cmd_geopoint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t target_latitude,int32_t target_longitude,int32_t target_altitude,uint8_t calib_stat,uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_LEN];
    _mav_put_int32_t(buf, 0, target_latitude);
    _mav_put_int32_t(buf, 4, target_longitude);
    _mav_put_int32_t(buf, 8, target_altitude);
    _mav_put_uint8_t(buf, 12, calib_stat);
    _mav_put_uint8_t(buf, 13, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_LEN);
#else
    mavlink_zas_gimbal_usr_cmd_geopoint_t packet;
    packet.target_latitude = target_latitude;
    packet.target_longitude = target_longitude;
    packet.target_altitude = target_altitude;
    packet.calib_stat = calib_stat;
    packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_CRC);
}

/**
 * @brief Encode a zas_gimbal_usr_cmd_geopoint struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param zas_gimbal_usr_cmd_geopoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_zas_gimbal_usr_cmd_geopoint_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_zas_gimbal_usr_cmd_geopoint_t* zas_gimbal_usr_cmd_geopoint)
{
    return mavlink_msg_zas_gimbal_usr_cmd_geopoint_pack(system_id, component_id, msg, zas_gimbal_usr_cmd_geopoint->target_latitude, zas_gimbal_usr_cmd_geopoint->target_longitude, zas_gimbal_usr_cmd_geopoint->target_altitude, zas_gimbal_usr_cmd_geopoint->calib_stat, zas_gimbal_usr_cmd_geopoint->mode);
}

/**
 * @brief Encode a zas_gimbal_usr_cmd_geopoint struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param zas_gimbal_usr_cmd_geopoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_zas_gimbal_usr_cmd_geopoint_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_zas_gimbal_usr_cmd_geopoint_t* zas_gimbal_usr_cmd_geopoint)
{
    return mavlink_msg_zas_gimbal_usr_cmd_geopoint_pack_chan(system_id, component_id, chan, msg, zas_gimbal_usr_cmd_geopoint->target_latitude, zas_gimbal_usr_cmd_geopoint->target_longitude, zas_gimbal_usr_cmd_geopoint->target_altitude, zas_gimbal_usr_cmd_geopoint->calib_stat, zas_gimbal_usr_cmd_geopoint->mode);
}

/**
 * @brief Send a zas_gimbal_usr_cmd_geopoint message
 * @param chan MAVLink channel to send the message
 *
 * @param target_latitude [degE7]  Target Latitude for gimbal tracking 
 * @param target_longitude [degE7]  Target Longitude for gimbal tracking 
 * @param target_altitude [m]  Target Altitude AMSL cm for gimbal tracking 
 * @param calib_stat [units]  calibration status for ZAS gimbal 
 * @param mode   Gimbal mode for ZAS servo gimbal 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_zas_gimbal_usr_cmd_geopoint_send(mavlink_channel_t chan, int32_t target_latitude, int32_t target_longitude, int32_t target_altitude, uint8_t calib_stat, uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_LEN];
    _mav_put_int32_t(buf, 0, target_latitude);
    _mav_put_int32_t(buf, 4, target_longitude);
    _mav_put_int32_t(buf, 8, target_altitude);
    _mav_put_uint8_t(buf, 12, calib_stat);
    _mav_put_uint8_t(buf, 13, mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT, buf, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_CRC);
#else
    mavlink_zas_gimbal_usr_cmd_geopoint_t packet;
    packet.target_latitude = target_latitude;
    packet.target_longitude = target_longitude;
    packet.target_altitude = target_altitude;
    packet.calib_stat = calib_stat;
    packet.mode = mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT, (const char *)&packet, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_CRC);
#endif
}

/**
 * @brief Send a zas_gimbal_usr_cmd_geopoint message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_zas_gimbal_usr_cmd_geopoint_send_struct(mavlink_channel_t chan, const mavlink_zas_gimbal_usr_cmd_geopoint_t* zas_gimbal_usr_cmd_geopoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_zas_gimbal_usr_cmd_geopoint_send(chan, zas_gimbal_usr_cmd_geopoint->target_latitude, zas_gimbal_usr_cmd_geopoint->target_longitude, zas_gimbal_usr_cmd_geopoint->target_altitude, zas_gimbal_usr_cmd_geopoint->calib_stat, zas_gimbal_usr_cmd_geopoint->mode);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT, (const char *)zas_gimbal_usr_cmd_geopoint, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_CRC);
#endif
}

#if MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_zas_gimbal_usr_cmd_geopoint_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t target_latitude, int32_t target_longitude, int32_t target_altitude, uint8_t calib_stat, uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, target_latitude);
    _mav_put_int32_t(buf, 4, target_longitude);
    _mav_put_int32_t(buf, 8, target_altitude);
    _mav_put_uint8_t(buf, 12, calib_stat);
    _mav_put_uint8_t(buf, 13, mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT, buf, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_CRC);
#else
    mavlink_zas_gimbal_usr_cmd_geopoint_t *packet = (mavlink_zas_gimbal_usr_cmd_geopoint_t *)msgbuf;
    packet->target_latitude = target_latitude;
    packet->target_longitude = target_longitude;
    packet->target_altitude = target_altitude;
    packet->calib_stat = calib_stat;
    packet->mode = mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT, (const char *)packet, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_CRC);
#endif
}
#endif

#endif

// MESSAGE ZAS_GIMBAL_USR_CMD_GEOPOINT UNPACKING


/**
 * @brief Get field target_latitude from zas_gimbal_usr_cmd_geopoint message
 *
 * @return [degE7]  Target Latitude for gimbal tracking 
 */
static inline int32_t mavlink_msg_zas_gimbal_usr_cmd_geopoint_get_target_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field target_longitude from zas_gimbal_usr_cmd_geopoint message
 *
 * @return [degE7]  Target Longitude for gimbal tracking 
 */
static inline int32_t mavlink_msg_zas_gimbal_usr_cmd_geopoint_get_target_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field target_altitude from zas_gimbal_usr_cmd_geopoint message
 *
 * @return [m]  Target Altitude AMSL cm for gimbal tracking 
 */
static inline int32_t mavlink_msg_zas_gimbal_usr_cmd_geopoint_get_target_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field calib_stat from zas_gimbal_usr_cmd_geopoint message
 *
 * @return [units]  calibration status for ZAS gimbal 
 */
static inline uint8_t mavlink_msg_zas_gimbal_usr_cmd_geopoint_get_calib_stat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field mode from zas_gimbal_usr_cmd_geopoint message
 *
 * @return   Gimbal mode for ZAS servo gimbal 
 */
static inline uint8_t mavlink_msg_zas_gimbal_usr_cmd_geopoint_get_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Decode a zas_gimbal_usr_cmd_geopoint message into a struct
 *
 * @param msg The message to decode
 * @param zas_gimbal_usr_cmd_geopoint C-struct to decode the message contents into
 */
static inline void mavlink_msg_zas_gimbal_usr_cmd_geopoint_decode(const mavlink_message_t* msg, mavlink_zas_gimbal_usr_cmd_geopoint_t* zas_gimbal_usr_cmd_geopoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    zas_gimbal_usr_cmd_geopoint->target_latitude = mavlink_msg_zas_gimbal_usr_cmd_geopoint_get_target_latitude(msg);
    zas_gimbal_usr_cmd_geopoint->target_longitude = mavlink_msg_zas_gimbal_usr_cmd_geopoint_get_target_longitude(msg);
    zas_gimbal_usr_cmd_geopoint->target_altitude = mavlink_msg_zas_gimbal_usr_cmd_geopoint_get_target_altitude(msg);
    zas_gimbal_usr_cmd_geopoint->calib_stat = mavlink_msg_zas_gimbal_usr_cmd_geopoint_get_calib_stat(msg);
    zas_gimbal_usr_cmd_geopoint->mode = mavlink_msg_zas_gimbal_usr_cmd_geopoint_get_mode(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_LEN? msg->len : MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_LEN;
        memset(zas_gimbal_usr_cmd_geopoint, 0, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT_LEN);
    memcpy(zas_gimbal_usr_cmd_geopoint, _MAV_PAYLOAD(msg), len);
#endif
}
