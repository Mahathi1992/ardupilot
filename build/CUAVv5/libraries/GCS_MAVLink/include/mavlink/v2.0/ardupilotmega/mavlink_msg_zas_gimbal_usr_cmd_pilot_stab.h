#pragma once
// MESSAGE ZAS_GIMBAL_USR_CMD_PILOT_STAB PACKING

#define MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB 2373


typedef struct __mavlink_zas_gimbal_usr_cmd_pilot_stab_t {
 float yawrate; /*< [deg/s]  Gimbal yawrate cmd from GCS in deg/s */
 float pitchrate; /*< [deg/s]  Gimbal pitchrate cmd from GCS in deg/s */
 float rollrate; /*< [deg/s]  Gimbal rollrate cmd from GCSin deg/s */
 float pan; /*< [deg]  Pan demanded from user via GCS cmd in deg */
 float tilt; /*< [deg]  Tilt demanded from user via GCS cmd in deg */
 uint8_t calib_stat; /*< [units]  calibration status for ZAS gimbal */
 uint8_t submode; /*<   Gimbal sub mode for ZAS servo gimbal in pilot or stabilize mode */
 uint8_t mode; /*<   Gimbal mode for ZAS servo gimbal */
} mavlink_zas_gimbal_usr_cmd_pilot_stab_t;

#define MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_LEN 23
#define MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_MIN_LEN 23
#define MAVLINK_MSG_ID_2373_LEN 23
#define MAVLINK_MSG_ID_2373_MIN_LEN 23

#define MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_CRC 171
#define MAVLINK_MSG_ID_2373_CRC 171



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ZAS_GIMBAL_USR_CMD_PILOT_STAB { \
    2373, \
    "ZAS_GIMBAL_USR_CMD_PILOT_STAB", \
    8, \
    {  { "yawrate", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_zas_gimbal_usr_cmd_pilot_stab_t, yawrate) }, \
         { "pitchrate", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_zas_gimbal_usr_cmd_pilot_stab_t, pitchrate) }, \
         { "rollrate", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_zas_gimbal_usr_cmd_pilot_stab_t, rollrate) }, \
         { "pan", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_zas_gimbal_usr_cmd_pilot_stab_t, pan) }, \
         { "tilt", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_zas_gimbal_usr_cmd_pilot_stab_t, tilt) }, \
         { "calib_stat", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_zas_gimbal_usr_cmd_pilot_stab_t, calib_stat) }, \
         { "submode", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_zas_gimbal_usr_cmd_pilot_stab_t, submode) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_zas_gimbal_usr_cmd_pilot_stab_t, mode) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ZAS_GIMBAL_USR_CMD_PILOT_STAB { \
    "ZAS_GIMBAL_USR_CMD_PILOT_STAB", \
    8, \
    {  { "yawrate", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_zas_gimbal_usr_cmd_pilot_stab_t, yawrate) }, \
         { "pitchrate", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_zas_gimbal_usr_cmd_pilot_stab_t, pitchrate) }, \
         { "rollrate", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_zas_gimbal_usr_cmd_pilot_stab_t, rollrate) }, \
         { "pan", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_zas_gimbal_usr_cmd_pilot_stab_t, pan) }, \
         { "tilt", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_zas_gimbal_usr_cmd_pilot_stab_t, tilt) }, \
         { "calib_stat", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_zas_gimbal_usr_cmd_pilot_stab_t, calib_stat) }, \
         { "submode", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_zas_gimbal_usr_cmd_pilot_stab_t, submode) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_zas_gimbal_usr_cmd_pilot_stab_t, mode) }, \
         } \
}
#endif

/**
 * @brief Pack a zas_gimbal_usr_cmd_pilot_stab message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param yawrate [deg/s]  Gimbal yawrate cmd from GCS in deg/s 
 * @param pitchrate [deg/s]  Gimbal pitchrate cmd from GCS in deg/s 
 * @param rollrate [deg/s]  Gimbal rollrate cmd from GCSin deg/s 
 * @param pan [deg]  Pan demanded from user via GCS cmd in deg 
 * @param tilt [deg]  Tilt demanded from user via GCS cmd in deg 
 * @param calib_stat [units]  calibration status for ZAS gimbal 
 * @param submode   Gimbal sub mode for ZAS servo gimbal in pilot or stabilize mode 
 * @param mode   Gimbal mode for ZAS servo gimbal 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float yawrate, float pitchrate, float rollrate, float pan, float tilt, uint8_t calib_stat, uint8_t submode, uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_LEN];
    _mav_put_float(buf, 0, yawrate);
    _mav_put_float(buf, 4, pitchrate);
    _mav_put_float(buf, 8, rollrate);
    _mav_put_float(buf, 12, pan);
    _mav_put_float(buf, 16, tilt);
    _mav_put_uint8_t(buf, 20, calib_stat);
    _mav_put_uint8_t(buf, 21, submode);
    _mav_put_uint8_t(buf, 22, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_LEN);
#else
    mavlink_zas_gimbal_usr_cmd_pilot_stab_t packet;
    packet.yawrate = yawrate;
    packet.pitchrate = pitchrate;
    packet.rollrate = rollrate;
    packet.pan = pan;
    packet.tilt = tilt;
    packet.calib_stat = calib_stat;
    packet.submode = submode;
    packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_CRC);
}

/**
 * @brief Pack a zas_gimbal_usr_cmd_pilot_stab message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param yawrate [deg/s]  Gimbal yawrate cmd from GCS in deg/s 
 * @param pitchrate [deg/s]  Gimbal pitchrate cmd from GCS in deg/s 
 * @param rollrate [deg/s]  Gimbal rollrate cmd from GCSin deg/s 
 * @param pan [deg]  Pan demanded from user via GCS cmd in deg 
 * @param tilt [deg]  Tilt demanded from user via GCS cmd in deg 
 * @param calib_stat [units]  calibration status for ZAS gimbal 
 * @param submode   Gimbal sub mode for ZAS servo gimbal in pilot or stabilize mode 
 * @param mode   Gimbal mode for ZAS servo gimbal 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float yawrate,float pitchrate,float rollrate,float pan,float tilt,uint8_t calib_stat,uint8_t submode,uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_LEN];
    _mav_put_float(buf, 0, yawrate);
    _mav_put_float(buf, 4, pitchrate);
    _mav_put_float(buf, 8, rollrate);
    _mav_put_float(buf, 12, pan);
    _mav_put_float(buf, 16, tilt);
    _mav_put_uint8_t(buf, 20, calib_stat);
    _mav_put_uint8_t(buf, 21, submode);
    _mav_put_uint8_t(buf, 22, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_LEN);
#else
    mavlink_zas_gimbal_usr_cmd_pilot_stab_t packet;
    packet.yawrate = yawrate;
    packet.pitchrate = pitchrate;
    packet.rollrate = rollrate;
    packet.pan = pan;
    packet.tilt = tilt;
    packet.calib_stat = calib_stat;
    packet.submode = submode;
    packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_CRC);
}

/**
 * @brief Encode a zas_gimbal_usr_cmd_pilot_stab struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param zas_gimbal_usr_cmd_pilot_stab C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_zas_gimbal_usr_cmd_pilot_stab_t* zas_gimbal_usr_cmd_pilot_stab)
{
    return mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_pack(system_id, component_id, msg, zas_gimbal_usr_cmd_pilot_stab->yawrate, zas_gimbal_usr_cmd_pilot_stab->pitchrate, zas_gimbal_usr_cmd_pilot_stab->rollrate, zas_gimbal_usr_cmd_pilot_stab->pan, zas_gimbal_usr_cmd_pilot_stab->tilt, zas_gimbal_usr_cmd_pilot_stab->calib_stat, zas_gimbal_usr_cmd_pilot_stab->submode, zas_gimbal_usr_cmd_pilot_stab->mode);
}

/**
 * @brief Encode a zas_gimbal_usr_cmd_pilot_stab struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param zas_gimbal_usr_cmd_pilot_stab C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_zas_gimbal_usr_cmd_pilot_stab_t* zas_gimbal_usr_cmd_pilot_stab)
{
    return mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_pack_chan(system_id, component_id, chan, msg, zas_gimbal_usr_cmd_pilot_stab->yawrate, zas_gimbal_usr_cmd_pilot_stab->pitchrate, zas_gimbal_usr_cmd_pilot_stab->rollrate, zas_gimbal_usr_cmd_pilot_stab->pan, zas_gimbal_usr_cmd_pilot_stab->tilt, zas_gimbal_usr_cmd_pilot_stab->calib_stat, zas_gimbal_usr_cmd_pilot_stab->submode, zas_gimbal_usr_cmd_pilot_stab->mode);
}

/**
 * @brief Send a zas_gimbal_usr_cmd_pilot_stab message
 * @param chan MAVLink channel to send the message
 *
 * @param yawrate [deg/s]  Gimbal yawrate cmd from GCS in deg/s 
 * @param pitchrate [deg/s]  Gimbal pitchrate cmd from GCS in deg/s 
 * @param rollrate [deg/s]  Gimbal rollrate cmd from GCSin deg/s 
 * @param pan [deg]  Pan demanded from user via GCS cmd in deg 
 * @param tilt [deg]  Tilt demanded from user via GCS cmd in deg 
 * @param calib_stat [units]  calibration status for ZAS gimbal 
 * @param submode   Gimbal sub mode for ZAS servo gimbal in pilot or stabilize mode 
 * @param mode   Gimbal mode for ZAS servo gimbal 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_send(mavlink_channel_t chan, float yawrate, float pitchrate, float rollrate, float pan, float tilt, uint8_t calib_stat, uint8_t submode, uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_LEN];
    _mav_put_float(buf, 0, yawrate);
    _mav_put_float(buf, 4, pitchrate);
    _mav_put_float(buf, 8, rollrate);
    _mav_put_float(buf, 12, pan);
    _mav_put_float(buf, 16, tilt);
    _mav_put_uint8_t(buf, 20, calib_stat);
    _mav_put_uint8_t(buf, 21, submode);
    _mav_put_uint8_t(buf, 22, mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB, buf, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_CRC);
#else
    mavlink_zas_gimbal_usr_cmd_pilot_stab_t packet;
    packet.yawrate = yawrate;
    packet.pitchrate = pitchrate;
    packet.rollrate = rollrate;
    packet.pan = pan;
    packet.tilt = tilt;
    packet.calib_stat = calib_stat;
    packet.submode = submode;
    packet.mode = mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB, (const char *)&packet, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_CRC);
#endif
}

/**
 * @brief Send a zas_gimbal_usr_cmd_pilot_stab message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_send_struct(mavlink_channel_t chan, const mavlink_zas_gimbal_usr_cmd_pilot_stab_t* zas_gimbal_usr_cmd_pilot_stab)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_send(chan, zas_gimbal_usr_cmd_pilot_stab->yawrate, zas_gimbal_usr_cmd_pilot_stab->pitchrate, zas_gimbal_usr_cmd_pilot_stab->rollrate, zas_gimbal_usr_cmd_pilot_stab->pan, zas_gimbal_usr_cmd_pilot_stab->tilt, zas_gimbal_usr_cmd_pilot_stab->calib_stat, zas_gimbal_usr_cmd_pilot_stab->submode, zas_gimbal_usr_cmd_pilot_stab->mode);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB, (const char *)zas_gimbal_usr_cmd_pilot_stab, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_CRC);
#endif
}

#if MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float yawrate, float pitchrate, float rollrate, float pan, float tilt, uint8_t calib_stat, uint8_t submode, uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, yawrate);
    _mav_put_float(buf, 4, pitchrate);
    _mav_put_float(buf, 8, rollrate);
    _mav_put_float(buf, 12, pan);
    _mav_put_float(buf, 16, tilt);
    _mav_put_uint8_t(buf, 20, calib_stat);
    _mav_put_uint8_t(buf, 21, submode);
    _mav_put_uint8_t(buf, 22, mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB, buf, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_CRC);
#else
    mavlink_zas_gimbal_usr_cmd_pilot_stab_t *packet = (mavlink_zas_gimbal_usr_cmd_pilot_stab_t *)msgbuf;
    packet->yawrate = yawrate;
    packet->pitchrate = pitchrate;
    packet->rollrate = rollrate;
    packet->pan = pan;
    packet->tilt = tilt;
    packet->calib_stat = calib_stat;
    packet->submode = submode;
    packet->mode = mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB, (const char *)packet, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_CRC);
#endif
}
#endif

#endif

// MESSAGE ZAS_GIMBAL_USR_CMD_PILOT_STAB UNPACKING


/**
 * @brief Get field yawrate from zas_gimbal_usr_cmd_pilot_stab message
 *
 * @return [deg/s]  Gimbal yawrate cmd from GCS in deg/s 
 */
static inline float mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_get_yawrate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pitchrate from zas_gimbal_usr_cmd_pilot_stab message
 *
 * @return [deg/s]  Gimbal pitchrate cmd from GCS in deg/s 
 */
static inline float mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_get_pitchrate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field rollrate from zas_gimbal_usr_cmd_pilot_stab message
 *
 * @return [deg/s]  Gimbal rollrate cmd from GCSin deg/s 
 */
static inline float mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_get_rollrate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pan from zas_gimbal_usr_cmd_pilot_stab message
 *
 * @return [deg]  Pan demanded from user via GCS cmd in deg 
 */
static inline float mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_get_pan(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field tilt from zas_gimbal_usr_cmd_pilot_stab message
 *
 * @return [deg]  Tilt demanded from user via GCS cmd in deg 
 */
static inline float mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_get_tilt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field calib_stat from zas_gimbal_usr_cmd_pilot_stab message
 *
 * @return [units]  calibration status for ZAS gimbal 
 */
static inline uint8_t mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_get_calib_stat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field submode from zas_gimbal_usr_cmd_pilot_stab message
 *
 * @return   Gimbal sub mode for ZAS servo gimbal in pilot or stabilize mode 
 */
static inline uint8_t mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_get_submode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field mode from zas_gimbal_usr_cmd_pilot_stab message
 *
 * @return   Gimbal mode for ZAS servo gimbal 
 */
static inline uint8_t mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_get_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Decode a zas_gimbal_usr_cmd_pilot_stab message into a struct
 *
 * @param msg The message to decode
 * @param zas_gimbal_usr_cmd_pilot_stab C-struct to decode the message contents into
 */
static inline void mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_decode(const mavlink_message_t* msg, mavlink_zas_gimbal_usr_cmd_pilot_stab_t* zas_gimbal_usr_cmd_pilot_stab)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    zas_gimbal_usr_cmd_pilot_stab->yawrate = mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_get_yawrate(msg);
    zas_gimbal_usr_cmd_pilot_stab->pitchrate = mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_get_pitchrate(msg);
    zas_gimbal_usr_cmd_pilot_stab->rollrate = mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_get_rollrate(msg);
    zas_gimbal_usr_cmd_pilot_stab->pan = mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_get_pan(msg);
    zas_gimbal_usr_cmd_pilot_stab->tilt = mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_get_tilt(msg);
    zas_gimbal_usr_cmd_pilot_stab->calib_stat = mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_get_calib_stat(msg);
    zas_gimbal_usr_cmd_pilot_stab->submode = mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_get_submode(msg);
    zas_gimbal_usr_cmd_pilot_stab->mode = mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_get_mode(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_LEN? msg->len : MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_LEN;
        memset(zas_gimbal_usr_cmd_pilot_stab, 0, MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB_LEN);
    memcpy(zas_gimbal_usr_cmd_pilot_stab, _MAV_PAYLOAD(msg), len);
#endif
}
