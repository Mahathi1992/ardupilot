#pragma once
// MESSAGE ZAS_TRACK3D_PARAMETERS PACKING

#define MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS 54030


typedef struct __mavlink_zas_track3d_parameters_t {
 int32_t approach_latitude; /*< [degE7]  Approach Latitude for Track 3D mode*/
 int32_t approach_longitude; /*< [degE7]  Approach Longitude for Track 3D mode*/
 uint16_t abort_dive_altitude; /*< [m]  Altitude above which dive can be aborted if track is lost*/
 int16_t dive_pitch; /*< [cdegs]  Forced Pitch to maintain while diving*/
 int16_t dive_airspeed; /*< [cmsec]  Altitude above which dive can be aborted if track is lost*/
} mavlink_zas_track3d_parameters_t;

#define MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_LEN 14
#define MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_MIN_LEN 14
#define MAVLINK_MSG_ID_54030_LEN 14
#define MAVLINK_MSG_ID_54030_MIN_LEN 14

#define MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_CRC 57
#define MAVLINK_MSG_ID_54030_CRC 57



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ZAS_TRACK3D_PARAMETERS { \
    54030, \
    "ZAS_TRACK3D_PARAMETERS", \
    5, \
    {  { "abort_dive_altitude", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_zas_track3d_parameters_t, abort_dive_altitude) }, \
         { "approach_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_zas_track3d_parameters_t, approach_latitude) }, \
         { "approach_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_zas_track3d_parameters_t, approach_longitude) }, \
         { "dive_pitch", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_zas_track3d_parameters_t, dive_pitch) }, \
         { "dive_airspeed", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_zas_track3d_parameters_t, dive_airspeed) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ZAS_TRACK3D_PARAMETERS { \
    "ZAS_TRACK3D_PARAMETERS", \
    5, \
    {  { "abort_dive_altitude", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_zas_track3d_parameters_t, abort_dive_altitude) }, \
         { "approach_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_zas_track3d_parameters_t, approach_latitude) }, \
         { "approach_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_zas_track3d_parameters_t, approach_longitude) }, \
         { "dive_pitch", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_zas_track3d_parameters_t, dive_pitch) }, \
         { "dive_airspeed", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_zas_track3d_parameters_t, dive_airspeed) }, \
         } \
}
#endif

/**
 * @brief Pack a zas_track3d_parameters message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param abort_dive_altitude [m]  Altitude above which dive can be aborted if track is lost
 * @param approach_latitude [degE7]  Approach Latitude for Track 3D mode
 * @param approach_longitude [degE7]  Approach Longitude for Track 3D mode
 * @param dive_pitch [cdegs]  Forced Pitch to maintain while diving
 * @param dive_airspeed [cmsec]  Altitude above which dive can be aborted if track is lost
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_zas_track3d_parameters_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t abort_dive_altitude, int32_t approach_latitude, int32_t approach_longitude, int16_t dive_pitch, int16_t dive_airspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_LEN];
    _mav_put_int32_t(buf, 0, approach_latitude);
    _mav_put_int32_t(buf, 4, approach_longitude);
    _mav_put_uint16_t(buf, 8, abort_dive_altitude);
    _mav_put_int16_t(buf, 10, dive_pitch);
    _mav_put_int16_t(buf, 12, dive_airspeed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_LEN);
#else
    mavlink_zas_track3d_parameters_t packet;
    packet.approach_latitude = approach_latitude;
    packet.approach_longitude = approach_longitude;
    packet.abort_dive_altitude = abort_dive_altitude;
    packet.dive_pitch = dive_pitch;
    packet.dive_airspeed = dive_airspeed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_MIN_LEN, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_LEN, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_CRC);
}

/**
 * @brief Pack a zas_track3d_parameters message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param abort_dive_altitude [m]  Altitude above which dive can be aborted if track is lost
 * @param approach_latitude [degE7]  Approach Latitude for Track 3D mode
 * @param approach_longitude [degE7]  Approach Longitude for Track 3D mode
 * @param dive_pitch [cdegs]  Forced Pitch to maintain while diving
 * @param dive_airspeed [cmsec]  Altitude above which dive can be aborted if track is lost
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_zas_track3d_parameters_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t abort_dive_altitude,int32_t approach_latitude,int32_t approach_longitude,int16_t dive_pitch,int16_t dive_airspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_LEN];
    _mav_put_int32_t(buf, 0, approach_latitude);
    _mav_put_int32_t(buf, 4, approach_longitude);
    _mav_put_uint16_t(buf, 8, abort_dive_altitude);
    _mav_put_int16_t(buf, 10, dive_pitch);
    _mav_put_int16_t(buf, 12, dive_airspeed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_LEN);
#else
    mavlink_zas_track3d_parameters_t packet;
    packet.approach_latitude = approach_latitude;
    packet.approach_longitude = approach_longitude;
    packet.abort_dive_altitude = abort_dive_altitude;
    packet.dive_pitch = dive_pitch;
    packet.dive_airspeed = dive_airspeed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_MIN_LEN, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_LEN, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_CRC);
}

/**
 * @brief Encode a zas_track3d_parameters struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param zas_track3d_parameters C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_zas_track3d_parameters_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_zas_track3d_parameters_t* zas_track3d_parameters)
{
    return mavlink_msg_zas_track3d_parameters_pack(system_id, component_id, msg, zas_track3d_parameters->abort_dive_altitude, zas_track3d_parameters->approach_latitude, zas_track3d_parameters->approach_longitude, zas_track3d_parameters->dive_pitch, zas_track3d_parameters->dive_airspeed);
}

/**
 * @brief Encode a zas_track3d_parameters struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param zas_track3d_parameters C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_zas_track3d_parameters_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_zas_track3d_parameters_t* zas_track3d_parameters)
{
    return mavlink_msg_zas_track3d_parameters_pack_chan(system_id, component_id, chan, msg, zas_track3d_parameters->abort_dive_altitude, zas_track3d_parameters->approach_latitude, zas_track3d_parameters->approach_longitude, zas_track3d_parameters->dive_pitch, zas_track3d_parameters->dive_airspeed);
}

/**
 * @brief Send a zas_track3d_parameters message
 * @param chan MAVLink channel to send the message
 *
 * @param abort_dive_altitude [m]  Altitude above which dive can be aborted if track is lost
 * @param approach_latitude [degE7]  Approach Latitude for Track 3D mode
 * @param approach_longitude [degE7]  Approach Longitude for Track 3D mode
 * @param dive_pitch [cdegs]  Forced Pitch to maintain while diving
 * @param dive_airspeed [cmsec]  Altitude above which dive can be aborted if track is lost
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_zas_track3d_parameters_send(mavlink_channel_t chan, uint16_t abort_dive_altitude, int32_t approach_latitude, int32_t approach_longitude, int16_t dive_pitch, int16_t dive_airspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_LEN];
    _mav_put_int32_t(buf, 0, approach_latitude);
    _mav_put_int32_t(buf, 4, approach_longitude);
    _mav_put_uint16_t(buf, 8, abort_dive_altitude);
    _mav_put_int16_t(buf, 10, dive_pitch);
    _mav_put_int16_t(buf, 12, dive_airspeed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS, buf, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_MIN_LEN, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_LEN, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_CRC);
#else
    mavlink_zas_track3d_parameters_t packet;
    packet.approach_latitude = approach_latitude;
    packet.approach_longitude = approach_longitude;
    packet.abort_dive_altitude = abort_dive_altitude;
    packet.dive_pitch = dive_pitch;
    packet.dive_airspeed = dive_airspeed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS, (const char *)&packet, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_MIN_LEN, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_LEN, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_CRC);
#endif
}

/**
 * @brief Send a zas_track3d_parameters message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_zas_track3d_parameters_send_struct(mavlink_channel_t chan, const mavlink_zas_track3d_parameters_t* zas_track3d_parameters)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_zas_track3d_parameters_send(chan, zas_track3d_parameters->abort_dive_altitude, zas_track3d_parameters->approach_latitude, zas_track3d_parameters->approach_longitude, zas_track3d_parameters->dive_pitch, zas_track3d_parameters->dive_airspeed);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS, (const char *)zas_track3d_parameters, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_MIN_LEN, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_LEN, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_zas_track3d_parameters_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t abort_dive_altitude, int32_t approach_latitude, int32_t approach_longitude, int16_t dive_pitch, int16_t dive_airspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, approach_latitude);
    _mav_put_int32_t(buf, 4, approach_longitude);
    _mav_put_uint16_t(buf, 8, abort_dive_altitude);
    _mav_put_int16_t(buf, 10, dive_pitch);
    _mav_put_int16_t(buf, 12, dive_airspeed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS, buf, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_MIN_LEN, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_LEN, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_CRC);
#else
    mavlink_zas_track3d_parameters_t *packet = (mavlink_zas_track3d_parameters_t *)msgbuf;
    packet->approach_latitude = approach_latitude;
    packet->approach_longitude = approach_longitude;
    packet->abort_dive_altitude = abort_dive_altitude;
    packet->dive_pitch = dive_pitch;
    packet->dive_airspeed = dive_airspeed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS, (const char *)packet, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_MIN_LEN, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_LEN, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_CRC);
#endif
}
#endif

#endif

// MESSAGE ZAS_TRACK3D_PARAMETERS UNPACKING


/**
 * @brief Get field abort_dive_altitude from zas_track3d_parameters message
 *
 * @return [m]  Altitude above which dive can be aborted if track is lost
 */
static inline uint16_t mavlink_msg_zas_track3d_parameters_get_abort_dive_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field approach_latitude from zas_track3d_parameters message
 *
 * @return [degE7]  Approach Latitude for Track 3D mode
 */
static inline int32_t mavlink_msg_zas_track3d_parameters_get_approach_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field approach_longitude from zas_track3d_parameters message
 *
 * @return [degE7]  Approach Longitude for Track 3D mode
 */
static inline int32_t mavlink_msg_zas_track3d_parameters_get_approach_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field dive_pitch from zas_track3d_parameters message
 *
 * @return [cdegs]  Forced Pitch to maintain while diving
 */
static inline int16_t mavlink_msg_zas_track3d_parameters_get_dive_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field dive_airspeed from zas_track3d_parameters message
 *
 * @return [cmsec]  Altitude above which dive can be aborted if track is lost
 */
static inline int16_t mavlink_msg_zas_track3d_parameters_get_dive_airspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Decode a zas_track3d_parameters message into a struct
 *
 * @param msg The message to decode
 * @param zas_track3d_parameters C-struct to decode the message contents into
 */
static inline void mavlink_msg_zas_track3d_parameters_decode(const mavlink_message_t* msg, mavlink_zas_track3d_parameters_t* zas_track3d_parameters)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    zas_track3d_parameters->approach_latitude = mavlink_msg_zas_track3d_parameters_get_approach_latitude(msg);
    zas_track3d_parameters->approach_longitude = mavlink_msg_zas_track3d_parameters_get_approach_longitude(msg);
    zas_track3d_parameters->abort_dive_altitude = mavlink_msg_zas_track3d_parameters_get_abort_dive_altitude(msg);
    zas_track3d_parameters->dive_pitch = mavlink_msg_zas_track3d_parameters_get_dive_pitch(msg);
    zas_track3d_parameters->dive_airspeed = mavlink_msg_zas_track3d_parameters_get_dive_airspeed(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_LEN? msg->len : MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_LEN;
        memset(zas_track3d_parameters, 0, MAVLINK_MSG_ID_ZAS_TRACK3D_PARAMETERS_LEN);
    memcpy(zas_track3d_parameters, _MAV_PAYLOAD(msg), len);
#endif
}
