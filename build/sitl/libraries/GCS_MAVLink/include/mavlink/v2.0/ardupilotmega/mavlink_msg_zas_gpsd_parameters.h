#pragma once
// MESSAGE ZAS_GPSD_PARAMETERS PACKING

#define MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS 54033


typedef struct __mavlink_zas_gpsd_parameters_t {
 int32_t approach_latitude; /*< [degE7]  Approach Latitude for Track 3D mode*/
 int32_t approach_longitude; /*< [degE7]  Approach Longitude for Track 3D mode*/
 int32_t approach_altitude; /*< [cm]  Approach Altitude AMSL cm for Track 3D mode*/
 int32_t target_latitude; /*< [degE7]  Target Latitude for Track 3D mode*/
 int32_t target_longitude; /*< [degE7]  Target Longitude for Track 3D mode*/
 int32_t target_altitude; /*< [cm]  Target Altitude AMSL cm for Track 3D mode*/
 int32_t abort_latitude; /*< [degE7]  Abort Latitude for Track 3D mode*/
 int32_t abort_longitude; /*< [degE7]  Abort Longitude for Track 3D mode*/
 int32_t abort_altitude; /*< [cm]  Abort Altitude AMSL cm for Track 3D mode*/
 uint16_t abort_dive_altitude; /*< [m]  Altitude above which dive can be aborted if track is lost*/
 uint16_t approach_loiter_radius; /*< [m]  Approach Loiter Radius [m]*/
 int16_t dive_pitch_saturation; /*< [cdegs]  Pitch Saturation -10 to -80*/
 int16_t pitch_offset; /*< [cdegs]  Pitch Offser 0 to 45*/
 uint16_t target_heading; /*< [cdegs] Target Heading in centi degrees*/
 uint8_t loiter_direction; /*< [0clockwise] Loiter Direction 0: Clockwise 1: Anticlockwise*/
} mavlink_zas_gpsd_parameters_t;

#define MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_LEN 47
#define MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_MIN_LEN 47
#define MAVLINK_MSG_ID_54033_LEN 47
#define MAVLINK_MSG_ID_54033_MIN_LEN 47

#define MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_CRC 139
#define MAVLINK_MSG_ID_54033_CRC 139



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ZAS_GPSD_PARAMETERS { \
    54033, \
    "ZAS_GPSD_PARAMETERS", \
    15, \
    {  { "abort_dive_altitude", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_zas_gpsd_parameters_t, abort_dive_altitude) }, \
         { "approach_loiter_radius", NULL, MAVLINK_TYPE_UINT16_T, 0, 38, offsetof(mavlink_zas_gpsd_parameters_t, approach_loiter_radius) }, \
         { "approach_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_zas_gpsd_parameters_t, approach_latitude) }, \
         { "approach_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_zas_gpsd_parameters_t, approach_longitude) }, \
         { "approach_altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_zas_gpsd_parameters_t, approach_altitude) }, \
         { "target_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_zas_gpsd_parameters_t, target_latitude) }, \
         { "target_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_zas_gpsd_parameters_t, target_longitude) }, \
         { "target_altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_zas_gpsd_parameters_t, target_altitude) }, \
         { "abort_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_zas_gpsd_parameters_t, abort_latitude) }, \
         { "abort_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 28, offsetof(mavlink_zas_gpsd_parameters_t, abort_longitude) }, \
         { "abort_altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 32, offsetof(mavlink_zas_gpsd_parameters_t, abort_altitude) }, \
         { "loiter_direction", NULL, MAVLINK_TYPE_UINT8_T, 0, 46, offsetof(mavlink_zas_gpsd_parameters_t, loiter_direction) }, \
         { "dive_pitch_saturation", NULL, MAVLINK_TYPE_INT16_T, 0, 40, offsetof(mavlink_zas_gpsd_parameters_t, dive_pitch_saturation) }, \
         { "pitch_offset", NULL, MAVLINK_TYPE_INT16_T, 0, 42, offsetof(mavlink_zas_gpsd_parameters_t, pitch_offset) }, \
         { "target_heading", NULL, MAVLINK_TYPE_UINT16_T, 0, 44, offsetof(mavlink_zas_gpsd_parameters_t, target_heading) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ZAS_GPSD_PARAMETERS { \
    "ZAS_GPSD_PARAMETERS", \
    15, \
    {  { "abort_dive_altitude", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_zas_gpsd_parameters_t, abort_dive_altitude) }, \
         { "approach_loiter_radius", NULL, MAVLINK_TYPE_UINT16_T, 0, 38, offsetof(mavlink_zas_gpsd_parameters_t, approach_loiter_radius) }, \
         { "approach_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_zas_gpsd_parameters_t, approach_latitude) }, \
         { "approach_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_zas_gpsd_parameters_t, approach_longitude) }, \
         { "approach_altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_zas_gpsd_parameters_t, approach_altitude) }, \
         { "target_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_zas_gpsd_parameters_t, target_latitude) }, \
         { "target_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_zas_gpsd_parameters_t, target_longitude) }, \
         { "target_altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_zas_gpsd_parameters_t, target_altitude) }, \
         { "abort_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_zas_gpsd_parameters_t, abort_latitude) }, \
         { "abort_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 28, offsetof(mavlink_zas_gpsd_parameters_t, abort_longitude) }, \
         { "abort_altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 32, offsetof(mavlink_zas_gpsd_parameters_t, abort_altitude) }, \
         { "loiter_direction", NULL, MAVLINK_TYPE_UINT8_T, 0, 46, offsetof(mavlink_zas_gpsd_parameters_t, loiter_direction) }, \
         { "dive_pitch_saturation", NULL, MAVLINK_TYPE_INT16_T, 0, 40, offsetof(mavlink_zas_gpsd_parameters_t, dive_pitch_saturation) }, \
         { "pitch_offset", NULL, MAVLINK_TYPE_INT16_T, 0, 42, offsetof(mavlink_zas_gpsd_parameters_t, pitch_offset) }, \
         { "target_heading", NULL, MAVLINK_TYPE_UINT16_T, 0, 44, offsetof(mavlink_zas_gpsd_parameters_t, target_heading) }, \
         } \
}
#endif

/**
 * @brief Pack a zas_gpsd_parameters message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param abort_dive_altitude [m]  Altitude above which dive can be aborted if track is lost
 * @param approach_loiter_radius [m]  Approach Loiter Radius [m]
 * @param approach_latitude [degE7]  Approach Latitude for Track 3D mode
 * @param approach_longitude [degE7]  Approach Longitude for Track 3D mode
 * @param approach_altitude [cm]  Approach Altitude AMSL cm for Track 3D mode
 * @param target_latitude [degE7]  Target Latitude for Track 3D mode
 * @param target_longitude [degE7]  Target Longitude for Track 3D mode
 * @param target_altitude [cm]  Target Altitude AMSL cm for Track 3D mode
 * @param abort_latitude [degE7]  Abort Latitude for Track 3D mode
 * @param abort_longitude [degE7]  Abort Longitude for Track 3D mode
 * @param abort_altitude [cm]  Abort Altitude AMSL cm for Track 3D mode
 * @param loiter_direction [0clockwise] Loiter Direction 0: Clockwise 1: Anticlockwise
 * @param dive_pitch_saturation [cdegs]  Pitch Saturation -10 to -80
 * @param pitch_offset [cdegs]  Pitch Offser 0 to 45
 * @param target_heading [cdegs] Target Heading in centi degrees
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_zas_gpsd_parameters_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t abort_dive_altitude, uint16_t approach_loiter_radius, int32_t approach_latitude, int32_t approach_longitude, int32_t approach_altitude, int32_t target_latitude, int32_t target_longitude, int32_t target_altitude, int32_t abort_latitude, int32_t abort_longitude, int32_t abort_altitude, uint8_t loiter_direction, int16_t dive_pitch_saturation, int16_t pitch_offset, uint16_t target_heading)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_LEN];
    _mav_put_int32_t(buf, 0, approach_latitude);
    _mav_put_int32_t(buf, 4, approach_longitude);
    _mav_put_int32_t(buf, 8, approach_altitude);
    _mav_put_int32_t(buf, 12, target_latitude);
    _mav_put_int32_t(buf, 16, target_longitude);
    _mav_put_int32_t(buf, 20, target_altitude);
    _mav_put_int32_t(buf, 24, abort_latitude);
    _mav_put_int32_t(buf, 28, abort_longitude);
    _mav_put_int32_t(buf, 32, abort_altitude);
    _mav_put_uint16_t(buf, 36, abort_dive_altitude);
    _mav_put_uint16_t(buf, 38, approach_loiter_radius);
    _mav_put_int16_t(buf, 40, dive_pitch_saturation);
    _mav_put_int16_t(buf, 42, pitch_offset);
    _mav_put_uint16_t(buf, 44, target_heading);
    _mav_put_uint8_t(buf, 46, loiter_direction);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_LEN);
#else
    mavlink_zas_gpsd_parameters_t packet;
    packet.approach_latitude = approach_latitude;
    packet.approach_longitude = approach_longitude;
    packet.approach_altitude = approach_altitude;
    packet.target_latitude = target_latitude;
    packet.target_longitude = target_longitude;
    packet.target_altitude = target_altitude;
    packet.abort_latitude = abort_latitude;
    packet.abort_longitude = abort_longitude;
    packet.abort_altitude = abort_altitude;
    packet.abort_dive_altitude = abort_dive_altitude;
    packet.approach_loiter_radius = approach_loiter_radius;
    packet.dive_pitch_saturation = dive_pitch_saturation;
    packet.pitch_offset = pitch_offset;
    packet.target_heading = target_heading;
    packet.loiter_direction = loiter_direction;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_MIN_LEN, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_LEN, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_CRC);
}

/**
 * @brief Pack a zas_gpsd_parameters message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param abort_dive_altitude [m]  Altitude above which dive can be aborted if track is lost
 * @param approach_loiter_radius [m]  Approach Loiter Radius [m]
 * @param approach_latitude [degE7]  Approach Latitude for Track 3D mode
 * @param approach_longitude [degE7]  Approach Longitude for Track 3D mode
 * @param approach_altitude [cm]  Approach Altitude AMSL cm for Track 3D mode
 * @param target_latitude [degE7]  Target Latitude for Track 3D mode
 * @param target_longitude [degE7]  Target Longitude for Track 3D mode
 * @param target_altitude [cm]  Target Altitude AMSL cm for Track 3D mode
 * @param abort_latitude [degE7]  Abort Latitude for Track 3D mode
 * @param abort_longitude [degE7]  Abort Longitude for Track 3D mode
 * @param abort_altitude [cm]  Abort Altitude AMSL cm for Track 3D mode
 * @param loiter_direction [0clockwise] Loiter Direction 0: Clockwise 1: Anticlockwise
 * @param dive_pitch_saturation [cdegs]  Pitch Saturation -10 to -80
 * @param pitch_offset [cdegs]  Pitch Offser 0 to 45
 * @param target_heading [cdegs] Target Heading in centi degrees
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_zas_gpsd_parameters_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t abort_dive_altitude,uint16_t approach_loiter_radius,int32_t approach_latitude,int32_t approach_longitude,int32_t approach_altitude,int32_t target_latitude,int32_t target_longitude,int32_t target_altitude,int32_t abort_latitude,int32_t abort_longitude,int32_t abort_altitude,uint8_t loiter_direction,int16_t dive_pitch_saturation,int16_t pitch_offset,uint16_t target_heading)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_LEN];
    _mav_put_int32_t(buf, 0, approach_latitude);
    _mav_put_int32_t(buf, 4, approach_longitude);
    _mav_put_int32_t(buf, 8, approach_altitude);
    _mav_put_int32_t(buf, 12, target_latitude);
    _mav_put_int32_t(buf, 16, target_longitude);
    _mav_put_int32_t(buf, 20, target_altitude);
    _mav_put_int32_t(buf, 24, abort_latitude);
    _mav_put_int32_t(buf, 28, abort_longitude);
    _mav_put_int32_t(buf, 32, abort_altitude);
    _mav_put_uint16_t(buf, 36, abort_dive_altitude);
    _mav_put_uint16_t(buf, 38, approach_loiter_radius);
    _mav_put_int16_t(buf, 40, dive_pitch_saturation);
    _mav_put_int16_t(buf, 42, pitch_offset);
    _mav_put_uint16_t(buf, 44, target_heading);
    _mav_put_uint8_t(buf, 46, loiter_direction);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_LEN);
#else
    mavlink_zas_gpsd_parameters_t packet;
    packet.approach_latitude = approach_latitude;
    packet.approach_longitude = approach_longitude;
    packet.approach_altitude = approach_altitude;
    packet.target_latitude = target_latitude;
    packet.target_longitude = target_longitude;
    packet.target_altitude = target_altitude;
    packet.abort_latitude = abort_latitude;
    packet.abort_longitude = abort_longitude;
    packet.abort_altitude = abort_altitude;
    packet.abort_dive_altitude = abort_dive_altitude;
    packet.approach_loiter_radius = approach_loiter_radius;
    packet.dive_pitch_saturation = dive_pitch_saturation;
    packet.pitch_offset = pitch_offset;
    packet.target_heading = target_heading;
    packet.loiter_direction = loiter_direction;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_MIN_LEN, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_LEN, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_CRC);
}

/**
 * @brief Encode a zas_gpsd_parameters struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param zas_gpsd_parameters C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_zas_gpsd_parameters_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_zas_gpsd_parameters_t* zas_gpsd_parameters)
{
    return mavlink_msg_zas_gpsd_parameters_pack(system_id, component_id, msg, zas_gpsd_parameters->abort_dive_altitude, zas_gpsd_parameters->approach_loiter_radius, zas_gpsd_parameters->approach_latitude, zas_gpsd_parameters->approach_longitude, zas_gpsd_parameters->approach_altitude, zas_gpsd_parameters->target_latitude, zas_gpsd_parameters->target_longitude, zas_gpsd_parameters->target_altitude, zas_gpsd_parameters->abort_latitude, zas_gpsd_parameters->abort_longitude, zas_gpsd_parameters->abort_altitude, zas_gpsd_parameters->loiter_direction, zas_gpsd_parameters->dive_pitch_saturation, zas_gpsd_parameters->pitch_offset, zas_gpsd_parameters->target_heading);
}

/**
 * @brief Encode a zas_gpsd_parameters struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param zas_gpsd_parameters C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_zas_gpsd_parameters_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_zas_gpsd_parameters_t* zas_gpsd_parameters)
{
    return mavlink_msg_zas_gpsd_parameters_pack_chan(system_id, component_id, chan, msg, zas_gpsd_parameters->abort_dive_altitude, zas_gpsd_parameters->approach_loiter_radius, zas_gpsd_parameters->approach_latitude, zas_gpsd_parameters->approach_longitude, zas_gpsd_parameters->approach_altitude, zas_gpsd_parameters->target_latitude, zas_gpsd_parameters->target_longitude, zas_gpsd_parameters->target_altitude, zas_gpsd_parameters->abort_latitude, zas_gpsd_parameters->abort_longitude, zas_gpsd_parameters->abort_altitude, zas_gpsd_parameters->loiter_direction, zas_gpsd_parameters->dive_pitch_saturation, zas_gpsd_parameters->pitch_offset, zas_gpsd_parameters->target_heading);
}

/**
 * @brief Send a zas_gpsd_parameters message
 * @param chan MAVLink channel to send the message
 *
 * @param abort_dive_altitude [m]  Altitude above which dive can be aborted if track is lost
 * @param approach_loiter_radius [m]  Approach Loiter Radius [m]
 * @param approach_latitude [degE7]  Approach Latitude for Track 3D mode
 * @param approach_longitude [degE7]  Approach Longitude for Track 3D mode
 * @param approach_altitude [cm]  Approach Altitude AMSL cm for Track 3D mode
 * @param target_latitude [degE7]  Target Latitude for Track 3D mode
 * @param target_longitude [degE7]  Target Longitude for Track 3D mode
 * @param target_altitude [cm]  Target Altitude AMSL cm for Track 3D mode
 * @param abort_latitude [degE7]  Abort Latitude for Track 3D mode
 * @param abort_longitude [degE7]  Abort Longitude for Track 3D mode
 * @param abort_altitude [cm]  Abort Altitude AMSL cm for Track 3D mode
 * @param loiter_direction [0clockwise] Loiter Direction 0: Clockwise 1: Anticlockwise
 * @param dive_pitch_saturation [cdegs]  Pitch Saturation -10 to -80
 * @param pitch_offset [cdegs]  Pitch Offser 0 to 45
 * @param target_heading [cdegs] Target Heading in centi degrees
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_zas_gpsd_parameters_send(mavlink_channel_t chan, uint16_t abort_dive_altitude, uint16_t approach_loiter_radius, int32_t approach_latitude, int32_t approach_longitude, int32_t approach_altitude, int32_t target_latitude, int32_t target_longitude, int32_t target_altitude, int32_t abort_latitude, int32_t abort_longitude, int32_t abort_altitude, uint8_t loiter_direction, int16_t dive_pitch_saturation, int16_t pitch_offset, uint16_t target_heading)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_LEN];
    _mav_put_int32_t(buf, 0, approach_latitude);
    _mav_put_int32_t(buf, 4, approach_longitude);
    _mav_put_int32_t(buf, 8, approach_altitude);
    _mav_put_int32_t(buf, 12, target_latitude);
    _mav_put_int32_t(buf, 16, target_longitude);
    _mav_put_int32_t(buf, 20, target_altitude);
    _mav_put_int32_t(buf, 24, abort_latitude);
    _mav_put_int32_t(buf, 28, abort_longitude);
    _mav_put_int32_t(buf, 32, abort_altitude);
    _mav_put_uint16_t(buf, 36, abort_dive_altitude);
    _mav_put_uint16_t(buf, 38, approach_loiter_radius);
    _mav_put_int16_t(buf, 40, dive_pitch_saturation);
    _mav_put_int16_t(buf, 42, pitch_offset);
    _mav_put_uint16_t(buf, 44, target_heading);
    _mav_put_uint8_t(buf, 46, loiter_direction);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS, buf, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_MIN_LEN, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_LEN, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_CRC);
#else
    mavlink_zas_gpsd_parameters_t packet;
    packet.approach_latitude = approach_latitude;
    packet.approach_longitude = approach_longitude;
    packet.approach_altitude = approach_altitude;
    packet.target_latitude = target_latitude;
    packet.target_longitude = target_longitude;
    packet.target_altitude = target_altitude;
    packet.abort_latitude = abort_latitude;
    packet.abort_longitude = abort_longitude;
    packet.abort_altitude = abort_altitude;
    packet.abort_dive_altitude = abort_dive_altitude;
    packet.approach_loiter_radius = approach_loiter_radius;
    packet.dive_pitch_saturation = dive_pitch_saturation;
    packet.pitch_offset = pitch_offset;
    packet.target_heading = target_heading;
    packet.loiter_direction = loiter_direction;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS, (const char *)&packet, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_MIN_LEN, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_LEN, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_CRC);
#endif
}

/**
 * @brief Send a zas_gpsd_parameters message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_zas_gpsd_parameters_send_struct(mavlink_channel_t chan, const mavlink_zas_gpsd_parameters_t* zas_gpsd_parameters)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_zas_gpsd_parameters_send(chan, zas_gpsd_parameters->abort_dive_altitude, zas_gpsd_parameters->approach_loiter_radius, zas_gpsd_parameters->approach_latitude, zas_gpsd_parameters->approach_longitude, zas_gpsd_parameters->approach_altitude, zas_gpsd_parameters->target_latitude, zas_gpsd_parameters->target_longitude, zas_gpsd_parameters->target_altitude, zas_gpsd_parameters->abort_latitude, zas_gpsd_parameters->abort_longitude, zas_gpsd_parameters->abort_altitude, zas_gpsd_parameters->loiter_direction, zas_gpsd_parameters->dive_pitch_saturation, zas_gpsd_parameters->pitch_offset, zas_gpsd_parameters->target_heading);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS, (const char *)zas_gpsd_parameters, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_MIN_LEN, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_LEN, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_zas_gpsd_parameters_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t abort_dive_altitude, uint16_t approach_loiter_radius, int32_t approach_latitude, int32_t approach_longitude, int32_t approach_altitude, int32_t target_latitude, int32_t target_longitude, int32_t target_altitude, int32_t abort_latitude, int32_t abort_longitude, int32_t abort_altitude, uint8_t loiter_direction, int16_t dive_pitch_saturation, int16_t pitch_offset, uint16_t target_heading)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, approach_latitude);
    _mav_put_int32_t(buf, 4, approach_longitude);
    _mav_put_int32_t(buf, 8, approach_altitude);
    _mav_put_int32_t(buf, 12, target_latitude);
    _mav_put_int32_t(buf, 16, target_longitude);
    _mav_put_int32_t(buf, 20, target_altitude);
    _mav_put_int32_t(buf, 24, abort_latitude);
    _mav_put_int32_t(buf, 28, abort_longitude);
    _mav_put_int32_t(buf, 32, abort_altitude);
    _mav_put_uint16_t(buf, 36, abort_dive_altitude);
    _mav_put_uint16_t(buf, 38, approach_loiter_radius);
    _mav_put_int16_t(buf, 40, dive_pitch_saturation);
    _mav_put_int16_t(buf, 42, pitch_offset);
    _mav_put_uint16_t(buf, 44, target_heading);
    _mav_put_uint8_t(buf, 46, loiter_direction);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS, buf, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_MIN_LEN, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_LEN, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_CRC);
#else
    mavlink_zas_gpsd_parameters_t *packet = (mavlink_zas_gpsd_parameters_t *)msgbuf;
    packet->approach_latitude = approach_latitude;
    packet->approach_longitude = approach_longitude;
    packet->approach_altitude = approach_altitude;
    packet->target_latitude = target_latitude;
    packet->target_longitude = target_longitude;
    packet->target_altitude = target_altitude;
    packet->abort_latitude = abort_latitude;
    packet->abort_longitude = abort_longitude;
    packet->abort_altitude = abort_altitude;
    packet->abort_dive_altitude = abort_dive_altitude;
    packet->approach_loiter_radius = approach_loiter_radius;
    packet->dive_pitch_saturation = dive_pitch_saturation;
    packet->pitch_offset = pitch_offset;
    packet->target_heading = target_heading;
    packet->loiter_direction = loiter_direction;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS, (const char *)packet, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_MIN_LEN, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_LEN, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_CRC);
#endif
}
#endif

#endif

// MESSAGE ZAS_GPSD_PARAMETERS UNPACKING


/**
 * @brief Get field abort_dive_altitude from zas_gpsd_parameters message
 *
 * @return [m]  Altitude above which dive can be aborted if track is lost
 */
static inline uint16_t mavlink_msg_zas_gpsd_parameters_get_abort_dive_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  36);
}

/**
 * @brief Get field approach_loiter_radius from zas_gpsd_parameters message
 *
 * @return [m]  Approach Loiter Radius [m]
 */
static inline uint16_t mavlink_msg_zas_gpsd_parameters_get_approach_loiter_radius(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  38);
}

/**
 * @brief Get field approach_latitude from zas_gpsd_parameters message
 *
 * @return [degE7]  Approach Latitude for Track 3D mode
 */
static inline int32_t mavlink_msg_zas_gpsd_parameters_get_approach_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field approach_longitude from zas_gpsd_parameters message
 *
 * @return [degE7]  Approach Longitude for Track 3D mode
 */
static inline int32_t mavlink_msg_zas_gpsd_parameters_get_approach_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field approach_altitude from zas_gpsd_parameters message
 *
 * @return [cm]  Approach Altitude AMSL cm for Track 3D mode
 */
static inline int32_t mavlink_msg_zas_gpsd_parameters_get_approach_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field target_latitude from zas_gpsd_parameters message
 *
 * @return [degE7]  Target Latitude for Track 3D mode
 */
static inline int32_t mavlink_msg_zas_gpsd_parameters_get_target_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field target_longitude from zas_gpsd_parameters message
 *
 * @return [degE7]  Target Longitude for Track 3D mode
 */
static inline int32_t mavlink_msg_zas_gpsd_parameters_get_target_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field target_altitude from zas_gpsd_parameters message
 *
 * @return [cm]  Target Altitude AMSL cm for Track 3D mode
 */
static inline int32_t mavlink_msg_zas_gpsd_parameters_get_target_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field abort_latitude from zas_gpsd_parameters message
 *
 * @return [degE7]  Abort Latitude for Track 3D mode
 */
static inline int32_t mavlink_msg_zas_gpsd_parameters_get_abort_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  24);
}

/**
 * @brief Get field abort_longitude from zas_gpsd_parameters message
 *
 * @return [degE7]  Abort Longitude for Track 3D mode
 */
static inline int32_t mavlink_msg_zas_gpsd_parameters_get_abort_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  28);
}

/**
 * @brief Get field abort_altitude from zas_gpsd_parameters message
 *
 * @return [cm]  Abort Altitude AMSL cm for Track 3D mode
 */
static inline int32_t mavlink_msg_zas_gpsd_parameters_get_abort_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  32);
}

/**
 * @brief Get field loiter_direction from zas_gpsd_parameters message
 *
 * @return [0clockwise] Loiter Direction 0: Clockwise 1: Anticlockwise
 */
static inline uint8_t mavlink_msg_zas_gpsd_parameters_get_loiter_direction(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  46);
}

/**
 * @brief Get field dive_pitch_saturation from zas_gpsd_parameters message
 *
 * @return [cdegs]  Pitch Saturation -10 to -80
 */
static inline int16_t mavlink_msg_zas_gpsd_parameters_get_dive_pitch_saturation(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  40);
}

/**
 * @brief Get field pitch_offset from zas_gpsd_parameters message
 *
 * @return [cdegs]  Pitch Offser 0 to 45
 */
static inline int16_t mavlink_msg_zas_gpsd_parameters_get_pitch_offset(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  42);
}

/**
 * @brief Get field target_heading from zas_gpsd_parameters message
 *
 * @return [cdegs] Target Heading in centi degrees
 */
static inline uint16_t mavlink_msg_zas_gpsd_parameters_get_target_heading(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  44);
}

/**
 * @brief Decode a zas_gpsd_parameters message into a struct
 *
 * @param msg The message to decode
 * @param zas_gpsd_parameters C-struct to decode the message contents into
 */
static inline void mavlink_msg_zas_gpsd_parameters_decode(const mavlink_message_t* msg, mavlink_zas_gpsd_parameters_t* zas_gpsd_parameters)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    zas_gpsd_parameters->approach_latitude = mavlink_msg_zas_gpsd_parameters_get_approach_latitude(msg);
    zas_gpsd_parameters->approach_longitude = mavlink_msg_zas_gpsd_parameters_get_approach_longitude(msg);
    zas_gpsd_parameters->approach_altitude = mavlink_msg_zas_gpsd_parameters_get_approach_altitude(msg);
    zas_gpsd_parameters->target_latitude = mavlink_msg_zas_gpsd_parameters_get_target_latitude(msg);
    zas_gpsd_parameters->target_longitude = mavlink_msg_zas_gpsd_parameters_get_target_longitude(msg);
    zas_gpsd_parameters->target_altitude = mavlink_msg_zas_gpsd_parameters_get_target_altitude(msg);
    zas_gpsd_parameters->abort_latitude = mavlink_msg_zas_gpsd_parameters_get_abort_latitude(msg);
    zas_gpsd_parameters->abort_longitude = mavlink_msg_zas_gpsd_parameters_get_abort_longitude(msg);
    zas_gpsd_parameters->abort_altitude = mavlink_msg_zas_gpsd_parameters_get_abort_altitude(msg);
    zas_gpsd_parameters->abort_dive_altitude = mavlink_msg_zas_gpsd_parameters_get_abort_dive_altitude(msg);
    zas_gpsd_parameters->approach_loiter_radius = mavlink_msg_zas_gpsd_parameters_get_approach_loiter_radius(msg);
    zas_gpsd_parameters->dive_pitch_saturation = mavlink_msg_zas_gpsd_parameters_get_dive_pitch_saturation(msg);
    zas_gpsd_parameters->pitch_offset = mavlink_msg_zas_gpsd_parameters_get_pitch_offset(msg);
    zas_gpsd_parameters->target_heading = mavlink_msg_zas_gpsd_parameters_get_target_heading(msg);
    zas_gpsd_parameters->loiter_direction = mavlink_msg_zas_gpsd_parameters_get_loiter_direction(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_LEN? msg->len : MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_LEN;
        memset(zas_gpsd_parameters, 0, MAVLINK_MSG_ID_ZAS_GPSD_PARAMETERS_LEN);
    memcpy(zas_gpsd_parameters, _MAV_PAYLOAD(msg), len);
#endif
}
