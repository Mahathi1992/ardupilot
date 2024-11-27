#pragma once
// MESSAGE ZAS_TRACK_STATUS PACKING

#define MAVLINK_MSG_ID_ZAS_TRACK_STATUS 54031


typedef struct __mavlink_zas_track_status_t {
 int32_t track_latitude; /*< [degE7]  Tracked Latitude for Track 2D/3D mode*/
 int32_t track_longitude; /*< [degE7]  Tracked Longitude for Track 2D/3D mode*/
 int16_t num_tracks; /*< [units]  Number of tracks for tracking*/
 int16_t pan; /*< [units]  Current PAN for MX servo*/
 int16_t tilt; /*< [units]  Current TILT for MX servo*/
 int16_t commanded_pan; /*< [units]  Current commanded PAN for MX servo*/
 int16_t commanded_tilt; /*< [units]  Current commanded tilt for MX servo*/
 int8_t selected_cam; /*< [units]  Selected cam for diving*/
} mavlink_zas_track_status_t;

#define MAVLINK_MSG_ID_ZAS_TRACK_STATUS_LEN 19
#define MAVLINK_MSG_ID_ZAS_TRACK_STATUS_MIN_LEN 19
#define MAVLINK_MSG_ID_54031_LEN 19
#define MAVLINK_MSG_ID_54031_MIN_LEN 19

#define MAVLINK_MSG_ID_ZAS_TRACK_STATUS_CRC 100
#define MAVLINK_MSG_ID_54031_CRC 100



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ZAS_TRACK_STATUS { \
    54031, \
    "ZAS_TRACK_STATUS", \
    8, \
    {  { "track_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_zas_track_status_t, track_latitude) }, \
         { "track_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_zas_track_status_t, track_longitude) }, \
         { "num_tracks", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_zas_track_status_t, num_tracks) }, \
         { "selected_cam", NULL, MAVLINK_TYPE_INT8_T, 0, 18, offsetof(mavlink_zas_track_status_t, selected_cam) }, \
         { "pan", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_zas_track_status_t, pan) }, \
         { "tilt", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_zas_track_status_t, tilt) }, \
         { "commanded_pan", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_zas_track_status_t, commanded_pan) }, \
         { "commanded_tilt", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_zas_track_status_t, commanded_tilt) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ZAS_TRACK_STATUS { \
    "ZAS_TRACK_STATUS", \
    8, \
    {  { "track_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_zas_track_status_t, track_latitude) }, \
         { "track_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_zas_track_status_t, track_longitude) }, \
         { "num_tracks", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_zas_track_status_t, num_tracks) }, \
         { "selected_cam", NULL, MAVLINK_TYPE_INT8_T, 0, 18, offsetof(mavlink_zas_track_status_t, selected_cam) }, \
         { "pan", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_zas_track_status_t, pan) }, \
         { "tilt", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_zas_track_status_t, tilt) }, \
         { "commanded_pan", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_zas_track_status_t, commanded_pan) }, \
         { "commanded_tilt", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_zas_track_status_t, commanded_tilt) }, \
         } \
}
#endif

/**
 * @brief Pack a zas_track_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param track_latitude [degE7]  Tracked Latitude for Track 2D/3D mode
 * @param track_longitude [degE7]  Tracked Longitude for Track 2D/3D mode
 * @param num_tracks [units]  Number of tracks for tracking
 * @param selected_cam [units]  Selected cam for diving
 * @param pan [units]  Current PAN for MX servo
 * @param tilt [units]  Current TILT for MX servo
 * @param commanded_pan [units]  Current commanded PAN for MX servo
 * @param commanded_tilt [units]  Current commanded tilt for MX servo
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_zas_track_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t track_latitude, int32_t track_longitude, int16_t num_tracks, int8_t selected_cam, int16_t pan, int16_t tilt, int16_t commanded_pan, int16_t commanded_tilt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZAS_TRACK_STATUS_LEN];
    _mav_put_int32_t(buf, 0, track_latitude);
    _mav_put_int32_t(buf, 4, track_longitude);
    _mav_put_int16_t(buf, 8, num_tracks);
    _mav_put_int16_t(buf, 10, pan);
    _mav_put_int16_t(buf, 12, tilt);
    _mav_put_int16_t(buf, 14, commanded_pan);
    _mav_put_int16_t(buf, 16, commanded_tilt);
    _mav_put_int8_t(buf, 18, selected_cam);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ZAS_TRACK_STATUS_LEN);
#else
    mavlink_zas_track_status_t packet;
    packet.track_latitude = track_latitude;
    packet.track_longitude = track_longitude;
    packet.num_tracks = num_tracks;
    packet.pan = pan;
    packet.tilt = tilt;
    packet.commanded_pan = commanded_pan;
    packet.commanded_tilt = commanded_tilt;
    packet.selected_cam = selected_cam;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ZAS_TRACK_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ZAS_TRACK_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ZAS_TRACK_STATUS_MIN_LEN, MAVLINK_MSG_ID_ZAS_TRACK_STATUS_LEN, MAVLINK_MSG_ID_ZAS_TRACK_STATUS_CRC);
}

/**
 * @brief Pack a zas_track_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param track_latitude [degE7]  Tracked Latitude for Track 2D/3D mode
 * @param track_longitude [degE7]  Tracked Longitude for Track 2D/3D mode
 * @param num_tracks [units]  Number of tracks for tracking
 * @param selected_cam [units]  Selected cam for diving
 * @param pan [units]  Current PAN for MX servo
 * @param tilt [units]  Current TILT for MX servo
 * @param commanded_pan [units]  Current commanded PAN for MX servo
 * @param commanded_tilt [units]  Current commanded tilt for MX servo
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_zas_track_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t track_latitude,int32_t track_longitude,int16_t num_tracks,int8_t selected_cam,int16_t pan,int16_t tilt,int16_t commanded_pan,int16_t commanded_tilt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZAS_TRACK_STATUS_LEN];
    _mav_put_int32_t(buf, 0, track_latitude);
    _mav_put_int32_t(buf, 4, track_longitude);
    _mav_put_int16_t(buf, 8, num_tracks);
    _mav_put_int16_t(buf, 10, pan);
    _mav_put_int16_t(buf, 12, tilt);
    _mav_put_int16_t(buf, 14, commanded_pan);
    _mav_put_int16_t(buf, 16, commanded_tilt);
    _mav_put_int8_t(buf, 18, selected_cam);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ZAS_TRACK_STATUS_LEN);
#else
    mavlink_zas_track_status_t packet;
    packet.track_latitude = track_latitude;
    packet.track_longitude = track_longitude;
    packet.num_tracks = num_tracks;
    packet.pan = pan;
    packet.tilt = tilt;
    packet.commanded_pan = commanded_pan;
    packet.commanded_tilt = commanded_tilt;
    packet.selected_cam = selected_cam;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ZAS_TRACK_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ZAS_TRACK_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ZAS_TRACK_STATUS_MIN_LEN, MAVLINK_MSG_ID_ZAS_TRACK_STATUS_LEN, MAVLINK_MSG_ID_ZAS_TRACK_STATUS_CRC);
}

/**
 * @brief Encode a zas_track_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param zas_track_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_zas_track_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_zas_track_status_t* zas_track_status)
{
    return mavlink_msg_zas_track_status_pack(system_id, component_id, msg, zas_track_status->track_latitude, zas_track_status->track_longitude, zas_track_status->num_tracks, zas_track_status->selected_cam, zas_track_status->pan, zas_track_status->tilt, zas_track_status->commanded_pan, zas_track_status->commanded_tilt);
}

/**
 * @brief Encode a zas_track_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param zas_track_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_zas_track_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_zas_track_status_t* zas_track_status)
{
    return mavlink_msg_zas_track_status_pack_chan(system_id, component_id, chan, msg, zas_track_status->track_latitude, zas_track_status->track_longitude, zas_track_status->num_tracks, zas_track_status->selected_cam, zas_track_status->pan, zas_track_status->tilt, zas_track_status->commanded_pan, zas_track_status->commanded_tilt);
}

/**
 * @brief Send a zas_track_status message
 * @param chan MAVLink channel to send the message
 *
 * @param track_latitude [degE7]  Tracked Latitude for Track 2D/3D mode
 * @param track_longitude [degE7]  Tracked Longitude for Track 2D/3D mode
 * @param num_tracks [units]  Number of tracks for tracking
 * @param selected_cam [units]  Selected cam for diving
 * @param pan [units]  Current PAN for MX servo
 * @param tilt [units]  Current TILT for MX servo
 * @param commanded_pan [units]  Current commanded PAN for MX servo
 * @param commanded_tilt [units]  Current commanded tilt for MX servo
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_zas_track_status_send(mavlink_channel_t chan, int32_t track_latitude, int32_t track_longitude, int16_t num_tracks, int8_t selected_cam, int16_t pan, int16_t tilt, int16_t commanded_pan, int16_t commanded_tilt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZAS_TRACK_STATUS_LEN];
    _mav_put_int32_t(buf, 0, track_latitude);
    _mav_put_int32_t(buf, 4, track_longitude);
    _mav_put_int16_t(buf, 8, num_tracks);
    _mav_put_int16_t(buf, 10, pan);
    _mav_put_int16_t(buf, 12, tilt);
    _mav_put_int16_t(buf, 14, commanded_pan);
    _mav_put_int16_t(buf, 16, commanded_tilt);
    _mav_put_int8_t(buf, 18, selected_cam);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_TRACK_STATUS, buf, MAVLINK_MSG_ID_ZAS_TRACK_STATUS_MIN_LEN, MAVLINK_MSG_ID_ZAS_TRACK_STATUS_LEN, MAVLINK_MSG_ID_ZAS_TRACK_STATUS_CRC);
#else
    mavlink_zas_track_status_t packet;
    packet.track_latitude = track_latitude;
    packet.track_longitude = track_longitude;
    packet.num_tracks = num_tracks;
    packet.pan = pan;
    packet.tilt = tilt;
    packet.commanded_pan = commanded_pan;
    packet.commanded_tilt = commanded_tilt;
    packet.selected_cam = selected_cam;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_TRACK_STATUS, (const char *)&packet, MAVLINK_MSG_ID_ZAS_TRACK_STATUS_MIN_LEN, MAVLINK_MSG_ID_ZAS_TRACK_STATUS_LEN, MAVLINK_MSG_ID_ZAS_TRACK_STATUS_CRC);
#endif
}

/**
 * @brief Send a zas_track_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_zas_track_status_send_struct(mavlink_channel_t chan, const mavlink_zas_track_status_t* zas_track_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_zas_track_status_send(chan, zas_track_status->track_latitude, zas_track_status->track_longitude, zas_track_status->num_tracks, zas_track_status->selected_cam, zas_track_status->pan, zas_track_status->tilt, zas_track_status->commanded_pan, zas_track_status->commanded_tilt);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_TRACK_STATUS, (const char *)zas_track_status, MAVLINK_MSG_ID_ZAS_TRACK_STATUS_MIN_LEN, MAVLINK_MSG_ID_ZAS_TRACK_STATUS_LEN, MAVLINK_MSG_ID_ZAS_TRACK_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ZAS_TRACK_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_zas_track_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t track_latitude, int32_t track_longitude, int16_t num_tracks, int8_t selected_cam, int16_t pan, int16_t tilt, int16_t commanded_pan, int16_t commanded_tilt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, track_latitude);
    _mav_put_int32_t(buf, 4, track_longitude);
    _mav_put_int16_t(buf, 8, num_tracks);
    _mav_put_int16_t(buf, 10, pan);
    _mav_put_int16_t(buf, 12, tilt);
    _mav_put_int16_t(buf, 14, commanded_pan);
    _mav_put_int16_t(buf, 16, commanded_tilt);
    _mav_put_int8_t(buf, 18, selected_cam);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_TRACK_STATUS, buf, MAVLINK_MSG_ID_ZAS_TRACK_STATUS_MIN_LEN, MAVLINK_MSG_ID_ZAS_TRACK_STATUS_LEN, MAVLINK_MSG_ID_ZAS_TRACK_STATUS_CRC);
#else
    mavlink_zas_track_status_t *packet = (mavlink_zas_track_status_t *)msgbuf;
    packet->track_latitude = track_latitude;
    packet->track_longitude = track_longitude;
    packet->num_tracks = num_tracks;
    packet->pan = pan;
    packet->tilt = tilt;
    packet->commanded_pan = commanded_pan;
    packet->commanded_tilt = commanded_tilt;
    packet->selected_cam = selected_cam;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_TRACK_STATUS, (const char *)packet, MAVLINK_MSG_ID_ZAS_TRACK_STATUS_MIN_LEN, MAVLINK_MSG_ID_ZAS_TRACK_STATUS_LEN, MAVLINK_MSG_ID_ZAS_TRACK_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE ZAS_TRACK_STATUS UNPACKING


/**
 * @brief Get field track_latitude from zas_track_status message
 *
 * @return [degE7]  Tracked Latitude for Track 2D/3D mode
 */
static inline int32_t mavlink_msg_zas_track_status_get_track_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field track_longitude from zas_track_status message
 *
 * @return [degE7]  Tracked Longitude for Track 2D/3D mode
 */
static inline int32_t mavlink_msg_zas_track_status_get_track_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field num_tracks from zas_track_status message
 *
 * @return [units]  Number of tracks for tracking
 */
static inline int16_t mavlink_msg_zas_track_status_get_num_tracks(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field selected_cam from zas_track_status message
 *
 * @return [units]  Selected cam for diving
 */
static inline int8_t mavlink_msg_zas_track_status_get_selected_cam(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  18);
}

/**
 * @brief Get field pan from zas_track_status message
 *
 * @return [units]  Current PAN for MX servo
 */
static inline int16_t mavlink_msg_zas_track_status_get_pan(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field tilt from zas_track_status message
 *
 * @return [units]  Current TILT for MX servo
 */
static inline int16_t mavlink_msg_zas_track_status_get_tilt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field commanded_pan from zas_track_status message
 *
 * @return [units]  Current commanded PAN for MX servo
 */
static inline int16_t mavlink_msg_zas_track_status_get_commanded_pan(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field commanded_tilt from zas_track_status message
 *
 * @return [units]  Current commanded tilt for MX servo
 */
static inline int16_t mavlink_msg_zas_track_status_get_commanded_tilt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Decode a zas_track_status message into a struct
 *
 * @param msg The message to decode
 * @param zas_track_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_zas_track_status_decode(const mavlink_message_t* msg, mavlink_zas_track_status_t* zas_track_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    zas_track_status->track_latitude = mavlink_msg_zas_track_status_get_track_latitude(msg);
    zas_track_status->track_longitude = mavlink_msg_zas_track_status_get_track_longitude(msg);
    zas_track_status->num_tracks = mavlink_msg_zas_track_status_get_num_tracks(msg);
    zas_track_status->pan = mavlink_msg_zas_track_status_get_pan(msg);
    zas_track_status->tilt = mavlink_msg_zas_track_status_get_tilt(msg);
    zas_track_status->commanded_pan = mavlink_msg_zas_track_status_get_commanded_pan(msg);
    zas_track_status->commanded_tilt = mavlink_msg_zas_track_status_get_commanded_tilt(msg);
    zas_track_status->selected_cam = mavlink_msg_zas_track_status_get_selected_cam(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ZAS_TRACK_STATUS_LEN? msg->len : MAVLINK_MSG_ID_ZAS_TRACK_STATUS_LEN;
        memset(zas_track_status, 0, MAVLINK_MSG_ID_ZAS_TRACK_STATUS_LEN);
    memcpy(zas_track_status, _MAV_PAYLOAD(msg), len);
#endif
}
