#pragma once
// MESSAGE ZAS_TRACK_PARAMETERS PACKING

#define MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS 54032


typedef struct __mavlink_zas_track_parameters_t {
 int32_t max_misses_before_tracking_lost; /*< [0-20]  Maximum number of times that track is missed before track lost is declared*/
 int32_t step_value; /*< [1-100]  Step between consecutive PAN/TILT goal*/
 uint16_t confidence_threshold; /*< [0-100]  Confidence level for tracking*/
 int16_t center_offset_pixel_delta; /*< [pixels]  The center target square*/
 int16_t cam0_height; /*< [pixels]  Camera 0 Resolution Height*/
 int16_t cam0_width; /*< [pixels]  Camera 0 Resolution Width*/
 int16_t cam1_height; /*< [pixels]  Camera 0 Resolution Height*/
 int16_t cam1_width; /*< [pixels]  Camera 0 Resolution Width*/
} mavlink_zas_track_parameters_t;

#define MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_LEN 20
#define MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_MIN_LEN 20
#define MAVLINK_MSG_ID_54032_LEN 20
#define MAVLINK_MSG_ID_54032_MIN_LEN 20

#define MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_CRC 97
#define MAVLINK_MSG_ID_54032_CRC 97



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ZAS_TRACK_PARAMETERS { \
    54032, \
    "ZAS_TRACK_PARAMETERS", \
    8, \
    {  { "confidence_threshold", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_zas_track_parameters_t, confidence_threshold) }, \
         { "max_misses_before_tracking_lost", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_zas_track_parameters_t, max_misses_before_tracking_lost) }, \
         { "step_value", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_zas_track_parameters_t, step_value) }, \
         { "center_offset_pixel_delta", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_zas_track_parameters_t, center_offset_pixel_delta) }, \
         { "cam0_height", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_zas_track_parameters_t, cam0_height) }, \
         { "cam0_width", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_zas_track_parameters_t, cam0_width) }, \
         { "cam1_height", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_zas_track_parameters_t, cam1_height) }, \
         { "cam1_width", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_zas_track_parameters_t, cam1_width) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ZAS_TRACK_PARAMETERS { \
    "ZAS_TRACK_PARAMETERS", \
    8, \
    {  { "confidence_threshold", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_zas_track_parameters_t, confidence_threshold) }, \
         { "max_misses_before_tracking_lost", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_zas_track_parameters_t, max_misses_before_tracking_lost) }, \
         { "step_value", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_zas_track_parameters_t, step_value) }, \
         { "center_offset_pixel_delta", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_zas_track_parameters_t, center_offset_pixel_delta) }, \
         { "cam0_height", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_zas_track_parameters_t, cam0_height) }, \
         { "cam0_width", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_zas_track_parameters_t, cam0_width) }, \
         { "cam1_height", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_zas_track_parameters_t, cam1_height) }, \
         { "cam1_width", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_zas_track_parameters_t, cam1_width) }, \
         } \
}
#endif

/**
 * @brief Pack a zas_track_parameters message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param confidence_threshold [0-100]  Confidence level for tracking
 * @param max_misses_before_tracking_lost [0-20]  Maximum number of times that track is missed before track lost is declared
 * @param step_value [1-100]  Step between consecutive PAN/TILT goal
 * @param center_offset_pixel_delta [pixels]  The center target square
 * @param cam0_height [pixels]  Camera 0 Resolution Height
 * @param cam0_width [pixels]  Camera 0 Resolution Width
 * @param cam1_height [pixels]  Camera 0 Resolution Height
 * @param cam1_width [pixels]  Camera 0 Resolution Width
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_zas_track_parameters_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t confidence_threshold, int32_t max_misses_before_tracking_lost, int32_t step_value, int16_t center_offset_pixel_delta, int16_t cam0_height, int16_t cam0_width, int16_t cam1_height, int16_t cam1_width)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_LEN];
    _mav_put_int32_t(buf, 0, max_misses_before_tracking_lost);
    _mav_put_int32_t(buf, 4, step_value);
    _mav_put_uint16_t(buf, 8, confidence_threshold);
    _mav_put_int16_t(buf, 10, center_offset_pixel_delta);
    _mav_put_int16_t(buf, 12, cam0_height);
    _mav_put_int16_t(buf, 14, cam0_width);
    _mav_put_int16_t(buf, 16, cam1_height);
    _mav_put_int16_t(buf, 18, cam1_width);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_LEN);
#else
    mavlink_zas_track_parameters_t packet;
    packet.max_misses_before_tracking_lost = max_misses_before_tracking_lost;
    packet.step_value = step_value;
    packet.confidence_threshold = confidence_threshold;
    packet.center_offset_pixel_delta = center_offset_pixel_delta;
    packet.cam0_height = cam0_height;
    packet.cam0_width = cam0_width;
    packet.cam1_height = cam1_height;
    packet.cam1_width = cam1_width;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_MIN_LEN, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_LEN, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_CRC);
}

/**
 * @brief Pack a zas_track_parameters message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param confidence_threshold [0-100]  Confidence level for tracking
 * @param max_misses_before_tracking_lost [0-20]  Maximum number of times that track is missed before track lost is declared
 * @param step_value [1-100]  Step between consecutive PAN/TILT goal
 * @param center_offset_pixel_delta [pixels]  The center target square
 * @param cam0_height [pixels]  Camera 0 Resolution Height
 * @param cam0_width [pixels]  Camera 0 Resolution Width
 * @param cam1_height [pixels]  Camera 0 Resolution Height
 * @param cam1_width [pixels]  Camera 0 Resolution Width
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_zas_track_parameters_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t confidence_threshold,int32_t max_misses_before_tracking_lost,int32_t step_value,int16_t center_offset_pixel_delta,int16_t cam0_height,int16_t cam0_width,int16_t cam1_height,int16_t cam1_width)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_LEN];
    _mav_put_int32_t(buf, 0, max_misses_before_tracking_lost);
    _mav_put_int32_t(buf, 4, step_value);
    _mav_put_uint16_t(buf, 8, confidence_threshold);
    _mav_put_int16_t(buf, 10, center_offset_pixel_delta);
    _mav_put_int16_t(buf, 12, cam0_height);
    _mav_put_int16_t(buf, 14, cam0_width);
    _mav_put_int16_t(buf, 16, cam1_height);
    _mav_put_int16_t(buf, 18, cam1_width);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_LEN);
#else
    mavlink_zas_track_parameters_t packet;
    packet.max_misses_before_tracking_lost = max_misses_before_tracking_lost;
    packet.step_value = step_value;
    packet.confidence_threshold = confidence_threshold;
    packet.center_offset_pixel_delta = center_offset_pixel_delta;
    packet.cam0_height = cam0_height;
    packet.cam0_width = cam0_width;
    packet.cam1_height = cam1_height;
    packet.cam1_width = cam1_width;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_MIN_LEN, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_LEN, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_CRC);
}

/**
 * @brief Encode a zas_track_parameters struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param zas_track_parameters C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_zas_track_parameters_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_zas_track_parameters_t* zas_track_parameters)
{
    return mavlink_msg_zas_track_parameters_pack(system_id, component_id, msg, zas_track_parameters->confidence_threshold, zas_track_parameters->max_misses_before_tracking_lost, zas_track_parameters->step_value, zas_track_parameters->center_offset_pixel_delta, zas_track_parameters->cam0_height, zas_track_parameters->cam0_width, zas_track_parameters->cam1_height, zas_track_parameters->cam1_width);
}

/**
 * @brief Encode a zas_track_parameters struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param zas_track_parameters C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_zas_track_parameters_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_zas_track_parameters_t* zas_track_parameters)
{
    return mavlink_msg_zas_track_parameters_pack_chan(system_id, component_id, chan, msg, zas_track_parameters->confidence_threshold, zas_track_parameters->max_misses_before_tracking_lost, zas_track_parameters->step_value, zas_track_parameters->center_offset_pixel_delta, zas_track_parameters->cam0_height, zas_track_parameters->cam0_width, zas_track_parameters->cam1_height, zas_track_parameters->cam1_width);
}

/**
 * @brief Send a zas_track_parameters message
 * @param chan MAVLink channel to send the message
 *
 * @param confidence_threshold [0-100]  Confidence level for tracking
 * @param max_misses_before_tracking_lost [0-20]  Maximum number of times that track is missed before track lost is declared
 * @param step_value [1-100]  Step between consecutive PAN/TILT goal
 * @param center_offset_pixel_delta [pixels]  The center target square
 * @param cam0_height [pixels]  Camera 0 Resolution Height
 * @param cam0_width [pixels]  Camera 0 Resolution Width
 * @param cam1_height [pixels]  Camera 0 Resolution Height
 * @param cam1_width [pixels]  Camera 0 Resolution Width
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_zas_track_parameters_send(mavlink_channel_t chan, uint16_t confidence_threshold, int32_t max_misses_before_tracking_lost, int32_t step_value, int16_t center_offset_pixel_delta, int16_t cam0_height, int16_t cam0_width, int16_t cam1_height, int16_t cam1_width)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_LEN];
    _mav_put_int32_t(buf, 0, max_misses_before_tracking_lost);
    _mav_put_int32_t(buf, 4, step_value);
    _mav_put_uint16_t(buf, 8, confidence_threshold);
    _mav_put_int16_t(buf, 10, center_offset_pixel_delta);
    _mav_put_int16_t(buf, 12, cam0_height);
    _mav_put_int16_t(buf, 14, cam0_width);
    _mav_put_int16_t(buf, 16, cam1_height);
    _mav_put_int16_t(buf, 18, cam1_width);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS, buf, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_MIN_LEN, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_LEN, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_CRC);
#else
    mavlink_zas_track_parameters_t packet;
    packet.max_misses_before_tracking_lost = max_misses_before_tracking_lost;
    packet.step_value = step_value;
    packet.confidence_threshold = confidence_threshold;
    packet.center_offset_pixel_delta = center_offset_pixel_delta;
    packet.cam0_height = cam0_height;
    packet.cam0_width = cam0_width;
    packet.cam1_height = cam1_height;
    packet.cam1_width = cam1_width;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS, (const char *)&packet, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_MIN_LEN, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_LEN, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_CRC);
#endif
}

/**
 * @brief Send a zas_track_parameters message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_zas_track_parameters_send_struct(mavlink_channel_t chan, const mavlink_zas_track_parameters_t* zas_track_parameters)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_zas_track_parameters_send(chan, zas_track_parameters->confidence_threshold, zas_track_parameters->max_misses_before_tracking_lost, zas_track_parameters->step_value, zas_track_parameters->center_offset_pixel_delta, zas_track_parameters->cam0_height, zas_track_parameters->cam0_width, zas_track_parameters->cam1_height, zas_track_parameters->cam1_width);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS, (const char *)zas_track_parameters, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_MIN_LEN, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_LEN, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_zas_track_parameters_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t confidence_threshold, int32_t max_misses_before_tracking_lost, int32_t step_value, int16_t center_offset_pixel_delta, int16_t cam0_height, int16_t cam0_width, int16_t cam1_height, int16_t cam1_width)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, max_misses_before_tracking_lost);
    _mav_put_int32_t(buf, 4, step_value);
    _mav_put_uint16_t(buf, 8, confidence_threshold);
    _mav_put_int16_t(buf, 10, center_offset_pixel_delta);
    _mav_put_int16_t(buf, 12, cam0_height);
    _mav_put_int16_t(buf, 14, cam0_width);
    _mav_put_int16_t(buf, 16, cam1_height);
    _mav_put_int16_t(buf, 18, cam1_width);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS, buf, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_MIN_LEN, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_LEN, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_CRC);
#else
    mavlink_zas_track_parameters_t *packet = (mavlink_zas_track_parameters_t *)msgbuf;
    packet->max_misses_before_tracking_lost = max_misses_before_tracking_lost;
    packet->step_value = step_value;
    packet->confidence_threshold = confidence_threshold;
    packet->center_offset_pixel_delta = center_offset_pixel_delta;
    packet->cam0_height = cam0_height;
    packet->cam0_width = cam0_width;
    packet->cam1_height = cam1_height;
    packet->cam1_width = cam1_width;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS, (const char *)packet, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_MIN_LEN, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_LEN, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_CRC);
#endif
}
#endif

#endif

// MESSAGE ZAS_TRACK_PARAMETERS UNPACKING


/**
 * @brief Get field confidence_threshold from zas_track_parameters message
 *
 * @return [0-100]  Confidence level for tracking
 */
static inline uint16_t mavlink_msg_zas_track_parameters_get_confidence_threshold(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field max_misses_before_tracking_lost from zas_track_parameters message
 *
 * @return [0-20]  Maximum number of times that track is missed before track lost is declared
 */
static inline int32_t mavlink_msg_zas_track_parameters_get_max_misses_before_tracking_lost(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field step_value from zas_track_parameters message
 *
 * @return [1-100]  Step between consecutive PAN/TILT goal
 */
static inline int32_t mavlink_msg_zas_track_parameters_get_step_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field center_offset_pixel_delta from zas_track_parameters message
 *
 * @return [pixels]  The center target square
 */
static inline int16_t mavlink_msg_zas_track_parameters_get_center_offset_pixel_delta(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field cam0_height from zas_track_parameters message
 *
 * @return [pixels]  Camera 0 Resolution Height
 */
static inline int16_t mavlink_msg_zas_track_parameters_get_cam0_height(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field cam0_width from zas_track_parameters message
 *
 * @return [pixels]  Camera 0 Resolution Width
 */
static inline int16_t mavlink_msg_zas_track_parameters_get_cam0_width(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field cam1_height from zas_track_parameters message
 *
 * @return [pixels]  Camera 0 Resolution Height
 */
static inline int16_t mavlink_msg_zas_track_parameters_get_cam1_height(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field cam1_width from zas_track_parameters message
 *
 * @return [pixels]  Camera 0 Resolution Width
 */
static inline int16_t mavlink_msg_zas_track_parameters_get_cam1_width(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  18);
}

/**
 * @brief Decode a zas_track_parameters message into a struct
 *
 * @param msg The message to decode
 * @param zas_track_parameters C-struct to decode the message contents into
 */
static inline void mavlink_msg_zas_track_parameters_decode(const mavlink_message_t* msg, mavlink_zas_track_parameters_t* zas_track_parameters)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    zas_track_parameters->max_misses_before_tracking_lost = mavlink_msg_zas_track_parameters_get_max_misses_before_tracking_lost(msg);
    zas_track_parameters->step_value = mavlink_msg_zas_track_parameters_get_step_value(msg);
    zas_track_parameters->confidence_threshold = mavlink_msg_zas_track_parameters_get_confidence_threshold(msg);
    zas_track_parameters->center_offset_pixel_delta = mavlink_msg_zas_track_parameters_get_center_offset_pixel_delta(msg);
    zas_track_parameters->cam0_height = mavlink_msg_zas_track_parameters_get_cam0_height(msg);
    zas_track_parameters->cam0_width = mavlink_msg_zas_track_parameters_get_cam0_width(msg);
    zas_track_parameters->cam1_height = mavlink_msg_zas_track_parameters_get_cam1_height(msg);
    zas_track_parameters->cam1_width = mavlink_msg_zas_track_parameters_get_cam1_width(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_LEN? msg->len : MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_LEN;
        memset(zas_track_parameters, 0, MAVLINK_MSG_ID_ZAS_TRACK_PARAMETERS_LEN);
    memcpy(zas_track_parameters, _MAV_PAYLOAD(msg), len);
#endif
}
