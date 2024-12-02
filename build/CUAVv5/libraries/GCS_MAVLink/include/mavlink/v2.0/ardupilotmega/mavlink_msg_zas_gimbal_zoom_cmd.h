#pragma once
// MESSAGE ZAS_GIMBAL_ZOOM_CMD PACKING

#define MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD 2371


typedef struct __mavlink_zas_gimbal_zoom_cmd_t {
 int16_t Zoom; /*< [x]  Zoom value */
 int16_t ZoomRate; /*< [x/s]  Zoom rate value */
 uint16_t trk_x; /*< [pixels]  Tracking x pixel chosen by user via GCS cmd */
 uint16_t trk_y; /*< [pixels]  Tracking y pixel chosen by user via GCS cmd */
 int16_t offset_box; /*< [int]  Box to offset from center */
 uint8_t trk_cmd; /*< [uint8]  Command to track object in image plane */
 int8_t offset_x; /*< [pixels]  Offset pixels x */
 int8_t offset_y; /*< [pixels]  Offset pixels y */
 uint8_t mode; /*<   Gimbal mode for ZAS servo gimbal */
} mavlink_zas_gimbal_zoom_cmd_t;

#define MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_LEN 14
#define MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_MIN_LEN 14
#define MAVLINK_MSG_ID_2371_LEN 14
#define MAVLINK_MSG_ID_2371_MIN_LEN 14

#define MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_CRC 167
#define MAVLINK_MSG_ID_2371_CRC 167



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ZAS_GIMBAL_ZOOM_CMD { \
    2371, \
    "ZAS_GIMBAL_ZOOM_CMD", \
    9, \
    {  { "Zoom", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_zas_gimbal_zoom_cmd_t, Zoom) }, \
         { "ZoomRate", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_zas_gimbal_zoom_cmd_t, ZoomRate) }, \
         { "trk_x", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_zas_gimbal_zoom_cmd_t, trk_x) }, \
         { "trk_y", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_zas_gimbal_zoom_cmd_t, trk_y) }, \
         { "trk_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_zas_gimbal_zoom_cmd_t, trk_cmd) }, \
         { "offset_x", NULL, MAVLINK_TYPE_INT8_T, 0, 11, offsetof(mavlink_zas_gimbal_zoom_cmd_t, offset_x) }, \
         { "offset_y", NULL, MAVLINK_TYPE_INT8_T, 0, 12, offsetof(mavlink_zas_gimbal_zoom_cmd_t, offset_y) }, \
         { "offset_box", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_zas_gimbal_zoom_cmd_t, offset_box) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_zas_gimbal_zoom_cmd_t, mode) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ZAS_GIMBAL_ZOOM_CMD { \
    "ZAS_GIMBAL_ZOOM_CMD", \
    9, \
    {  { "Zoom", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_zas_gimbal_zoom_cmd_t, Zoom) }, \
         { "ZoomRate", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_zas_gimbal_zoom_cmd_t, ZoomRate) }, \
         { "trk_x", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_zas_gimbal_zoom_cmd_t, trk_x) }, \
         { "trk_y", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_zas_gimbal_zoom_cmd_t, trk_y) }, \
         { "trk_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_zas_gimbal_zoom_cmd_t, trk_cmd) }, \
         { "offset_x", NULL, MAVLINK_TYPE_INT8_T, 0, 11, offsetof(mavlink_zas_gimbal_zoom_cmd_t, offset_x) }, \
         { "offset_y", NULL, MAVLINK_TYPE_INT8_T, 0, 12, offsetof(mavlink_zas_gimbal_zoom_cmd_t, offset_y) }, \
         { "offset_box", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_zas_gimbal_zoom_cmd_t, offset_box) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_zas_gimbal_zoom_cmd_t, mode) }, \
         } \
}
#endif

/**
 * @brief Pack a zas_gimbal_zoom_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param Zoom [x]  Zoom value 
 * @param ZoomRate [x/s]  Zoom rate value 
 * @param trk_x [pixels]  Tracking x pixel chosen by user via GCS cmd 
 * @param trk_y [pixels]  Tracking y pixel chosen by user via GCS cmd 
 * @param trk_cmd [uint8]  Command to track object in image plane 
 * @param offset_x [pixels]  Offset pixels x 
 * @param offset_y [pixels]  Offset pixels y 
 * @param offset_box [int]  Box to offset from center 
 * @param mode   Gimbal mode for ZAS servo gimbal 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_zas_gimbal_zoom_cmd_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int16_t Zoom, int16_t ZoomRate, uint16_t trk_x, uint16_t trk_y, uint8_t trk_cmd, int8_t offset_x, int8_t offset_y, int16_t offset_box, uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_LEN];
    _mav_put_int16_t(buf, 0, Zoom);
    _mav_put_int16_t(buf, 2, ZoomRate);
    _mav_put_uint16_t(buf, 4, trk_x);
    _mav_put_uint16_t(buf, 6, trk_y);
    _mav_put_int16_t(buf, 8, offset_box);
    _mav_put_uint8_t(buf, 10, trk_cmd);
    _mav_put_int8_t(buf, 11, offset_x);
    _mav_put_int8_t(buf, 12, offset_y);
    _mav_put_uint8_t(buf, 13, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_LEN);
#else
    mavlink_zas_gimbal_zoom_cmd_t packet;
    packet.Zoom = Zoom;
    packet.ZoomRate = ZoomRate;
    packet.trk_x = trk_x;
    packet.trk_y = trk_y;
    packet.offset_box = offset_box;
    packet.trk_cmd = trk_cmd;
    packet.offset_x = offset_x;
    packet.offset_y = offset_y;
    packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_CRC);
}

/**
 * @brief Pack a zas_gimbal_zoom_cmd message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param Zoom [x]  Zoom value 
 * @param ZoomRate [x/s]  Zoom rate value 
 * @param trk_x [pixels]  Tracking x pixel chosen by user via GCS cmd 
 * @param trk_y [pixels]  Tracking y pixel chosen by user via GCS cmd 
 * @param trk_cmd [uint8]  Command to track object in image plane 
 * @param offset_x [pixels]  Offset pixels x 
 * @param offset_y [pixels]  Offset pixels y 
 * @param offset_box [int]  Box to offset from center 
 * @param mode   Gimbal mode for ZAS servo gimbal 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_zas_gimbal_zoom_cmd_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int16_t Zoom,int16_t ZoomRate,uint16_t trk_x,uint16_t trk_y,uint8_t trk_cmd,int8_t offset_x,int8_t offset_y,int16_t offset_box,uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_LEN];
    _mav_put_int16_t(buf, 0, Zoom);
    _mav_put_int16_t(buf, 2, ZoomRate);
    _mav_put_uint16_t(buf, 4, trk_x);
    _mav_put_uint16_t(buf, 6, trk_y);
    _mav_put_int16_t(buf, 8, offset_box);
    _mav_put_uint8_t(buf, 10, trk_cmd);
    _mav_put_int8_t(buf, 11, offset_x);
    _mav_put_int8_t(buf, 12, offset_y);
    _mav_put_uint8_t(buf, 13, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_LEN);
#else
    mavlink_zas_gimbal_zoom_cmd_t packet;
    packet.Zoom = Zoom;
    packet.ZoomRate = ZoomRate;
    packet.trk_x = trk_x;
    packet.trk_y = trk_y;
    packet.offset_box = offset_box;
    packet.trk_cmd = trk_cmd;
    packet.offset_x = offset_x;
    packet.offset_y = offset_y;
    packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_CRC);
}

/**
 * @brief Encode a zas_gimbal_zoom_cmd struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param zas_gimbal_zoom_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_zas_gimbal_zoom_cmd_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_zas_gimbal_zoom_cmd_t* zas_gimbal_zoom_cmd)
{
    return mavlink_msg_zas_gimbal_zoom_cmd_pack(system_id, component_id, msg, zas_gimbal_zoom_cmd->Zoom, zas_gimbal_zoom_cmd->ZoomRate, zas_gimbal_zoom_cmd->trk_x, zas_gimbal_zoom_cmd->trk_y, zas_gimbal_zoom_cmd->trk_cmd, zas_gimbal_zoom_cmd->offset_x, zas_gimbal_zoom_cmd->offset_y, zas_gimbal_zoom_cmd->offset_box, zas_gimbal_zoom_cmd->mode);
}

/**
 * @brief Encode a zas_gimbal_zoom_cmd struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param zas_gimbal_zoom_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_zas_gimbal_zoom_cmd_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_zas_gimbal_zoom_cmd_t* zas_gimbal_zoom_cmd)
{
    return mavlink_msg_zas_gimbal_zoom_cmd_pack_chan(system_id, component_id, chan, msg, zas_gimbal_zoom_cmd->Zoom, zas_gimbal_zoom_cmd->ZoomRate, zas_gimbal_zoom_cmd->trk_x, zas_gimbal_zoom_cmd->trk_y, zas_gimbal_zoom_cmd->trk_cmd, zas_gimbal_zoom_cmd->offset_x, zas_gimbal_zoom_cmd->offset_y, zas_gimbal_zoom_cmd->offset_box, zas_gimbal_zoom_cmd->mode);
}

/**
 * @brief Send a zas_gimbal_zoom_cmd message
 * @param chan MAVLink channel to send the message
 *
 * @param Zoom [x]  Zoom value 
 * @param ZoomRate [x/s]  Zoom rate value 
 * @param trk_x [pixels]  Tracking x pixel chosen by user via GCS cmd 
 * @param trk_y [pixels]  Tracking y pixel chosen by user via GCS cmd 
 * @param trk_cmd [uint8]  Command to track object in image plane 
 * @param offset_x [pixels]  Offset pixels x 
 * @param offset_y [pixels]  Offset pixels y 
 * @param offset_box [int]  Box to offset from center 
 * @param mode   Gimbal mode for ZAS servo gimbal 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_zas_gimbal_zoom_cmd_send(mavlink_channel_t chan, int16_t Zoom, int16_t ZoomRate, uint16_t trk_x, uint16_t trk_y, uint8_t trk_cmd, int8_t offset_x, int8_t offset_y, int16_t offset_box, uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_LEN];
    _mav_put_int16_t(buf, 0, Zoom);
    _mav_put_int16_t(buf, 2, ZoomRate);
    _mav_put_uint16_t(buf, 4, trk_x);
    _mav_put_uint16_t(buf, 6, trk_y);
    _mav_put_int16_t(buf, 8, offset_box);
    _mav_put_uint8_t(buf, 10, trk_cmd);
    _mav_put_int8_t(buf, 11, offset_x);
    _mav_put_int8_t(buf, 12, offset_y);
    _mav_put_uint8_t(buf, 13, mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD, buf, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_CRC);
#else
    mavlink_zas_gimbal_zoom_cmd_t packet;
    packet.Zoom = Zoom;
    packet.ZoomRate = ZoomRate;
    packet.trk_x = trk_x;
    packet.trk_y = trk_y;
    packet.offset_box = offset_box;
    packet.trk_cmd = trk_cmd;
    packet.offset_x = offset_x;
    packet.offset_y = offset_y;
    packet.mode = mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD, (const char *)&packet, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_CRC);
#endif
}

/**
 * @brief Send a zas_gimbal_zoom_cmd message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_zas_gimbal_zoom_cmd_send_struct(mavlink_channel_t chan, const mavlink_zas_gimbal_zoom_cmd_t* zas_gimbal_zoom_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_zas_gimbal_zoom_cmd_send(chan, zas_gimbal_zoom_cmd->Zoom, zas_gimbal_zoom_cmd->ZoomRate, zas_gimbal_zoom_cmd->trk_x, zas_gimbal_zoom_cmd->trk_y, zas_gimbal_zoom_cmd->trk_cmd, zas_gimbal_zoom_cmd->offset_x, zas_gimbal_zoom_cmd->offset_y, zas_gimbal_zoom_cmd->offset_box, zas_gimbal_zoom_cmd->mode);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD, (const char *)zas_gimbal_zoom_cmd, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_CRC);
#endif
}

#if MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_zas_gimbal_zoom_cmd_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int16_t Zoom, int16_t ZoomRate, uint16_t trk_x, uint16_t trk_y, uint8_t trk_cmd, int8_t offset_x, int8_t offset_y, int16_t offset_box, uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, Zoom);
    _mav_put_int16_t(buf, 2, ZoomRate);
    _mav_put_uint16_t(buf, 4, trk_x);
    _mav_put_uint16_t(buf, 6, trk_y);
    _mav_put_int16_t(buf, 8, offset_box);
    _mav_put_uint8_t(buf, 10, trk_cmd);
    _mav_put_int8_t(buf, 11, offset_x);
    _mav_put_int8_t(buf, 12, offset_y);
    _mav_put_uint8_t(buf, 13, mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD, buf, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_CRC);
#else
    mavlink_zas_gimbal_zoom_cmd_t *packet = (mavlink_zas_gimbal_zoom_cmd_t *)msgbuf;
    packet->Zoom = Zoom;
    packet->ZoomRate = ZoomRate;
    packet->trk_x = trk_x;
    packet->trk_y = trk_y;
    packet->offset_box = offset_box;
    packet->trk_cmd = trk_cmd;
    packet->offset_x = offset_x;
    packet->offset_y = offset_y;
    packet->mode = mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD, (const char *)packet, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_CRC);
#endif
}
#endif

#endif

// MESSAGE ZAS_GIMBAL_ZOOM_CMD UNPACKING


/**
 * @brief Get field Zoom from zas_gimbal_zoom_cmd message
 *
 * @return [x]  Zoom value 
 */
static inline int16_t mavlink_msg_zas_gimbal_zoom_cmd_get_Zoom(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field ZoomRate from zas_gimbal_zoom_cmd message
 *
 * @return [x/s]  Zoom rate value 
 */
static inline int16_t mavlink_msg_zas_gimbal_zoom_cmd_get_ZoomRate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field trk_x from zas_gimbal_zoom_cmd message
 *
 * @return [pixels]  Tracking x pixel chosen by user via GCS cmd 
 */
static inline uint16_t mavlink_msg_zas_gimbal_zoom_cmd_get_trk_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field trk_y from zas_gimbal_zoom_cmd message
 *
 * @return [pixels]  Tracking y pixel chosen by user via GCS cmd 
 */
static inline uint16_t mavlink_msg_zas_gimbal_zoom_cmd_get_trk_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field trk_cmd from zas_gimbal_zoom_cmd message
 *
 * @return [uint8]  Command to track object in image plane 
 */
static inline uint8_t mavlink_msg_zas_gimbal_zoom_cmd_get_trk_cmd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field offset_x from zas_gimbal_zoom_cmd message
 *
 * @return [pixels]  Offset pixels x 
 */
static inline int8_t mavlink_msg_zas_gimbal_zoom_cmd_get_offset_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  11);
}

/**
 * @brief Get field offset_y from zas_gimbal_zoom_cmd message
 *
 * @return [pixels]  Offset pixels y 
 */
static inline int8_t mavlink_msg_zas_gimbal_zoom_cmd_get_offset_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  12);
}

/**
 * @brief Get field offset_box from zas_gimbal_zoom_cmd message
 *
 * @return [int]  Box to offset from center 
 */
static inline int16_t mavlink_msg_zas_gimbal_zoom_cmd_get_offset_box(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field mode from zas_gimbal_zoom_cmd message
 *
 * @return   Gimbal mode for ZAS servo gimbal 
 */
static inline uint8_t mavlink_msg_zas_gimbal_zoom_cmd_get_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Decode a zas_gimbal_zoom_cmd message into a struct
 *
 * @param msg The message to decode
 * @param zas_gimbal_zoom_cmd C-struct to decode the message contents into
 */
static inline void mavlink_msg_zas_gimbal_zoom_cmd_decode(const mavlink_message_t* msg, mavlink_zas_gimbal_zoom_cmd_t* zas_gimbal_zoom_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    zas_gimbal_zoom_cmd->Zoom = mavlink_msg_zas_gimbal_zoom_cmd_get_Zoom(msg);
    zas_gimbal_zoom_cmd->ZoomRate = mavlink_msg_zas_gimbal_zoom_cmd_get_ZoomRate(msg);
    zas_gimbal_zoom_cmd->trk_x = mavlink_msg_zas_gimbal_zoom_cmd_get_trk_x(msg);
    zas_gimbal_zoom_cmd->trk_y = mavlink_msg_zas_gimbal_zoom_cmd_get_trk_y(msg);
    zas_gimbal_zoom_cmd->offset_box = mavlink_msg_zas_gimbal_zoom_cmd_get_offset_box(msg);
    zas_gimbal_zoom_cmd->trk_cmd = mavlink_msg_zas_gimbal_zoom_cmd_get_trk_cmd(msg);
    zas_gimbal_zoom_cmd->offset_x = mavlink_msg_zas_gimbal_zoom_cmd_get_offset_x(msg);
    zas_gimbal_zoom_cmd->offset_y = mavlink_msg_zas_gimbal_zoom_cmd_get_offset_y(msg);
    zas_gimbal_zoom_cmd->mode = mavlink_msg_zas_gimbal_zoom_cmd_get_mode(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_LEN? msg->len : MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_LEN;
        memset(zas_gimbal_zoom_cmd, 0, MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD_LEN);
    memcpy(zas_gimbal_zoom_cmd, _MAV_PAYLOAD(msg), len);
#endif
}
