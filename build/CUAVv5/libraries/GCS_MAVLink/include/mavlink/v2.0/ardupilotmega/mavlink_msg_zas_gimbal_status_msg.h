#pragma once
// MESSAGE ZAS_GIMBAL_STATUS_MSG PACKING

#define MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG 238


typedef struct __mavlink_zas_gimbal_status_msg_t {
 float cmd_pan; /*< [deg]  Commanded pan status */
 float cmd_tilt; /*< [deg]  Commanded tilt status */
 float pan; /*< [deg]  Current pan status in deg */
 float tilt; /*< [deg]  Current tilt status in deg */
 float elevation; /*< [deg]  Current elevation in global frame status in deg */
 float azimuth; /*< [deg]  Current azimuth in global frame status in deg */
 int16_t trk_x; /*< [pixels]  Tracking x pixel (current) */
 int16_t trk_y; /*< [pixels]  Tracking y pixel (current) */
 uint8_t Zoom; /*< [x]  Zoom value */
 uint8_t wb_mode; /*< [x]  white hot/black hot */
 uint8_t mode; /*<   Gimbal mode for ZAS servo gimbal */
 uint8_t cam_type; /*< [x]  Camera type value */
 uint8_t trk_status; /*< [uint8]  Tracker status */
} mavlink_zas_gimbal_status_msg_t;

#define MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_LEN 33
#define MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_MIN_LEN 33
#define MAVLINK_MSG_ID_238_LEN 33
#define MAVLINK_MSG_ID_238_MIN_LEN 33

#define MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_CRC 12
#define MAVLINK_MSG_ID_238_CRC 12



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ZAS_GIMBAL_STATUS_MSG { \
    238, \
    "ZAS_GIMBAL_STATUS_MSG", \
    13, \
    {  { "trk_x", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_zas_gimbal_status_msg_t, trk_x) }, \
         { "trk_y", NULL, MAVLINK_TYPE_INT16_T, 0, 26, offsetof(mavlink_zas_gimbal_status_msg_t, trk_y) }, \
         { "cmd_pan", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_zas_gimbal_status_msg_t, cmd_pan) }, \
         { "cmd_tilt", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_zas_gimbal_status_msg_t, cmd_tilt) }, \
         { "pan", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_zas_gimbal_status_msg_t, pan) }, \
         { "tilt", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_zas_gimbal_status_msg_t, tilt) }, \
         { "elevation", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_zas_gimbal_status_msg_t, elevation) }, \
         { "azimuth", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_zas_gimbal_status_msg_t, azimuth) }, \
         { "Zoom", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_zas_gimbal_status_msg_t, Zoom) }, \
         { "wb_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_zas_gimbal_status_msg_t, wb_mode) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_zas_gimbal_status_msg_t, mode) }, \
         { "cam_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_zas_gimbal_status_msg_t, cam_type) }, \
         { "trk_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_zas_gimbal_status_msg_t, trk_status) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ZAS_GIMBAL_STATUS_MSG { \
    "ZAS_GIMBAL_STATUS_MSG", \
    13, \
    {  { "trk_x", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_zas_gimbal_status_msg_t, trk_x) }, \
         { "trk_y", NULL, MAVLINK_TYPE_INT16_T, 0, 26, offsetof(mavlink_zas_gimbal_status_msg_t, trk_y) }, \
         { "cmd_pan", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_zas_gimbal_status_msg_t, cmd_pan) }, \
         { "cmd_tilt", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_zas_gimbal_status_msg_t, cmd_tilt) }, \
         { "pan", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_zas_gimbal_status_msg_t, pan) }, \
         { "tilt", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_zas_gimbal_status_msg_t, tilt) }, \
         { "elevation", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_zas_gimbal_status_msg_t, elevation) }, \
         { "azimuth", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_zas_gimbal_status_msg_t, azimuth) }, \
         { "Zoom", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_zas_gimbal_status_msg_t, Zoom) }, \
         { "wb_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_zas_gimbal_status_msg_t, wb_mode) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_zas_gimbal_status_msg_t, mode) }, \
         { "cam_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_zas_gimbal_status_msg_t, cam_type) }, \
         { "trk_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_zas_gimbal_status_msg_t, trk_status) }, \
         } \
}
#endif

/**
 * @brief Pack a zas_gimbal_status_msg message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param trk_x [pixels]  Tracking x pixel (current) 
 * @param trk_y [pixels]  Tracking y pixel (current) 
 * @param cmd_pan [deg]  Commanded pan status 
 * @param cmd_tilt [deg]  Commanded tilt status 
 * @param pan [deg]  Current pan status in deg 
 * @param tilt [deg]  Current tilt status in deg 
 * @param elevation [deg]  Current elevation in global frame status in deg 
 * @param azimuth [deg]  Current azimuth in global frame status in deg 
 * @param Zoom [x]  Zoom value 
 * @param wb_mode [x]  white hot/black hot 
 * @param mode   Gimbal mode for ZAS servo gimbal 
 * @param cam_type [x]  Camera type value 
 * @param trk_status [uint8]  Tracker status 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_zas_gimbal_status_msg_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int16_t trk_x, int16_t trk_y, float cmd_pan, float cmd_tilt, float pan, float tilt, float elevation, float azimuth, uint8_t Zoom, uint8_t wb_mode, uint8_t mode, uint8_t cam_type, uint8_t trk_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_LEN];
    _mav_put_float(buf, 0, cmd_pan);
    _mav_put_float(buf, 4, cmd_tilt);
    _mav_put_float(buf, 8, pan);
    _mav_put_float(buf, 12, tilt);
    _mav_put_float(buf, 16, elevation);
    _mav_put_float(buf, 20, azimuth);
    _mav_put_int16_t(buf, 24, trk_x);
    _mav_put_int16_t(buf, 26, trk_y);
    _mav_put_uint8_t(buf, 28, Zoom);
    _mav_put_uint8_t(buf, 29, wb_mode);
    _mav_put_uint8_t(buf, 30, mode);
    _mav_put_uint8_t(buf, 31, cam_type);
    _mav_put_uint8_t(buf, 32, trk_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_LEN);
#else
    mavlink_zas_gimbal_status_msg_t packet;
    packet.cmd_pan = cmd_pan;
    packet.cmd_tilt = cmd_tilt;
    packet.pan = pan;
    packet.tilt = tilt;
    packet.elevation = elevation;
    packet.azimuth = azimuth;
    packet.trk_x = trk_x;
    packet.trk_y = trk_y;
    packet.Zoom = Zoom;
    packet.wb_mode = wb_mode;
    packet.mode = mode;
    packet.cam_type = cam_type;
    packet.trk_status = trk_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_CRC);
}

/**
 * @brief Pack a zas_gimbal_status_msg message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param trk_x [pixels]  Tracking x pixel (current) 
 * @param trk_y [pixels]  Tracking y pixel (current) 
 * @param cmd_pan [deg]  Commanded pan status 
 * @param cmd_tilt [deg]  Commanded tilt status 
 * @param pan [deg]  Current pan status in deg 
 * @param tilt [deg]  Current tilt status in deg 
 * @param elevation [deg]  Current elevation in global frame status in deg 
 * @param azimuth [deg]  Current azimuth in global frame status in deg 
 * @param Zoom [x]  Zoom value 
 * @param wb_mode [x]  white hot/black hot 
 * @param mode   Gimbal mode for ZAS servo gimbal 
 * @param cam_type [x]  Camera type value 
 * @param trk_status [uint8]  Tracker status 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_zas_gimbal_status_msg_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int16_t trk_x,int16_t trk_y,float cmd_pan,float cmd_tilt,float pan,float tilt,float elevation,float azimuth,uint8_t Zoom,uint8_t wb_mode,uint8_t mode,uint8_t cam_type,uint8_t trk_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_LEN];
    _mav_put_float(buf, 0, cmd_pan);
    _mav_put_float(buf, 4, cmd_tilt);
    _mav_put_float(buf, 8, pan);
    _mav_put_float(buf, 12, tilt);
    _mav_put_float(buf, 16, elevation);
    _mav_put_float(buf, 20, azimuth);
    _mav_put_int16_t(buf, 24, trk_x);
    _mav_put_int16_t(buf, 26, trk_y);
    _mav_put_uint8_t(buf, 28, Zoom);
    _mav_put_uint8_t(buf, 29, wb_mode);
    _mav_put_uint8_t(buf, 30, mode);
    _mav_put_uint8_t(buf, 31, cam_type);
    _mav_put_uint8_t(buf, 32, trk_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_LEN);
#else
    mavlink_zas_gimbal_status_msg_t packet;
    packet.cmd_pan = cmd_pan;
    packet.cmd_tilt = cmd_tilt;
    packet.pan = pan;
    packet.tilt = tilt;
    packet.elevation = elevation;
    packet.azimuth = azimuth;
    packet.trk_x = trk_x;
    packet.trk_y = trk_y;
    packet.Zoom = Zoom;
    packet.wb_mode = wb_mode;
    packet.mode = mode;
    packet.cam_type = cam_type;
    packet.trk_status = trk_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_CRC);
}

/**
 * @brief Encode a zas_gimbal_status_msg struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param zas_gimbal_status_msg C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_zas_gimbal_status_msg_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_zas_gimbal_status_msg_t* zas_gimbal_status_msg)
{
    return mavlink_msg_zas_gimbal_status_msg_pack(system_id, component_id, msg, zas_gimbal_status_msg->trk_x, zas_gimbal_status_msg->trk_y, zas_gimbal_status_msg->cmd_pan, zas_gimbal_status_msg->cmd_tilt, zas_gimbal_status_msg->pan, zas_gimbal_status_msg->tilt, zas_gimbal_status_msg->elevation, zas_gimbal_status_msg->azimuth, zas_gimbal_status_msg->Zoom, zas_gimbal_status_msg->wb_mode, zas_gimbal_status_msg->mode, zas_gimbal_status_msg->cam_type, zas_gimbal_status_msg->trk_status);
}

/**
 * @brief Encode a zas_gimbal_status_msg struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param zas_gimbal_status_msg C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_zas_gimbal_status_msg_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_zas_gimbal_status_msg_t* zas_gimbal_status_msg)
{
    return mavlink_msg_zas_gimbal_status_msg_pack_chan(system_id, component_id, chan, msg, zas_gimbal_status_msg->trk_x, zas_gimbal_status_msg->trk_y, zas_gimbal_status_msg->cmd_pan, zas_gimbal_status_msg->cmd_tilt, zas_gimbal_status_msg->pan, zas_gimbal_status_msg->tilt, zas_gimbal_status_msg->elevation, zas_gimbal_status_msg->azimuth, zas_gimbal_status_msg->Zoom, zas_gimbal_status_msg->wb_mode, zas_gimbal_status_msg->mode, zas_gimbal_status_msg->cam_type, zas_gimbal_status_msg->trk_status);
}

/**
 * @brief Send a zas_gimbal_status_msg message
 * @param chan MAVLink channel to send the message
 *
 * @param trk_x [pixels]  Tracking x pixel (current) 
 * @param trk_y [pixels]  Tracking y pixel (current) 
 * @param cmd_pan [deg]  Commanded pan status 
 * @param cmd_tilt [deg]  Commanded tilt status 
 * @param pan [deg]  Current pan status in deg 
 * @param tilt [deg]  Current tilt status in deg 
 * @param elevation [deg]  Current elevation in global frame status in deg 
 * @param azimuth [deg]  Current azimuth in global frame status in deg 
 * @param Zoom [x]  Zoom value 
 * @param wb_mode [x]  white hot/black hot 
 * @param mode   Gimbal mode for ZAS servo gimbal 
 * @param cam_type [x]  Camera type value 
 * @param trk_status [uint8]  Tracker status 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_zas_gimbal_status_msg_send(mavlink_channel_t chan, int16_t trk_x, int16_t trk_y, float cmd_pan, float cmd_tilt, float pan, float tilt, float elevation, float azimuth, uint8_t Zoom, uint8_t wb_mode, uint8_t mode, uint8_t cam_type, uint8_t trk_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_LEN];
    _mav_put_float(buf, 0, cmd_pan);
    _mav_put_float(buf, 4, cmd_tilt);
    _mav_put_float(buf, 8, pan);
    _mav_put_float(buf, 12, tilt);
    _mav_put_float(buf, 16, elevation);
    _mav_put_float(buf, 20, azimuth);
    _mav_put_int16_t(buf, 24, trk_x);
    _mav_put_int16_t(buf, 26, trk_y);
    _mav_put_uint8_t(buf, 28, Zoom);
    _mav_put_uint8_t(buf, 29, wb_mode);
    _mav_put_uint8_t(buf, 30, mode);
    _mav_put_uint8_t(buf, 31, cam_type);
    _mav_put_uint8_t(buf, 32, trk_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG, buf, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_CRC);
#else
    mavlink_zas_gimbal_status_msg_t packet;
    packet.cmd_pan = cmd_pan;
    packet.cmd_tilt = cmd_tilt;
    packet.pan = pan;
    packet.tilt = tilt;
    packet.elevation = elevation;
    packet.azimuth = azimuth;
    packet.trk_x = trk_x;
    packet.trk_y = trk_y;
    packet.Zoom = Zoom;
    packet.wb_mode = wb_mode;
    packet.mode = mode;
    packet.cam_type = cam_type;
    packet.trk_status = trk_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG, (const char *)&packet, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_CRC);
#endif
}

/**
 * @brief Send a zas_gimbal_status_msg message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_zas_gimbal_status_msg_send_struct(mavlink_channel_t chan, const mavlink_zas_gimbal_status_msg_t* zas_gimbal_status_msg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_zas_gimbal_status_msg_send(chan, zas_gimbal_status_msg->trk_x, zas_gimbal_status_msg->trk_y, zas_gimbal_status_msg->cmd_pan, zas_gimbal_status_msg->cmd_tilt, zas_gimbal_status_msg->pan, zas_gimbal_status_msg->tilt, zas_gimbal_status_msg->elevation, zas_gimbal_status_msg->azimuth, zas_gimbal_status_msg->Zoom, zas_gimbal_status_msg->wb_mode, zas_gimbal_status_msg->mode, zas_gimbal_status_msg->cam_type, zas_gimbal_status_msg->trk_status);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG, (const char *)zas_gimbal_status_msg, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_CRC);
#endif
}

#if MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_zas_gimbal_status_msg_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int16_t trk_x, int16_t trk_y, float cmd_pan, float cmd_tilt, float pan, float tilt, float elevation, float azimuth, uint8_t Zoom, uint8_t wb_mode, uint8_t mode, uint8_t cam_type, uint8_t trk_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, cmd_pan);
    _mav_put_float(buf, 4, cmd_tilt);
    _mav_put_float(buf, 8, pan);
    _mav_put_float(buf, 12, tilt);
    _mav_put_float(buf, 16, elevation);
    _mav_put_float(buf, 20, azimuth);
    _mav_put_int16_t(buf, 24, trk_x);
    _mav_put_int16_t(buf, 26, trk_y);
    _mav_put_uint8_t(buf, 28, Zoom);
    _mav_put_uint8_t(buf, 29, wb_mode);
    _mav_put_uint8_t(buf, 30, mode);
    _mav_put_uint8_t(buf, 31, cam_type);
    _mav_put_uint8_t(buf, 32, trk_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG, buf, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_CRC);
#else
    mavlink_zas_gimbal_status_msg_t *packet = (mavlink_zas_gimbal_status_msg_t *)msgbuf;
    packet->cmd_pan = cmd_pan;
    packet->cmd_tilt = cmd_tilt;
    packet->pan = pan;
    packet->tilt = tilt;
    packet->elevation = elevation;
    packet->azimuth = azimuth;
    packet->trk_x = trk_x;
    packet->trk_y = trk_y;
    packet->Zoom = Zoom;
    packet->wb_mode = wb_mode;
    packet->mode = mode;
    packet->cam_type = cam_type;
    packet->trk_status = trk_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG, (const char *)packet, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_MIN_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_LEN, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_CRC);
#endif
}
#endif

#endif

// MESSAGE ZAS_GIMBAL_STATUS_MSG UNPACKING


/**
 * @brief Get field trk_x from zas_gimbal_status_msg message
 *
 * @return [pixels]  Tracking x pixel (current) 
 */
static inline int16_t mavlink_msg_zas_gimbal_status_msg_get_trk_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  24);
}

/**
 * @brief Get field trk_y from zas_gimbal_status_msg message
 *
 * @return [pixels]  Tracking y pixel (current) 
 */
static inline int16_t mavlink_msg_zas_gimbal_status_msg_get_trk_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  26);
}

/**
 * @brief Get field cmd_pan from zas_gimbal_status_msg message
 *
 * @return [deg]  Commanded pan status 
 */
static inline float mavlink_msg_zas_gimbal_status_msg_get_cmd_pan(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field cmd_tilt from zas_gimbal_status_msg message
 *
 * @return [deg]  Commanded tilt status 
 */
static inline float mavlink_msg_zas_gimbal_status_msg_get_cmd_tilt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field pan from zas_gimbal_status_msg message
 *
 * @return [deg]  Current pan status in deg 
 */
static inline float mavlink_msg_zas_gimbal_status_msg_get_pan(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field tilt from zas_gimbal_status_msg message
 *
 * @return [deg]  Current tilt status in deg 
 */
static inline float mavlink_msg_zas_gimbal_status_msg_get_tilt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field elevation from zas_gimbal_status_msg message
 *
 * @return [deg]  Current elevation in global frame status in deg 
 */
static inline float mavlink_msg_zas_gimbal_status_msg_get_elevation(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field azimuth from zas_gimbal_status_msg message
 *
 * @return [deg]  Current azimuth in global frame status in deg 
 */
static inline float mavlink_msg_zas_gimbal_status_msg_get_azimuth(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field Zoom from zas_gimbal_status_msg message
 *
 * @return [x]  Zoom value 
 */
static inline uint8_t mavlink_msg_zas_gimbal_status_msg_get_Zoom(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field wb_mode from zas_gimbal_status_msg message
 *
 * @return [x]  white hot/black hot 
 */
static inline uint8_t mavlink_msg_zas_gimbal_status_msg_get_wb_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field mode from zas_gimbal_status_msg message
 *
 * @return   Gimbal mode for ZAS servo gimbal 
 */
static inline uint8_t mavlink_msg_zas_gimbal_status_msg_get_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field cam_type from zas_gimbal_status_msg message
 *
 * @return [x]  Camera type value 
 */
static inline uint8_t mavlink_msg_zas_gimbal_status_msg_get_cam_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  31);
}

/**
 * @brief Get field trk_status from zas_gimbal_status_msg message
 *
 * @return [uint8]  Tracker status 
 */
static inline uint8_t mavlink_msg_zas_gimbal_status_msg_get_trk_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Decode a zas_gimbal_status_msg message into a struct
 *
 * @param msg The message to decode
 * @param zas_gimbal_status_msg C-struct to decode the message contents into
 */
static inline void mavlink_msg_zas_gimbal_status_msg_decode(const mavlink_message_t* msg, mavlink_zas_gimbal_status_msg_t* zas_gimbal_status_msg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    zas_gimbal_status_msg->cmd_pan = mavlink_msg_zas_gimbal_status_msg_get_cmd_pan(msg);
    zas_gimbal_status_msg->cmd_tilt = mavlink_msg_zas_gimbal_status_msg_get_cmd_tilt(msg);
    zas_gimbal_status_msg->pan = mavlink_msg_zas_gimbal_status_msg_get_pan(msg);
    zas_gimbal_status_msg->tilt = mavlink_msg_zas_gimbal_status_msg_get_tilt(msg);
    zas_gimbal_status_msg->elevation = mavlink_msg_zas_gimbal_status_msg_get_elevation(msg);
    zas_gimbal_status_msg->azimuth = mavlink_msg_zas_gimbal_status_msg_get_azimuth(msg);
    zas_gimbal_status_msg->trk_x = mavlink_msg_zas_gimbal_status_msg_get_trk_x(msg);
    zas_gimbal_status_msg->trk_y = mavlink_msg_zas_gimbal_status_msg_get_trk_y(msg);
    zas_gimbal_status_msg->Zoom = mavlink_msg_zas_gimbal_status_msg_get_Zoom(msg);
    zas_gimbal_status_msg->wb_mode = mavlink_msg_zas_gimbal_status_msg_get_wb_mode(msg);
    zas_gimbal_status_msg->mode = mavlink_msg_zas_gimbal_status_msg_get_mode(msg);
    zas_gimbal_status_msg->cam_type = mavlink_msg_zas_gimbal_status_msg_get_cam_type(msg);
    zas_gimbal_status_msg->trk_status = mavlink_msg_zas_gimbal_status_msg_get_trk_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_LEN? msg->len : MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_LEN;
        memset(zas_gimbal_status_msg, 0, MAVLINK_MSG_ID_ZAS_GIMBAL_STATUS_MSG_LEN);
    memcpy(zas_gimbal_status_msg, _MAV_PAYLOAD(msg), len);
#endif
}
