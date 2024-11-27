#pragma once
// MESSAGE POWER_12V PACKING

#define MAVLINK_MSG_ID_POWER_12V 272


typedef struct __mavlink_power_12v_t {
 float Roll; /*<  Roll value*/
 float Pitch; /*<  Pitch Value*/
 float Yaw; /*<  Yaw value*/
 uint8_t Power12_On_Off; /*<  Power ON and OFF should be 1 or 0*/
 uint8_t Mount_Mode; /*<  Mount Mode Value*/
} mavlink_power_12v_t;

#define MAVLINK_MSG_ID_POWER_12V_LEN 14
#define MAVLINK_MSG_ID_POWER_12V_MIN_LEN 14
#define MAVLINK_MSG_ID_272_LEN 14
#define MAVLINK_MSG_ID_272_MIN_LEN 14

#define MAVLINK_MSG_ID_POWER_12V_CRC 247
#define MAVLINK_MSG_ID_272_CRC 247



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_POWER_12V { \
    272, \
    "POWER_12V", \
    5, \
    {  { "Power12_On_Off", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_power_12v_t, Power12_On_Off) }, \
         { "Roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_power_12v_t, Roll) }, \
         { "Pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_power_12v_t, Pitch) }, \
         { "Yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_power_12v_t, Yaw) }, \
         { "Mount_Mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_power_12v_t, Mount_Mode) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_POWER_12V { \
    "POWER_12V", \
    5, \
    {  { "Power12_On_Off", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_power_12v_t, Power12_On_Off) }, \
         { "Roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_power_12v_t, Roll) }, \
         { "Pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_power_12v_t, Pitch) }, \
         { "Yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_power_12v_t, Yaw) }, \
         { "Mount_Mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_power_12v_t, Mount_Mode) }, \
         } \
}
#endif

/**
 * @brief Pack a power_12v message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param Power12_On_Off  Power ON and OFF should be 1 or 0
 * @param Roll  Roll value
 * @param Pitch  Pitch Value
 * @param Yaw  Yaw value
 * @param Mount_Mode  Mount Mode Value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_power_12v_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t Power12_On_Off, float Roll, float Pitch, float Yaw, uint8_t Mount_Mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_POWER_12V_LEN];
    _mav_put_float(buf, 0, Roll);
    _mav_put_float(buf, 4, Pitch);
    _mav_put_float(buf, 8, Yaw);
    _mav_put_uint8_t(buf, 12, Power12_On_Off);
    _mav_put_uint8_t(buf, 13, Mount_Mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_POWER_12V_LEN);
#else
    mavlink_power_12v_t packet;
    packet.Roll = Roll;
    packet.Pitch = Pitch;
    packet.Yaw = Yaw;
    packet.Power12_On_Off = Power12_On_Off;
    packet.Mount_Mode = Mount_Mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_POWER_12V_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_POWER_12V;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_POWER_12V_MIN_LEN, MAVLINK_MSG_ID_POWER_12V_LEN, MAVLINK_MSG_ID_POWER_12V_CRC);
}

/**
 * @brief Pack a power_12v message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param Power12_On_Off  Power ON and OFF should be 1 or 0
 * @param Roll  Roll value
 * @param Pitch  Pitch Value
 * @param Yaw  Yaw value
 * @param Mount_Mode  Mount Mode Value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_power_12v_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t Power12_On_Off,float Roll,float Pitch,float Yaw,uint8_t Mount_Mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_POWER_12V_LEN];
    _mav_put_float(buf, 0, Roll);
    _mav_put_float(buf, 4, Pitch);
    _mav_put_float(buf, 8, Yaw);
    _mav_put_uint8_t(buf, 12, Power12_On_Off);
    _mav_put_uint8_t(buf, 13, Mount_Mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_POWER_12V_LEN);
#else
    mavlink_power_12v_t packet;
    packet.Roll = Roll;
    packet.Pitch = Pitch;
    packet.Yaw = Yaw;
    packet.Power12_On_Off = Power12_On_Off;
    packet.Mount_Mode = Mount_Mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_POWER_12V_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_POWER_12V;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_POWER_12V_MIN_LEN, MAVLINK_MSG_ID_POWER_12V_LEN, MAVLINK_MSG_ID_POWER_12V_CRC);
}

/**
 * @brief Encode a power_12v struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param power_12v C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_power_12v_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_power_12v_t* power_12v)
{
    return mavlink_msg_power_12v_pack(system_id, component_id, msg, power_12v->Power12_On_Off, power_12v->Roll, power_12v->Pitch, power_12v->Yaw, power_12v->Mount_Mode);
}

/**
 * @brief Encode a power_12v struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param power_12v C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_power_12v_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_power_12v_t* power_12v)
{
    return mavlink_msg_power_12v_pack_chan(system_id, component_id, chan, msg, power_12v->Power12_On_Off, power_12v->Roll, power_12v->Pitch, power_12v->Yaw, power_12v->Mount_Mode);
}

/**
 * @brief Send a power_12v message
 * @param chan MAVLink channel to send the message
 *
 * @param Power12_On_Off  Power ON and OFF should be 1 or 0
 * @param Roll  Roll value
 * @param Pitch  Pitch Value
 * @param Yaw  Yaw value
 * @param Mount_Mode  Mount Mode Value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_power_12v_send(mavlink_channel_t chan, uint8_t Power12_On_Off, float Roll, float Pitch, float Yaw, uint8_t Mount_Mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_POWER_12V_LEN];
    _mav_put_float(buf, 0, Roll);
    _mav_put_float(buf, 4, Pitch);
    _mav_put_float(buf, 8, Yaw);
    _mav_put_uint8_t(buf, 12, Power12_On_Off);
    _mav_put_uint8_t(buf, 13, Mount_Mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POWER_12V, buf, MAVLINK_MSG_ID_POWER_12V_MIN_LEN, MAVLINK_MSG_ID_POWER_12V_LEN, MAVLINK_MSG_ID_POWER_12V_CRC);
#else
    mavlink_power_12v_t packet;
    packet.Roll = Roll;
    packet.Pitch = Pitch;
    packet.Yaw = Yaw;
    packet.Power12_On_Off = Power12_On_Off;
    packet.Mount_Mode = Mount_Mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POWER_12V, (const char *)&packet, MAVLINK_MSG_ID_POWER_12V_MIN_LEN, MAVLINK_MSG_ID_POWER_12V_LEN, MAVLINK_MSG_ID_POWER_12V_CRC);
#endif
}

/**
 * @brief Send a power_12v message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_power_12v_send_struct(mavlink_channel_t chan, const mavlink_power_12v_t* power_12v)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_power_12v_send(chan, power_12v->Power12_On_Off, power_12v->Roll, power_12v->Pitch, power_12v->Yaw, power_12v->Mount_Mode);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POWER_12V, (const char *)power_12v, MAVLINK_MSG_ID_POWER_12V_MIN_LEN, MAVLINK_MSG_ID_POWER_12V_LEN, MAVLINK_MSG_ID_POWER_12V_CRC);
#endif
}

#if MAVLINK_MSG_ID_POWER_12V_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_power_12v_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t Power12_On_Off, float Roll, float Pitch, float Yaw, uint8_t Mount_Mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, Roll);
    _mav_put_float(buf, 4, Pitch);
    _mav_put_float(buf, 8, Yaw);
    _mav_put_uint8_t(buf, 12, Power12_On_Off);
    _mav_put_uint8_t(buf, 13, Mount_Mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POWER_12V, buf, MAVLINK_MSG_ID_POWER_12V_MIN_LEN, MAVLINK_MSG_ID_POWER_12V_LEN, MAVLINK_MSG_ID_POWER_12V_CRC);
#else
    mavlink_power_12v_t *packet = (mavlink_power_12v_t *)msgbuf;
    packet->Roll = Roll;
    packet->Pitch = Pitch;
    packet->Yaw = Yaw;
    packet->Power12_On_Off = Power12_On_Off;
    packet->Mount_Mode = Mount_Mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POWER_12V, (const char *)packet, MAVLINK_MSG_ID_POWER_12V_MIN_LEN, MAVLINK_MSG_ID_POWER_12V_LEN, MAVLINK_MSG_ID_POWER_12V_CRC);
#endif
}
#endif

#endif

// MESSAGE POWER_12V UNPACKING


/**
 * @brief Get field Power12_On_Off from power_12v message
 *
 * @return  Power ON and OFF should be 1 or 0
 */
static inline uint8_t mavlink_msg_power_12v_get_Power12_On_Off(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field Roll from power_12v message
 *
 * @return  Roll value
 */
static inline float mavlink_msg_power_12v_get_Roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field Pitch from power_12v message
 *
 * @return  Pitch Value
 */
static inline float mavlink_msg_power_12v_get_Pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field Yaw from power_12v message
 *
 * @return  Yaw value
 */
static inline float mavlink_msg_power_12v_get_Yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field Mount_Mode from power_12v message
 *
 * @return  Mount Mode Value
 */
static inline uint8_t mavlink_msg_power_12v_get_Mount_Mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Decode a power_12v message into a struct
 *
 * @param msg The message to decode
 * @param power_12v C-struct to decode the message contents into
 */
static inline void mavlink_msg_power_12v_decode(const mavlink_message_t* msg, mavlink_power_12v_t* power_12v)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    power_12v->Roll = mavlink_msg_power_12v_get_Roll(msg);
    power_12v->Pitch = mavlink_msg_power_12v_get_Pitch(msg);
    power_12v->Yaw = mavlink_msg_power_12v_get_Yaw(msg);
    power_12v->Power12_On_Off = mavlink_msg_power_12v_get_Power12_On_Off(msg);
    power_12v->Mount_Mode = mavlink_msg_power_12v_get_Mount_Mode(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_POWER_12V_LEN? msg->len : MAVLINK_MSG_ID_POWER_12V_LEN;
        memset(power_12v, 0, MAVLINK_MSG_ID_POWER_12V_LEN);
    memcpy(power_12v, _MAV_PAYLOAD(msg), len);
#endif
}
