/*
 * Author       : Mahathi
 * Description  : mx28 gimbal Control functionality through serial port
 * Date         : 28-11-2024
 */

#pragma once

// #include <stdio.h>
// // #include <iostream>
// #include <stdexcept>
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

// define ZAS gimbal serial communication parameters
#define ZAS_COMM_RX_PACKET_SIZE 38
#define ZAS_COMM_TX_PACKET_SIZE 20
#define TRACK_COMM_PKT_SIZE 29  // for getting JetsonNano tracker data. Added by Mahathi
#define TRACK_HDR1 0x51
#define TRACK_HDR2 0xAC

#define COMM_HDR1 0xFF
#define COMM_HDR2 0xF0
#define GCS_HDR1 100
#define GCS_HDR2 100
#define GCS_HDR3 240


class AP_MX28_Control {
public:
    //constructor
    AP_MX28_Control();

    /* Do not allow copies */
    AP_MX28_Control(const AP_MX28_Control &other) = delete;
    AP_MX28_Control &operator=(const AP_MX28_Control&) = delete;
	// get singleton instance
    static AP_MX28_Control *get_singleton() {
           return _singleton;
       }

    void init();

    // write usr commands from GCS onto UART port
    void write_zas_usr_cmd();

    //read incoming data from ZAS GCB
    void read_incoming_zas();

    void handle_usr_cmd(mavlink_channel_t chan, const mavlink_message_t &msg);

    void handle_zas_mount_control_zoom(const mavlink_message_t &msg);
    void handle_zas_mount_control(const mavlink_message_t &msg);
    void handle_zas_mount_control_pilot(const mavlink_message_t &msg);
    void handle_zas_mount_control_geopoint(const mavlink_message_t &msg);

    // send ZAS gimbal status
    void send_zas_gimbal_status(mavlink_channel_t chan);

public:

    // tick - main call to send updates to transmitter
    void tick(void);

    AP_HAL::UARTDriver *_port;              // UART used to send data to receiver
    uint32_t _last_frame_ms;
    // mavlink_channel_t chan;
    bool isChannelSet;
    bool isSerialInit;

    // Serial Protocol Variables
    uint8_t _checksum;
    uint8_t _step;
    uint8_t _command_id;
    uint8_t _payload_length;
    uint8_t _payload_counter;

    // struct definition for sending data from ZAS gimbal to GCS
    struct PACKED zas_gimbal_status_t
    {
        int16_t trk_x;
        int16_t trk_y;
        float cmd_pan;
        float cmd_tilt;
        float pan;
        float tilt;
        float elevation;
        float azimuth;
        uint8_t Zoom;
        uint8_t wb_mode;
        uint8_t mode;
        uint8_t cam_type;
        uint8_t trk_status;
    };

    // struct definition for sending data from AP to ZAS gimbal 
    struct PACKED zas_gimbal_cmd_t
    {
        float yawrate = 0;
        float pitchrate = 0;
        float rollrate = 0;
        float des_pan = 0;
        float des_tilt = 0;
        int16_t Zoom = 0;
        int16_t ZoomRate = 0;
        int32_t target_lat = 0;
        int32_t target_lng = 0;
        int32_t target_amsl = 0;
        uint8_t calib_stat = 0;
        int16_t trk_x = 0;
        int16_t trk_y = 0;
        uint8_t trk_cmd = 0;
        int8_t offset_x = 0;
        int8_t offset_y = 0;
        int16_t offset_box = 0;
        uint8_t submode = 0;
        uint8_t mode = 0;
    };

    // get zas gimbal data from Alexmos.cpp
    zas_gimbal_status_t status = {};

    // get GCS commands for ZAS gimbal to be used in Alexmos.cpp
    zas_gimbal_cmd_t usr_cmd = {};

protected:
    static AP_MX28_Control *_singleton;
    
    void get_uart_data();

};

namespace AP {
AP_MX28_Control *mx28_control();
};
