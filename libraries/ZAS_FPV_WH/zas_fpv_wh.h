/*
 * Author       : Mahathi
 * Description  : mx28 gimbal Control functionality through serial port
 * Date         : 28-11-2024
 */

 #pragma once
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
 
 
 class ZAS_FPV_WH {
 public:
     //constructor
     ZAS_FPV_WH();
 
     /* Do not allow copies */
     ZAS_FPV_WH(const ZAS_FPV_WH &other) = delete;
     ZAS_FPV_WH &operator=(const ZAS_FPV_WH&) = delete;
     // get singleton instance
     static ZAS_FPV_WH *get_singleton() {
            return _singleton;
        }
 
     void init();
 
     // write usr commands from GCS onto UART port
     void write_zas_usr_cmd();
 
     //read incoming data from ZAS warhead
     void read_incoming_zas();
 
     void handle_usr_cmd(mavlink_channel_t chan, const mavlink_message_t &msg);
 
     void handle_zas_warhead_command(const mavlink_message_t &msg);
 
     // send ZAS warhead status
     void send_zas_warhead_status(mavlink_channel_t chan);
 
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
     struct PACKED zas_fpv_status_t
     {
         uint8_t target_system;
         uint8_t power_status;
         uint8_t arm_status;
         uint8_t disarm_status;
     };
 
     // struct definition for sending data from AP to ZAS gimbal 
     struct PACKED zas_fpv_cmd_t
     {
         uint8_t wh_state_cmd = 0;
         uint8_t fire_cmd = 0;
     };
 
     // get zas gimbal data from Alexmos.cpp
     zas_fpv_status_t status = {};
 
     // get GCS commands for ZAS gimbal to be used in Alexmos.cpp
     zas_fpv_cmd_t usr_cmd = {};
 
 protected:
     static ZAS_FPV_WH *_singleton;
     
     void get_uart_data();
 
 };
 
 namespace AP {
 ZAS_FPV_WH *zas_fpv_wh();
 };
 