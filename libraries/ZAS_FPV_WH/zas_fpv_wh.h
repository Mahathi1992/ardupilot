/*
 * Author       : Mahathi
 * Description  : mx28 gimbal Control functionality through serial port
 * Date         : 28-11-2024
 */

 #pragma once
 #include <AP_HAL/AP_HAL.h>
 #include <AP_SerialManager/AP_SerialManager.h>
 #include <GCS_MAVLink/GCS_MAVLink.h>
 

 #define gpio_pin_wh 55
 // define ZAS fpv warhead serial communication parameters
 #define ZAS_warhead_RX_PACKET_SIZE 6
 #define ZAS_warhead_TX_PACKET_SIZE 5
 #define warhead_PKT_FOOTER 0x0A 

 #define warhead_ON_cmd 0x3F
 #define warhead_OFF_cmd 0x0A
 #define fire_cmd_from_ctrl 0xE2
 #define NO_fire_cmd_from_ctrl 0x7C 

 #define power_ON_status 0x9A
 #define power_OFF_status 0xD4
 
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
     void write_zas_usr_cmd_fpv_wh();
 
     //read incoming data from ZAS warhead
     void read_incoming_zas_fpv_wh();
 
     void handle_usr_cmd_fpv_wh(mavlink_channel_t chan, const mavlink_message_t &msg);
 
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

     bool power_status_flag_zas = false;
     bool abort_status_flag_zas = false;
     bool arm_status_flag_zas = false;
     bool fire_status_flag_zas = false;

     bool arm_sent_flag = false;
     bool disarm_sent_flag = false;

     uint8_t power_status_prev = 0xA5;

     uint8_t response_power_byte_2;
     uint8_t response_power_byte_3;
     uint8_t response_power_byte_4;

     uint8_t response_arm_byte_2;
     uint8_t response_arm_byte_3;
     uint8_t response_arm_byte_4;

     uint8_t response_fire_byte_2;
     uint8_t response_fire_byte_3;
     uint8_t response_fire_byte_4;

     uint8_t response_abort_byte_2;
     uint8_t response_abort_byte_3;
     uint8_t response_abort_byte_4;
 
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
 