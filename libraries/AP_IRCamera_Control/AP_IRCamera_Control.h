/*
 * Author       : Akash L Patil<akash@zmotion.in>
 * Description  : To control IR camera functionality through serial port
 * Date         : 10-04-2019
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#define HVT640_HEADER                  0xE0
#define HVT640_PKT_SEQ_NUM_1           0x00
#define HVT640_PKT_SEQ_NUM_2           0x01
#define HVT640_DEVICE_ID               0x3E
#define HVT640_DEVICE_NUM              0xFF
#define HVT640_LENGTH                  0x07
#define HVT640_COMMAND_TYPE            0x57
#define HVT640_COMMAND_CODE_1          0x50
#define HVT640_COMMAND_CODE_2_ZOOM          0x86
#define HVT640_FOOTER_1                0xFF
#define HVT640_FOOTER_2                0xFE
#define HVT640_COMMAND_CODE_2_POLARITY          0x52

#define HVT640_CAMERA_ID               0x01
#define HVT640_MAX_ZOOM                0x04

class AP_IRCamera_Control {
public:
    //constructor
    AP_IRCamera_Control();

    /* Do not allow copies */
    AP_IRCamera_Control(const AP_IRCamera_Control &other) = delete;
    AP_IRCamera_Control &operator=(const AP_IRCamera_Control&) = delete;
	// get singleton instance
    static AP_IRCamera_Control *get_singleton() {
           return _singleton;
       }

    void init();

    // update flight control mode. The control mode is vehicle type specific
    void update_control_mode(uint8_t mode)
    {
        _control_mode = mode;
    }


public:

    uint32_t gpsDdToDmsFormat(float ddm);
    uint8_t max_zoom;
    uint8_t zoom_data;

    void send_camera_settings_status(mavlink_channel_t chan);

    // tick - main call to send updates to transmitter
    void tick(void);

    // send_frames - sends updates down telemetry link
    void set_zoom(uint8_t camera_id,uint8_t zoom_value);

    uint8_t _control_mode;

    AP_HAL::UARTDriver *_port;              // UART used to send data to receiver
    uint32_t _last_frame_ms;

    typedef struct PACKED {
        uint8_t Header;
        uint8_t Pack_Seq_Num_1;
        uint8_t Pack_Seq_Num_2;
        uint8_t Device_Id;
        uint8_t Device_Num;
        uint8_t Length;
        uint8_t Command_Type;
        uint8_t Command_Code_1;
        uint8_t Command_Code_2;
        uint8_t Zoom1;
        uint8_t Zoom2;
        uint8_t Zoom3;
        uint8_t Zoom4;
        uint8_t crc;
        uint8_t Footer_1;
        uint8_t Footer_2;
    }tonbo_protocol;

    tonbo_protocol command_packet;

protected:
    static AP_IRCamera_Control *_singleton;

};

namespace AP {
AP_IRCamera_Control *ircamera_control();
};
