/*
 * Author       : Akash L Patil<akash@zmotion.in>
 * Description  : To control EO camera functionality through serial port
 * Date         : 10-04-2019
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#define VID_EXSDI_ZOOM_LEVEL_1      0x0000
#define VID_EXSDI_ZOOM_LEVEL_2      0x1816
#define VID_EXSDI_ZOOM_LEVEL_3      0x240B
#define VID_EXSDI_ZOOM_LEVEL_4      0x2BC7
#define VID_EXSDI_ZOOM_LEVEL_5      0x31AB
#define VID_EXSDI_ZOOM_LEVEL_6      0x363D
#define VID_EXSDI_ZOOM_LEVEL_7      0x39B6
#define VID_EXSDI_ZOOM_LEVEL_8      0x3C65
#define VID_EXSDI_ZOOM_LEVEL_9      0x3E81
#define VID_EXSDI_ZOOM_LEVEL_10     0x4000

#define HD_8210G_ZOOM_LEVEL_1       0x0000
#define HD_8210G_ZOOM_LEVEL_2       0x1A00
#define HD_8210G_ZOOM_LEVEL_3       0x2580
#define HD_8210G_ZOOM_LEVEL_4       0x2C80
#define HD_8210G_ZOOM_LEVEL_5       0x3180
#define HD_8210G_ZOOM_LEVEL_6       0x3580
#define HD_8210G_ZOOM_LEVEL_7       0x3900
#define HD_8210G_ZOOM_LEVEL_8       0x3C00
#define HD_8210G_ZOOM_LEVEL_9       0x3E40
#define HD_8210G_ZOOM_LEVEL_10      0x4000


#define VISCA_ADDRESS                   0x81
#define VISCA_COMMAND                   0x01
#define VISCA_INQUIRY                   0x09
#define VISCA_TERMINATOR                0xFF
#define VISCA_CAMERA_VLAUE              0x04
#define VISCA_ZOOM_VALUE                0x47

#define VISCA_MODEL_VID_EXSDI           0x0466
#define VISCA_ID_VID_EXSDI              0x01
#define VISCA_ID_HD_8210G               0x03
#define VISCA_MAX_ZOOM_VID_EXSDI        10
#define VISCA_MAX_ZOOM_HD_8210G         10

class AP_EOCamera_Control {
public:
    //constructor
    AP_EOCamera_Control();

    /* Do not allow copies */
    AP_EOCamera_Control(const AP_EOCamera_Control &other) = delete;
    AP_EOCamera_Control &operator=(const AP_EOCamera_Control&) = delete;

    // get singleton instance
    static AP_EOCamera_Control *get_singleton() {
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
    uint8_t prevoius_camera_id;

    void send_camera_settings_status(mavlink_channel_t chan);

    // tick - main call to send updates to transmitter
    void tick(void);

    // send_frames - sends updates down telemetry link
    void set_zoom(uint8_t camera_id,uint8_t zoom_value);

    uint8_t _control_mode;

    AP_HAL::UARTDriver *_port;              // UART used to send data to receiver
    uint32_t _last_frame_ms;

    unsigned int ZOOM_LEVEL[10];

    //added by Akash for Serial Testing purpose
    typedef struct PACKED {
        uint8_t Address;
        uint8_t Command;
        uint8_t Camera;
        uint8_t Zoom_Id;
        uint8_t Zoom1;
        uint8_t Zoom2;
        uint8_t Zoom3;
        uint8_t Zoom4;
        uint8_t Terminator;
    }visca_protocol;

    visca_protocol command_packet;
protected:
    static AP_EOCamera_Control *_singleton;

};

namespace AP {
AP_EOCamera_Control *eocamera_control();
};
