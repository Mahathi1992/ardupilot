/*
 * Author       : Akash L Patil<akash@zmotion.in>
 * Description  : To control EO camera functionality through serial port
 * Date         : 10-04-2019
 */

#define DEVOM_SYNC_BYTE        0xAA


#define AP_SERIALMANAGER_DEVO_TELEM_BAUD        57600
#define AP_SERIALMANAGER_DEVO_BUFSIZE_RX        64
#define AP_SERIALMANAGER_DEVO_BUFSIZE_TX        64

#include "AP_EOCamera_Control.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

unsigned int VID_EXSDI_ZOOM_LEVEL[10] = {VID_EXSDI_ZOOM_LEVEL_1,VID_EXSDI_ZOOM_LEVEL_2,VID_EXSDI_ZOOM_LEVEL_3,VID_EXSDI_ZOOM_LEVEL_4,VID_EXSDI_ZOOM_LEVEL_5,VID_EXSDI_ZOOM_LEVEL_6,VID_EXSDI_ZOOM_LEVEL_7,VID_EXSDI_ZOOM_LEVEL_8,VID_EXSDI_ZOOM_LEVEL_9,VID_EXSDI_ZOOM_LEVEL_10};
unsigned int HD_8210G_ZOOM_LEVEL[10] = {HD_8210G_ZOOM_LEVEL_1,HD_8210G_ZOOM_LEVEL_2,HD_8210G_ZOOM_LEVEL_3,HD_8210G_ZOOM_LEVEL_4,HD_8210G_ZOOM_LEVEL_5,HD_8210G_ZOOM_LEVEL_6,HD_8210G_ZOOM_LEVEL_7,HD_8210G_ZOOM_LEVEL_8,HD_8210G_ZOOM_LEVEL_9,HD_8210G_ZOOM_LEVEL_10};

//constructor
AP_EOCamera_Control::AP_EOCamera_Control()
{
    if (_singleton != nullptr) {
    #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            AP_HAL::panic("Mount must be singleton");
    #endif
            return;
    }

    prevoius_camera_id = 0;
        _singleton = this;
}

void AP_EOCamera_Control::init()
{
    const AP_SerialManager& serial_manager = AP::serialmanager();

    // check for DEVO_DPort
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_EOCamera, 0))) {
       // _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        // initialise uart
       // _port->begin(AP_SERIALMANAGER_DEVO_TELEM_BAUD, AP_SERIALMANAGER_DEVO_BUFSIZE_RX, AP_SERIALMANAGER_DEVO_BUFSIZE_TX);

        hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_EOCamera_Control::tick, void));
    }
    zoom_data = 1;
}

void AP_EOCamera_Control::send_camera_settings_status(mavlink_channel_t chan)
{
    mavlink_msg_camera_settings_send(chan,0,1,zoom_data,1.0f);
}

uint32_t AP_EOCamera_Control::gpsDdToDmsFormat(float ddm)
{
    int32_t deg = (int32_t)ddm;
    float mm = (ddm - deg) * 60.0f;

    mm = ((float)deg * 100.0f + mm) /100.0;

    if ((mm < -180.0f) || (mm > 180.0f)) {
        mm = 0.0f;
    }

    return mm * 1.0e7;
}


#define DEVO_SPEED_FACTOR 0.0194384f

void AP_EOCamera_Control::set_zoom(uint8_t camera_id,uint8_t zoom_value)
{
    // return immediately if not initialised
    if (_port == nullptr) {
        return;
    }


    if(prevoius_camera_id != camera_id){
        if(camera_id == VISCA_ID_VID_EXSDI){
            max_zoom = VISCA_MAX_ZOOM_VID_EXSDI;
            for(int i = 0 ; i < VISCA_MAX_ZOOM_VID_EXSDI ; i++)
                ZOOM_LEVEL[i] = VID_EXSDI_ZOOM_LEVEL[i];
        }
        else if(camera_id == VISCA_ID_HD_8210G)
        {
            max_zoom = VISCA_MAX_ZOOM_HD_8210G;
            for(int i = 0 ; i < VISCA_MAX_ZOOM_HD_8210G ; i++)
                ZOOM_LEVEL[i] = HD_8210G_ZOOM_LEVEL[i];
        }
        else
        {
            max_zoom = 1;
            for(int i = 0 ; i < VISCA_MAX_ZOOM_VID_EXSDI ; i++)
                ZOOM_LEVEL[i] = VID_EXSDI_ZOOM_LEVEL[i];
            return;
        }
    prevoius_camera_id = camera_id;
    }
    if(zoom_value > max_zoom)
        zoom_value = max_zoom - 1;

    if(zoom_value == 0)zoom_value = 1;

    zoom_data = zoom_value;//testing purpose
    command_packet.Address = VISCA_ADDRESS;
    command_packet.Command = VISCA_COMMAND;
    command_packet.Camera  = VISCA_CAMERA_VLAUE;
    command_packet.Zoom_Id = VISCA_ZOOM_VALUE;
    command_packet.Zoom1   = (ZOOM_LEVEL[zoom_value -1]>> 12) & 0x0F;
    command_packet.Zoom2   = (ZOOM_LEVEL[zoom_value -1]>> 8) & 0x0F;
    command_packet.Zoom3   = (ZOOM_LEVEL[zoom_value -1]>> 4) & 0x0F;
    command_packet.Zoom4   = (ZOOM_LEVEL[zoom_value -1]) & 0x0F;
    command_packet.Terminator = VISCA_TERMINATOR;

    //gcs().send_text(MAV_SEVERITY_CRITICAL, "Zoom AP %d", HD_8210G_ZOOM_LEVEL[zoom_value -1]);
    uint8_t* buf = (uint8_t*)&command_packet;
    for(int i=0; i< 9 ;i++)
    {
    _port->write(buf[i]);
    }
}

void AP_EOCamera_Control::tick(void)
{
    uint32_t now = AP_HAL::millis();

    if (now - _last_frame_ms > 1000) {
        _last_frame_ms = now;
       // set_zoom(1,2);
    }
}

// singleton instance
AP_EOCamera_Control *AP_EOCamera_Control::_singleton;

namespace AP {

AP_EOCamera_Control *eocamera_control()
{
    return AP_EOCamera_Control::get_singleton();
}

};
