/*
 * Author       : Akash L Patil<akash@zmotion.in>
 * Description  : To control IR camera functionality through serial port
 * Date         : 10-04-2019
 */

#define DEVOM_SYNC_BYTE        0xAA


#define AP_SERIALMANAGER_DEVO_TELEM_BAUD        57600
#define AP_SERIALMANAGER_DEVO_BUFSIZE_RX        64
#define AP_SERIALMANAGER_DEVO_BUFSIZE_TX        64

#include "AP_IRCamera_Control.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <GCS_MAVLink/GCS.h>

int IR_ZOOM[5] = {0,0,1,1,2};

extern const AP_HAL::HAL& hal;


//constructor
AP_IRCamera_Control::AP_IRCamera_Control()
{
    if (_singleton != nullptr) {
    #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            AP_HAL::panic("Mount must be singleton");
    #endif
            return;
    }
    _singleton = this;
}

void AP_IRCamera_Control::init()
{
    const AP_SerialManager& serial_manager = AP::serialmanager();

    // check for DEVO_DPort
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_IRCamera, 0))) {
        //_port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        // initialise uart
        //_port->begin(AP_SERIALMANAGER_DEVO_TELEM_BAUD, AP_SERIALMANAGER_DEVO_BUFSIZE_RX, AP_SERIALMANAGER_DEVO_BUFSIZE_TX);

        hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_IRCamera_Control::tick, void));
    }
    zoom_data = 1;
}

void AP_IRCamera_Control::send_camera_settings_status(mavlink_channel_t chan)
{
    mavlink_msg_camera_settings_send(chan,0,2,zoom_data,1.0f);
}

uint32_t AP_IRCamera_Control::gpsDdToDmsFormat(float ddm)
{
    int32_t deg = (int32_t)ddm;
    float mm = (ddm - deg) * 60.0f;

    mm = ((float)deg * 100.0f + mm) /100.0;

    if ((mm < -180.0f) || (mm > 180.0f)) {
        mm = 0.0f;
    }

    return mm * 1.0e7;
}


/*
  send_frames - sends updates down telemetry link
  should be called at 1hz
*/

#define DEVO_SPEED_FACTOR 0.0194384f

void AP_IRCamera_Control::set_zoom(uint8_t camera_id,uint8_t zoom_value)
{

    // return immediately if not initialised
    if (_port == nullptr) {
        return;
    }



    if(zoom_value == 51) //To set Camera Polarity as Black
    {
        command_packet.Header           = HVT640_HEADER;
        command_packet.Pack_Seq_Num_1   = HVT640_PKT_SEQ_NUM_1;
        command_packet.Pack_Seq_Num_2   = HVT640_PKT_SEQ_NUM_2;
        command_packet.Device_Id        = HVT640_DEVICE_ID;
        command_packet.Device_Num       = HVT640_DEVICE_NUM;
        command_packet.Length           = HVT640_LENGTH;
        command_packet.Command_Type     = HVT640_COMMAND_TYPE;
        command_packet.Command_Code_1   = HVT640_COMMAND_CODE_1;
        command_packet.Command_Code_2   = HVT640_COMMAND_CODE_2_POLARITY;
        command_packet.Zoom1            = 0x00;
        command_packet.Zoom2            = 0x00;
        command_packet.Zoom3            = 0x00;
        command_packet.Zoom4            = 0x01;
        command_packet.Footer_1         = HVT640_FOOTER_1;
        command_packet.Footer_2         = HVT640_FOOTER_2;

        command_packet.crc              = (command_packet.Device_Id + command_packet.Device_Num  + command_packet.Length + command_packet.Command_Type + command_packet.Command_Code_1 + command_packet.Command_Code_2 + command_packet.Zoom1 + command_packet.Zoom2 + command_packet.Zoom3 + command_packet.Zoom4) % 256;
    }
    else if(zoom_value == 52) //To set Camera Polarity as White
    {
        command_packet.Header           = HVT640_HEADER;
        command_packet.Pack_Seq_Num_1   = HVT640_PKT_SEQ_NUM_1;
        command_packet.Pack_Seq_Num_2   = HVT640_PKT_SEQ_NUM_2;
        command_packet.Device_Id        = HVT640_DEVICE_ID;
        command_packet.Device_Num       = HVT640_DEVICE_NUM;
        command_packet.Length           = HVT640_LENGTH;
        command_packet.Command_Type     = HVT640_COMMAND_TYPE;
        command_packet.Command_Code_1   = HVT640_COMMAND_CODE_1;
        command_packet.Command_Code_2   = HVT640_COMMAND_CODE_2_POLARITY;
        command_packet.Zoom1            = 0x00;
        command_packet.Zoom2            = 0x00;
        command_packet.Zoom3            = 0x00;
        command_packet.Zoom4            = 0x00;
        command_packet.Footer_1         = HVT640_FOOTER_1;
        command_packet.Footer_2         = HVT640_FOOTER_2;

        command_packet.crc              = (command_packet.Device_Id + command_packet.Device_Num  + command_packet.Length + command_packet.Command_Type + command_packet.Command_Code_1 + command_packet.Command_Code_2 + command_packet.Zoom1 + command_packet.Zoom2 + command_packet.Zoom3 + command_packet.Zoom4) % 256;

    }
    else {//To set camera zoom
        if(camera_id == HVT640_CAMERA_ID)
        {
            max_zoom = HVT640_MAX_ZOOM;
        }
        else
        {
            max_zoom = 1;
            return;
        }

        if(zoom_value > max_zoom)
            zoom_value = max_zoom;
		

        zoom_data = zoom_value;//testing purpose

        command_packet.Header           = HVT640_HEADER;
        command_packet.Pack_Seq_Num_1   = HVT640_PKT_SEQ_NUM_1;
        command_packet.Pack_Seq_Num_2   = HVT640_PKT_SEQ_NUM_2;
        command_packet.Device_Id        = HVT640_DEVICE_ID;
        command_packet.Device_Num       = HVT640_DEVICE_NUM;
        command_packet.Length           = HVT640_LENGTH;
        command_packet.Command_Type     = HVT640_COMMAND_TYPE;
        command_packet.Command_Code_1   = HVT640_COMMAND_CODE_1;
        command_packet.Command_Code_2   = HVT640_COMMAND_CODE_2_ZOOM;
        command_packet.Zoom1            = 0x00;
        command_packet.Zoom2            = 0x00;
        command_packet.Zoom3            = 0x00;
        command_packet.Zoom4            = IR_ZOOM[zoom_value];
        command_packet.Footer_1         = HVT640_FOOTER_1;
        command_packet.Footer_2         = HVT640_FOOTER_2;

        command_packet.crc              = (command_packet.Device_Id + command_packet.Device_Num  + command_packet.Length + command_packet.Command_Type + command_packet.Command_Code_1 + command_packet.Command_Code_2 + command_packet.Zoom1 + command_packet.Zoom2 + command_packet.Zoom3 + command_packet.Zoom4) % 256;
    }

    //gcs().send_text(MAV_SEVERITY_CRITICAL, "Zoom %f", zoom_value);

    uint8_t* buf = (uint8_t*)&command_packet;
    for(int i=0; i< sizeof(command_packet) ;i++)
    {
    _port->write(buf[i]);
    }
}

void AP_IRCamera_Control::tick(void)
{
    uint32_t now = AP_HAL::millis();

    if (now - _last_frame_ms > 1000) {
        _last_frame_ms = now;
    }
}

// singleton instance
AP_IRCamera_Control *AP_IRCamera_Control::_singleton;

namespace AP {

AP_IRCamera_Control *ircamera_control()
{
    return AP_IRCamera_Control::get_singleton();
}

};
