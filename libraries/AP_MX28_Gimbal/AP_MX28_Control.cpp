#include <AP_MX28_Gimbal/AP_MX28_Control.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

//constructor
AP_MX28_Control::AP_MX28_Control() {
    if (_singleton != nullptr) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("mx28_gimbal must be singleton");
#endif
        return;
    }
    isChannelSet = false;
    isSerialInit = false;
    _singleton = this;
}


void AP_MX28_Control::init() {
    //const AP_SerialManager& serial_manager = AP::serialmanager();
    
    _port = AP::serialmanager().get_alexmos2_uart();
    if ((_port != nullptr )) {
        hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_MX28_Control::tick, void));
    }
    if (_port != nullptr){
        _port->printf("AP_MX28_Gimbal INIT.\r\n");
    }
    //_port = nullptr;
}

void AP_MX28_Control::get_uart_data() {

    if (!isSerialInit){
        _port->begin(115200,128,128);
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        isSerialInit = true;
    }
    
    read_incoming_zas(); // This is used with MX28 gimbal no tracking

}

void AP_MX28_Control::handle_usr_cmd(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_ZAS_GIMBAL_ZOOM_CMD:
            handle_zas_mount_control_zoom(msg);
            break;
        case MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD:
            handle_zas_mount_control(msg);
            break;
        case MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB:
            handle_zas_mount_control_pilot(msg);
            break;
        case MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT:
            handle_zas_mount_control_geopoint(msg);
            break;
        default:
            gcs().send_text(MAV_SEVERITY_WARNING, "ZAS MSG TYPE NOT DEFINED");
            break;
    }
}

/// Control the ZAS Gimbal 
void AP_MX28_Control::handle_zas_mount_control_zoom(const mavlink_message_t &msg)
{
    mavlink_zas_gimbal_zoom_cmd_t packet;
    mavlink_msg_zas_gimbal_zoom_cmd_decode(&msg, &packet);

    usr_cmd.offset_box = packet.offset_box;
    usr_cmd.offset_y = packet.offset_y;
    usr_cmd.offset_x = packet.offset_x;
    usr_cmd.trk_cmd = packet.trk_cmd;
    usr_cmd.trk_y = packet.trk_y;
    usr_cmd.trk_x = packet.trk_x;
    usr_cmd.ZoomRate = packet.ZoomRate; //mode=200
    usr_cmd.Zoom = packet.Zoom; //mode=201
    usr_cmd.mode = packet.mode; 

    gcs().send_text(MAV_SEVERITY_INFO, "ZOOM decoded. MODE: %d", packet.Zoom);

    write_zas_usr_cmd();
}

void AP_MX28_Control::handle_zas_mount_control(const mavlink_message_t &msg)
{
    mavlink_zas_gimbal_usr_cmd_t packet;
    mavlink_msg_zas_gimbal_usr_cmd_decode(&msg, &packet);

    usr_cmd.mode = packet.mode;
    usr_cmd.calib_stat = packet.calib_stat;

    write_zas_usr_cmd();
}

void AP_MX28_Control::handle_zas_mount_control_pilot(const mavlink_message_t &msg)
{
    mavlink_zas_gimbal_usr_cmd_pilot_stab_t packet;
    mavlink_msg_zas_gimbal_usr_cmd_pilot_stab_decode(&msg, &packet);

    usr_cmd.mode = packet.mode;
    usr_cmd.submode = packet.submode;
    usr_cmd.calib_stat = packet.calib_stat;
    usr_cmd.des_tilt = packet.tilt;
    usr_cmd.des_pan = packet.pan;
    usr_cmd.rollrate = packet.rollrate;
    usr_cmd.pitchrate = packet.pitchrate;
    usr_cmd.yawrate = packet.yawrate;

    write_zas_usr_cmd();
}

void AP_MX28_Control::handle_zas_mount_control_geopoint(const mavlink_message_t &msg)
{
    mavlink_zas_gimbal_usr_cmd_geopoint_t packet;
    mavlink_msg_zas_gimbal_usr_cmd_geopoint_decode(&msg, &packet);

    usr_cmd.mode = packet.mode;
    usr_cmd.calib_stat = packet.calib_stat;
    usr_cmd.target_lat = packet.target_latitude;
    usr_cmd.target_lng = packet.target_longitude;
    usr_cmd.target_amsl = packet.target_altitude;

    write_zas_usr_cmd();
}

// function to send ZAS gimbal status to GCS

void AP_MX28_Control::send_zas_gimbal_status(mavlink_channel_t chan)
{
    // gcs().send_text(MAV_SEVERITY_WARNING, "zas status send CHECK. PAN ANGLE: %f", status.cmd_pan);

    mavlink_msg_zas_gimbal_status_msg_send(chan, status.trk_x, status.trk_y, status.cmd_pan, status.cmd_tilt, status.pan, status.tilt, status.elevation, status.azimuth, status.Zoom, status.wb_mode, status.mode, status.cam_type, status.trk_status);
}

/*
 * detect and read the header of the incoming message from the ZAS gimbal
 */
void AP_MX28_Control::read_incoming_zas()
{
    uint8_t data;
    int16_t numc;
    uint8_t _zas_buf[ZAS_COMM_RX_PACKET_SIZE] = {0};

    numc = _port->available();
    if (numc < 0 ){
        return;
    }

    // static uint16_t counter = 0;
    // counter++;
    // if (counter > 50)
    // {
    //     counter = 0;
    //     gcs().send_text(MAV_SEVERITY_INFO, "numc: %d", numc);
    // }

    for (int16_t i = 0; i < numc; i++) {        // Process bytes received
        data = _port->read();

        switch (_step) {
            case 0:
                if (COMM_HDR1 == data) {
                    _step = 1;
                    _checksum = 0; //reset checksum accumulator
                    // gcs().send_text(MAV_SEVERITY_INFO, "Case 0, header 1: %d", data);
                }
                break;

            case 1:
                if (COMM_HDR2 == data) {
                    _step++;
                    _checksum = 0; //reset checksum accumulator
                    // gcs().send_text(MAV_SEVERITY_INFO, "Case 1, header 2: %d", data);
                }
                break;

            case 2: // Size of the body of the message
                _payload_length = data;
                // gcs().send_text(MAV_SEVERITY_INFO, "Payload length: %d", data);
                _step++;
                _payload_counter = 3;
                break;

            case 3: // parsing body
                if (_payload_counter <= _payload_length) {
                    _zas_buf[_payload_counter] = data;
                    if(_payload_counter == 4){
                        memcpy(&status.trk_x, &_zas_buf[3],2);
                        // gcs().send_text(MAV_SEVERITY_INFO, "trk_x copied: %d, %d", status.trk_x, _payload_counter);
                    }
                    if(_payload_counter == 6){
                        memcpy(&status.trk_y, &_zas_buf[5],2);
                        // gcs().send_text(MAV_SEVERITY_INFO, "trk_y copied: %d, %d", status.trk_y, _payload_counter);
                    }
                    if(_payload_counter == 10){
                        memcpy(&status.cmd_pan, &_zas_buf[7],4);
                        // gcs().send_text(MAV_SEVERITY_INFO, "cmd_pan copied: %f, %d", status.cmd_pan, _payload_counter);
                    }
                    if(_payload_counter == 14){
                        memcpy(&status.cmd_tilt, &_zas_buf[11],4);
                        // gcs().send_text(MAV_SEVERITY_INFO, "cmd_tilt copied: %f, %d", status.cmd_tilt, _payload_counter);
                    }
                    if(_payload_counter == 18){
                        memcpy(&status.pan, &_zas_buf[15],4);
                        // gcs().send_text(MAV_SEVERITY_INFO, "pan copied: %f, %d", status.pan, _payload_counter);
                    }
                    if(_payload_counter == 22){
                        memcpy(&status.tilt, &_zas_buf[19],4);
                    }
                    if(_payload_counter == 26){
                        memcpy(&status.azimuth, &_zas_buf[23],4);
                        // gcs().send_text(MAV_SEVERITY_INFO, "azimuth copied: %f, %d", status.azimuth, _payload_counter);
                    }
                    if(_payload_counter == 30){
                        memcpy(&status.elevation, &_zas_buf[27],4);
                        // gcs().send_text(MAV_SEVERITY_INFO, "elevation copied: %f, %d", status.elevation, _payload_counter);
                    }
                    if(_payload_counter == 31){
                        memcpy(&status.Zoom, &_zas_buf[31],1);
                        // gcs().send_text(MAV_SEVERITY_INFO, "Zoom copied: %d, %d", status.Zoom, _payload_counter);
                    }
                    if(_payload_counter == 32){
                        memcpy(&status.wb_mode, &_zas_buf[32],1);
                        // gcs().send_text(MAV_SEVERITY_INFO, "mode copied: %d, %d", status.mode, _payload_counter);
                    }
                    if(_payload_counter == 33){
                        memcpy(&status.mode, &_zas_buf[33],1);
                        // gcs().send_text(MAV_SEVERITY_INFO, "mode copied: %d, %d", status.mode, _payload_counter);
                    }
                    if(_payload_counter == 34){
                        memcpy(&status.cam_type, &_zas_buf[34],1);
                        // gcs().send_text(MAV_SEVERITY_INFO, "mode copied: %d, %d", status.mode, _payload_counter);
                    }
                    if(_payload_counter == 35){
                        memcpy(&status.trk_status, &_zas_buf[35],1);
                        // gcs().send_text(MAV_SEVERITY_INFO, "trk_status copied: %d, %d", status.trk_status, _payload_counter);
                    }
                }
                if (_payload_counter++ == _payload_length)
                    _step++;
                break;

            case 4:
                _step = 0;
                // static uint16_t counter = 0;
                // counter++;
                // if (counter > 20)
                // {
                //     counter = 0;
                //     gcs().send_text(MAV_SEVERITY_INFO, "cam_type: %d", status.cam_type);
                // }
                _port->flush();
                break;
        }

    }

}

void AP_MX28_Control::write_zas_usr_cmd()
{
    uint8_t buf[ZAS_COMM_TX_PACKET_SIZE] = {0};
    uint8_t checksum = 0;

    _port->flush();

    // check for sufficient space in outgoing buffer
    if (_port->txspace() < ZAS_COMM_TX_PACKET_SIZE+1) {
        return;
    }

    buf[0] = GCS_HDR1;
    buf[1] = GCS_HDR2;
    buf[2] = GCS_HDR3;

    ZAS_MOUNT_MODE mode = (ZAS_MOUNT_MODE)usr_cmd.mode;
    // usr_cmd.mode = 5;
    
    // gcs().send_text(MAV_SEVERITY_INFO, "mode: %d", mode);

    memcpy(&buf[3], &usr_cmd.mode, 1);
    buf[4] = ZAS_COMM_TX_PACKET_SIZE - 7; //msg length only takes into account payload+crc+unused
    // write switch case based on mode

    switch (mode) {

        case ZAS_MOUNT_MODE_HOME:
            break;
        case ZAS_MOUNT_MODE_STABILIZE:
            memcpy(&buf[7], &usr_cmd.submode, 1);
            
            if (usr_cmd.submode == 1){
                memcpy(&buf[8], &usr_cmd.rollrate, 4);
                memcpy(&buf[12], &usr_cmd.pitchrate, 4);
                memcpy(&buf[16], &usr_cmd.yawrate, 4);
            }
            else if (usr_cmd.submode == 2){
                memcpy(&buf[12], &usr_cmd.des_tilt, 4);
                memcpy(&buf[16], &usr_cmd.des_pan, 4);
                // gcs().send_text(MAV_SEVERITY_WARNING, "ZAS Gimbal in STAB mode. SUBmode: %d, cmd_tilt: %f", usr_cmd.submode, usr_cmd.des_tilt);
            }
            break;
        case ZAS_MOUNT_MODE_GEOPOINT:
            memcpy(&buf[8], &usr_cmd.target_lat, 4);
            memcpy(&buf[12], &usr_cmd.target_lng, 4);
            memcpy(&buf[16], &usr_cmd.target_amsl, 4);
            break;
        case ZAS_MOUNT_MODE_PILOT:
            memcpy(&buf[7], &usr_cmd.submode, 1);

            if (usr_cmd.submode == 1){
                memcpy(&buf[8], &usr_cmd.rollrate, 4);
                memcpy(&buf[12], &usr_cmd.pitchrate, 4);
                memcpy(&buf[16], &usr_cmd.yawrate, 4);
            }
            else if (usr_cmd.submode == 2){
                memcpy(&buf[12], &usr_cmd.des_tilt, 4);
                memcpy(&buf[16], &usr_cmd.des_pan, 4);
            }
            break;
        case ZAS_MOUNT_MODE_STOW:
            break;
        case ZAS_MOUNT_MODE_CALIBRATE:
            break;
        case ZAS_MOUNT_MODE_HOME_TRACK:
            memcpy(&buf[8], &usr_cmd.trk_x, 4);
            memcpy(&buf[12], &usr_cmd.trk_y, 4);
            memcpy(&buf[7], &usr_cmd.trk_cmd, 1);
            break;
        case ZAS_MOUNT_MODE_HOME_TRACK_LOCK:
            break;
        case ZAS_MOUNT_MODE_ZOOMRATE:
            memcpy(&buf[7], &usr_cmd.ZoomRate, 2);
            break;
        case ZAS_MOUNT_MODE_ZOOMPOS:
            memcpy(&buf[7], &usr_cmd.Zoom, 2);
            break;
        case ZAS_MOUNT_MODE_TRACKBOX_NUDGE:
            memcpy(&buf[8], &usr_cmd.offset_x, 1);
            memcpy(&buf[7], &usr_cmd.offset_y, 1);
            break;
        case ZAS_MOUNT_MODE_TRACKBOX_SIZE:
            memcpy(&buf[7], &usr_cmd.offset_box, 2);
            break;
        case ZAS_MOUNT_MODE_REBOOT:
            gcs().send_text(MAV_SEVERITY_WARNING, "ZAS Gimbal in REBOOT mode.");
            break;
        default:
            break;
    }

    for (uint8_t i = 0;  i < ZAS_COMM_TX_PACKET_SIZE ; i++) {
        checksum += buf[i];
        _port->write( buf[i] );
    }
    _port->write(checksum);

    return;
}

void AP_MX28_Control::tick(void) {
    uint32_t now = AP_HAL::millis();

    if (now - _last_frame_ms > 100) {
        this->get_uart_data();
        _last_frame_ms = now;
    }
    if (_port == nullptr)
        return;
}

// singleton instance
AP_MX28_Control *AP_MX28_Control::_singleton;

namespace AP {

AP_MX28_Control *mx28_control() {
    return AP_MX28_Control::get_singleton();
}

};