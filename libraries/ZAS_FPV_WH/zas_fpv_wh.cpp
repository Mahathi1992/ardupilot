#include <ZAS_FPV_WH/zas_fpv_wh.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

//constructor
ZAS_FPV_WH::ZAS_FPV_WH() {
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


void ZAS_FPV_WH::init() {
    //const AP_SerialManager& serial_manager = AP::serialmanager();
    
    _port = AP::serialmanager().get_warheadFPV_uart();
    if ((_port != nullptr )) {
        hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&ZAS_FPV_WH::tick, void));
    }
    if (_port != nullptr){
        _port->printf("ZAS_FPV_WH INIT.\r\n");
    }
    //_port = nullptr;
}

void ZAS_FPV_WH::get_uart_data() {

    if (!isSerialInit){
        _port->begin(9600,128,128);
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        isSerialInit = true;
    }
    
    read_incoming_zas_fpv_wh(); // This is used with MX28 gimbal no tracking

}

void ZAS_FPV_WH::handle_usr_cmd_fpv_wh(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_ZAS_WARHEAD_COMMAND:
            handle_zas_warhead_command(msg);
            break;
        // case MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD:
        //     handle_zas_mount_control(msg);
        //     break;
        // case MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_PILOT_STAB:
        //     handle_zas_mount_control_pilot(msg);
        //     break;
        // case MAVLINK_MSG_ID_ZAS_GIMBAL_USR_CMD_GEOPOINT:
        //     handle_zas_mount_control_geopoint(msg);
        //     break;
        default:
            gcs().send_text(MAV_SEVERITY_WARNING, "ZAS MSG TYPE NOT DEFINED");
            break;
    }
}

// Control the ZAS Gimbal 
void ZAS_FPV_WH::handle_zas_warhead_command(const mavlink_message_t &msg)
{
    mavlink_zas_warhead_command_t packet;
    mavlink_msg_zas_warhead_command_decode(&msg, &packet);

    usr_cmd.wh_state_cmd = packet.warhead_state; //mode=201
    usr_cmd.fire_cmd = packet.fire_command; 

    gcs().send_text(MAV_SEVERITY_INFO, "hand controller commands decoded. wh_state: %d, fire_cmd: %d", packet.warhead_state, packet.fire_command);

    write_zas_usr_cmd_fpv_wh();
}

// function to send ZAS gimbal status to GCS

void ZAS_FPV_WH::send_zas_warhead_status(mavlink_channel_t chan)
{
    static uint16_t counter = 0;
    // static bool print_flag = false;
    counter++;
    if (counter > 50)
    {
        counter = 0;
        // print_flag = true;
        gcs().send_text(MAV_SEVERITY_INFO, "target_system: %d", status.target_system);
        gcs().send_text(MAV_SEVERITY_INFO, "power_status: %d", status.power_status);
        gcs().send_text(MAV_SEVERITY_INFO, "arm_status: %d", status.arm_status);
        gcs().send_text(MAV_SEVERITY_INFO, "disarm_status: %d", status.disarm_status);
    }

    mavlink_msg_zas_arm_ack_send(chan, status.target_system, status.power_status, status.arm_status, status.disarm_status);
}

/*
 * detect and read the header of the incoming message from the ZAS gimbal
 */
void ZAS_FPV_WH::read_incoming_zas_fpv_wh()
{
    uint8_t data;
    int16_t numc;
    uint8_t _zas_buf[ZAS_warhead_RX_PACKET_SIZE] = {0};

    numc = _port->available();
    if (numc < 0 ){
        return;
    }

    static uint16_t counter = 0;
    static bool print_flag = false;
    counter++;
    if (counter > 50)
    {
        counter = 0;
        print_flag = true;
        gcs().send_text(MAV_SEVERITY_INFO, "numc: %d", numc);
    }

    if(print_flag) {} // just to avoid warnings 

    for (int16_t i = 0; i < numc; i++) {        // Process bytes received
        data = _port->read();

        switch (_step) {
            case 0:
                if (data == 0x50 || data == 0x41 || data == 0x46) {
                    if (data == 0x50) {
                        _step = 1;
                    }
                    if (data == 0x41) {
                        _step = 3;
                    }
                    if (data == 0x46) {
                        _step = 6;
                    }
                    
                    // _checksum = 0; //reset checksum accumulator
                    gcs().send_text(MAV_SEVERITY_INFO, "Case 0, Byte 0: %d", data);
                }
                break;

            case 1:
                if (data == 0x57) {
                    _step = 2;
                    _checksum = 0; //reset checksum accumulator
                    gcs().send_text(MAV_SEVERITY_INFO, "Case 1, Byte 1: %d", data);
                }
                break;

            case 2: // parsing body
                if (_payload_counter <= ZAS_warhead_RX_PACKET_SIZE) {
                    _zas_buf[_payload_counter] = data;
                    if(_payload_counter == 2){
                        memcpy(&response_power_byte_2, &_zas_buf[2],1);
                        if (print_flag) {
                            gcs().send_text(MAV_SEVERITY_INFO, "response_power_byte_2 copied: %d, %d", response_power_byte_2, _payload_counter);
                        }
                    }
                    if(_payload_counter == 3){
                        memcpy(&response_power_byte_3, &_zas_buf[3],1);
                        if (print_flag) {
                            gcs().send_text(MAV_SEVERITY_INFO, "response_power_byte_3 copied: %d, %d", response_power_byte_3, _payload_counter);
                        }
                    }
                    if(_payload_counter == 4){
                        memcpy(&response_power_byte_4, &_zas_buf[4],1);
                        if (print_flag) {
                            gcs().send_text(MAV_SEVERITY_INFO, "response_power_byte_4 copied: %d, %d", response_power_byte_4, _payload_counter);
                        }
                    }
                    
                }
                if (_payload_counter++ == ZAS_warhead_RX_PACKET_SIZE || data == warhead_PKT_FOOTER) 
                {
                    if (print_flag && data == warhead_PKT_FOOTER) {
                            gcs().send_text(MAV_SEVERITY_INFO, "Footer: %d, %d", data, _payload_counter);
                    }
                    _step = 10;
                }
                break;
            
            case 3:
                if (data == 0x52 || data == 0x42) {
                    if (data == 0x52) {
                        _step = 4;
                    }
                    if (data == 0x42) {
                        _step = 5;
                    }
                }
                break;

            case 4: // parsing body for ARM response
                if (_payload_counter <= ZAS_warhead_RX_PACKET_SIZE) {
                    _zas_buf[_payload_counter] = data;
                    if(_payload_counter == 2){
                        memcpy(&response_arm_byte_2, &_zas_buf[2],1);
                        if (print_flag) {
                            gcs().send_text(MAV_SEVERITY_INFO, "response_arm_byte_2 copied: %d, %d", response_arm_byte_2, _payload_counter);
                        }
                    }
                    if(_payload_counter == 3){
                        memcpy(&response_arm_byte_3, &_zas_buf[3],1);
                        if (print_flag) {
                            gcs().send_text(MAV_SEVERITY_INFO, "response_arm_byte_3 copied: %d, %d", response_arm_byte_3, _payload_counter);
                        }
                    }
                    if(_payload_counter == 4){
                        memcpy(&response_arm_byte_4, &_zas_buf[4],1);
                        if (print_flag) {
                            gcs().send_text(MAV_SEVERITY_INFO, "response_arm_byte_4 copied: %d, %d", response_arm_byte_4, _payload_counter);
                        }
                    }
                    
                }
                if (_payload_counter++ == ZAS_warhead_RX_PACKET_SIZE || data == warhead_PKT_FOOTER) 
                {
                    if (print_flag && data == warhead_PKT_FOOTER) {
                            gcs().send_text(MAV_SEVERITY_INFO, "Footer: %d, %d", data, _payload_counter);
                    }
                    _step = 10;
                }
                break;

            case 5: // parsing body for ABORT response
                if (_payload_counter <= ZAS_warhead_RX_PACKET_SIZE) {
                    _zas_buf[_payload_counter] = data;
                    if(_payload_counter == 2){
                        memcpy(&response_abort_byte_2, &_zas_buf[2],1);
                        if (print_flag) {
                            gcs().send_text(MAV_SEVERITY_INFO, "response_abort_byte_2 copied: %d, %d", response_abort_byte_2, _payload_counter);
                        }
                    }
                    if(_payload_counter == 3){
                        memcpy(&response_abort_byte_3, &_zas_buf[3],1);
                        if (print_flag) {
                            gcs().send_text(MAV_SEVERITY_INFO, "response_abort_byte_3 copied: %d, %d", response_abort_byte_3, _payload_counter);
                        }
                    }
                    if(_payload_counter == 4){
                        memcpy(&response_abort_byte_4, &_zas_buf[4],1);
                        if (print_flag) {
                            gcs().send_text(MAV_SEVERITY_INFO, "response_abort_byte_4 copied: %d, %d", response_abort_byte_4, _payload_counter);
                        }
                    }
                    
                }
                if (_payload_counter++ == ZAS_warhead_RX_PACKET_SIZE || data == warhead_PKT_FOOTER) 
                {
                    if (print_flag && data == warhead_PKT_FOOTER) {
                            gcs().send_text(MAV_SEVERITY_INFO, "Footer: %d, %d", data, _payload_counter);
                    }
                    _step = 10;
                }
                break;

            case 6:
                if (data == 0x49) {
                    _step = 7;
                    _checksum = 0; //reset checksum accumulator
                    gcs().send_text(MAV_SEVERITY_INFO, "FIRE response, Byte 1: %d", data);
                }
                break;

            case 7: // parsing body for FIRE response
                if (_payload_counter <= ZAS_warhead_RX_PACKET_SIZE) {
                    _zas_buf[_payload_counter] = data;
                    if(_payload_counter == 2){
                        memcpy(&response_fire_byte_2, &_zas_buf[2],1);
                        if (print_flag) {
                            gcs().send_text(MAV_SEVERITY_INFO, "response_fire_byte_2 copied: %d, %d", response_fire_byte_2, _payload_counter);
                        }
                    }
                    if(_payload_counter == 3){
                        memcpy(&response_fire_byte_3, &_zas_buf[3],1);
                        if (print_flag) {
                            gcs().send_text(MAV_SEVERITY_INFO, "response_fire_byte_3 copied: %d, %d", response_fire_byte_3, _payload_counter);
                        }
                    }
                    if(_payload_counter == 4){
                        memcpy(&response_fire_byte_4, &_zas_buf[4],1);
                        if (print_flag) {
                            gcs().send_text(MAV_SEVERITY_INFO, "response_fire_byte_4 copied: %d, %d", response_fire_byte_4, _payload_counter);
                        }
                    }
                    
                }
                if (_payload_counter++ == ZAS_warhead_RX_PACKET_SIZE || data == warhead_PKT_FOOTER) 
                {
                    if (print_flag && data == warhead_PKT_FOOTER) {
                            gcs().send_text(MAV_SEVERITY_INFO, "Footer: %d, %d", data, _payload_counter);
                    }
                    _step = 10;
                }
                break;

            case 10:
                _step = 0;
                _port->flush();
                break;
        }

    }

//     // AP_Logger *logger = AP_Logger::get_singleton();
//     // logger->Write_GIMZ();
    
//     AP::logger().Write("GimZ", "TimeUS,cmdPan,cmdTilt,pan,tilt,az,el", "Qffffff",
//                                         AP_HAL::micros64(),
//                                         status.cmd_pan,
//                                         status.cmd_tilt,
//                                         status.pan,
//                                         status.tilt,
//                                         status.azimuth,
//                                         status.elevation);

}

void ZAS_FPV_WH::write_zas_usr_cmd_fpv_wh()
{
//     uint8_t buf[ZAS_COMM_TX_PACKET_SIZE] = {0};
//     uint8_t checksum = 0;

//     // _port->flush();

//     // check for sufficient space in outgoing buffer
//     if (_port->txspace() < ZAS_COMM_TX_PACKET_SIZE+1) {
//         return;
//     }

//     buf[0] = GCS_HDR1;
//     buf[1] = GCS_HDR2;
//     buf[2] = GCS_HDR3;

//     ZAS_MOUNT_MODE mode = (ZAS_MOUNT_MODE)usr_cmd.mode;
//     // usr_cmd.mode = 5;
    
//     // gcs().send_text(MAV_SEVERITY_INFO, "mode: %d", mode);

//     memcpy(&buf[3], &usr_cmd.mode, 1);
//     buf[4] = ZAS_COMM_TX_PACKET_SIZE - 7; //msg length only takes into account payload+crc+unused
//     // write switch case based on mode

//     switch (mode) {

//         case ZAS_MOUNT_MODE_HOME:
//             break;
//         case ZAS_MOUNT_MODE_STABILIZE:
//             memcpy(&buf[7], &usr_cmd.submode, 1);
            
//             if (usr_cmd.submode == 1){
//                 memcpy(&buf[8], &usr_cmd.rollrate, 4);
//                 memcpy(&buf[12], &usr_cmd.pitchrate, 4);
//                 memcpy(&buf[16], &usr_cmd.yawrate, 4);
//             }
//             else if (usr_cmd.submode == 2){
//                 memcpy(&buf[12], &usr_cmd.des_tilt, 4);
//                 memcpy(&buf[16], &usr_cmd.des_pan, 4);
//                 // gcs().send_text(MAV_SEVERITY_DEBUG, "ZAS Gimbal in STAB mode. SUBmode: %d, cmd_pan: %f", usr_cmd.submode, usr_cmd.des_pan);
//             }
//             break;
//         case ZAS_MOUNT_MODE_GEOPOINT:
//             memcpy(&buf[8], &usr_cmd.target_lat, 4);
//             memcpy(&buf[12], &usr_cmd.target_lng, 4);
//             memcpy(&buf[16], &usr_cmd.target_amsl, 4);
//             break;
//         case ZAS_MOUNT_MODE_PILOT:
//             memcpy(&buf[7], &usr_cmd.submode, 1);

//             if (usr_cmd.submode == 1){
//                 memcpy(&buf[8], &usr_cmd.rollrate, 4);
//                 memcpy(&buf[12], &usr_cmd.pitchrate, 4);
//                 memcpy(&buf[16], &usr_cmd.yawrate, 4);
//             }
//             else if (usr_cmd.submode == 2){
//                 memcpy(&buf[12], &usr_cmd.des_tilt, 4);
//                 memcpy(&buf[16], &usr_cmd.des_pan, 4);
//             }
//             break;
//         case ZAS_MOUNT_MODE_STOW:
//             break;
//         case ZAS_MOUNT_MODE_CALIBRATE:
//             break;
//         case ZAS_MOUNT_MODE_HOME_TRACK:
//             memcpy(&buf[8], &usr_cmd.trk_x, 4);
//             memcpy(&buf[12], &usr_cmd.trk_y, 4);
//             memcpy(&buf[7], &usr_cmd.trk_cmd, 1);
//             break;
//         case ZAS_MOUNT_MODE_HOME_TRACK_LOCK:
//             break;
//         case ZAS_MOUNT_MODE_ZOOMRATE:
//             memcpy(&buf[7], &usr_cmd.ZoomRate, 2);
//             break;
//         case ZAS_MOUNT_MODE_ZOOMPOS:
//             memcpy(&buf[7], &usr_cmd.Zoom, 2);
//             break;
//         case ZAS_MOUNT_MODE_TRACKBOX_NUDGE:
//             memcpy(&buf[8], &usr_cmd.offset_x, 1);
//             memcpy(&buf[7], &usr_cmd.offset_y, 1);
//             break;
//         case ZAS_MOUNT_MODE_TRACKBOX_SIZE:
//             memcpy(&buf[7], &usr_cmd.offset_box, 2);
//             break;
//         case ZAS_MOUNT_MODE_REBOOT:
//             gcs().send_text(MAV_SEVERITY_WARNING, "ZAS Gimbal in REBOOT mode.");
//             break;
//         case ZAS_MOUNT_MODE_WHOT_BHOT:
//             memcpy(&buf[7], &usr_cmd.wb_mode, 1);
//             break;
//         default:
//             break;
//     }

//     for (uint8_t i = 0;  i < ZAS_COMM_TX_PACKET_SIZE ; i++) {
//         checksum += buf[i];
//         _port->write( buf[i] );
//     }
//     _port->write(checksum);

    return;
}

void ZAS_FPV_WH::tick(void) {
    uint32_t now = AP_HAL::millis();

    if (now - _last_frame_ms > 20) {
        this->get_uart_data();
        _last_frame_ms = now;
    }
    if (_port == nullptr)
        return;
}

// singleton instance
ZAS_FPV_WH *ZAS_FPV_WH::_singleton;

namespace AP {
    ZAS_FPV_WH *zas_fpv_wh() {
        return ZAS_FPV_WH::get_singleton();
    }
};