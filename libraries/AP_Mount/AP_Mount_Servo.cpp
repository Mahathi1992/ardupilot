#include "AP_Mount_Servo.h"
#include "TrackerStabiliser.h"
#include "SightLine.h"
#include "DynamixelRead.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_GPS/AP_GPS.h>

#define PAN_MAX_ANGLE  1800
#define PAN_MIN_ANGLE -1800

#define TILT_MAX_ANGLE  1200
#define TILT_MIN_ANGLE -450


#define MAX_SERVO_LIMIT 4096 // 3072 // 2048 //4096 //0x3FF
#define PAN_SERVO_LIMIT 4096
#define MIN_PAN_LIMIT   0

#define TILT_SERVO_LIMIT (((2048*TILT_MAX_ANGLE)/1800) + 2048)
#define MIN_TILT_LIMIT   512


#define SERVO_PAN_PER_PIXEL 0.5
#define SERVO_TILT_PER_PIXEL 0

#define INVALID_SERVO_GOAL -10000
//#define UART4_AS_DEBUG


extern const AP_HAL::HAL& hal;
static ParserSightLine parser;
static DynamixelParser dynamixelParser;
AP_HAL::UARTDriver *uartTracker = nullptr, *uart1Debug = nullptr, *uartDynamixel = nullptr;
//static int16_t last_pan_goal = INVALID_SERVO_GOAL, pan_mov_delta= 0, last_pan_mov = INVALID_SERVO_GOAL;
//static int8_t inc_or_dec = 0;
//static mavlink_channel_t  chan;
static uint16_t pan_goal, tilt_goal;
static int16_t poff = 0,toff = 0; // Pitch offset, Tilt Offset
mavlink_channel_t channel_to_gcs;

// extern void send_zas_track3d_parameters(mavlink_channel_t channel);
// extern void send_zas_gpsd_parameters(mavlink_channel_t channel);
// extern void send_zas_track_status_10Hz(mavlink_channel_t);
// extern void send_zas_default_parameters();
// extern void tracker_update_thread_ap_mount_servo();

int16_t angles_to_steps(float angle);
void handle_gimbal_control_servo(float pan_yaw, float tilt_pitch);

void applyTrackParametersUpdate(mavlink_zas_track_parameters_t *params) {
    if (uart1Debug != nullptr) {
        uart1Debug->printf("Setting3\n");
    }
    parser.stabiliser->setTrackParameters(params);
}


// void send_zas_track_status_10Hz(mavlink_channel_t channel) {
//     static uint8_t count_until_send = 0;
//     channel_to_gcs = channel;

//     if (count_until_send < 10) {
//         count_until_send++;
//     } else {
//         count_until_send=0;
//         {
//             mavlink_zas_track_status_t packet;
//             packet.track_latitude = (12.77729233*10000000);
//             packet.track_longitude = (77.939283958*10000000);
//             packet.num_tracks = 1;
//             packet.pan = pan_goal;
//             packet.tilt = tilt_goal;
//             packet.commanded_pan = pan_goal;
//             packet.commanded_tilt = tilt_goal;
//             packet.selected_cam = 1;
//             mavlink_msg_zas_track_status_send_struct(channel, &packet);
//             /*
//             if (uartTracker != nullptr){
//                 uartTracker->printf("lat: %li, lon: %li.\r\n",packet.track_latitude,packet.track_longitude );
//             }
//             */
//         }
//     }
// }

// void send_zas_default_parameters() {
//     mavlink_zas_track_parameters_t track_msg;

//     track_msg.max_misses_before_tracking_lost = parser.stabiliser->max_misses_before_tracking_lost;
//     track_msg.step_value = parser.stabiliser->step_value;
//     track_msg.confidence_threshold = parser.stabiliser->confidence_threshold;
//     track_msg.center_offset_pixel_delta = parser.stabiliser->center_offset_pixel_delta;
//     track_msg.cam0_height = parser.stabiliser->cam0_h;
//     track_msg.cam0_width = parser.stabiliser->cam0_w;
//     track_msg.cam1_height = parser.stabiliser->cam1_h;
//     track_msg.cam1_width = parser.stabiliser->cam1_w;

//     if (uart1Debug != nullptr) {
//         uart1Debug->printf("Sending: Max misses: %li, Step Value: %li, Confidence Threshold: %i, Center Offset: %i, Cam0 W: %i, H: %i, Cam1 W: %i, Cam1 H: %i \r\n",
//                            track_msg.max_misses_before_tracking_lost, track_msg.step_value, track_msg.confidence_threshold, track_msg.center_offset_pixel_delta,
//                            track_msg.cam0_height, track_msg.cam0_width, track_msg.cam1_height, track_msg.cam1_width
//                           );
//     }

//     mavlink_msg_zas_track_parameters_send_struct(channel_to_gcs, &track_msg);

//     send_zas_track3d_parameters(channel_to_gcs);
//     send_zas_gpsd_parameters(channel_to_gcs);
// }

int16_t angles_to_steps(float angle) {
    // 180 degrees is MAX_SERVO_LIMIT steps
    float x = (angle*(MAX_SERVO_LIMIT/2));
    x /= 180;
    return (int16_t) x;
}

void handle_gimbal_control_servo(float pan_yaw, float tilt_pitch) {
    poff = angles_to_steps(pan_yaw);
    toff = angles_to_steps(tilt_pitch);

    if (uart1Debug != nullptr) {
        uart1Debug->printf("Y: %f, P: %f, poff: %i, toff: %i", pan_yaw, tilt_pitch, poff, toff);
    }
    /*
    if (parser.stabiliser != nullptr){
        parser.stabiliser->setOffsets(poff, toff);
    }
    */

}

class ParserSightLine* getSightLineParser(void) {
    return &parser;
}

#define DIRECT_ASSIGN_SERIAL_MANAGER_4_0_7

#ifdef DIRECT_ASSIGN_SERIAL_MANAGER_4_0_7
static void setup_uart() {
    dynamixelParser.inputUart = uartDynamixel;
    if (uart1Debug != nullptr) {
        parser.uartDebug = uart1Debug;
    }
    if (uartTracker != nullptr) {
        uartTracker->printf("Tracker UART started.\n\r");
    }
    if (uart1Debug != nullptr) {
        uart1Debug->printf("Debug UART started.\n\r");
    }
    if (dynamixelParser.inputUart != nullptr) {
        dynamixelParser.inputUart->printf("Dynamixel UART UART started.\n\r");
    }
}

#else
static void setup_uart() {
    const AP_SerialManager& serial_manager = AP::serialmanager();
    AP_HAL::UARTDriver *_port;
    /*
    AP_HAL::UARTDriver *uart;
    uart = hal.serial(2);
    uart->begin(57600);
    */

    // check for Dynamixel servo protcol
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Dynamixel_MX, 0))) {
        dynamixelParser.inputUart = _port;
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Setup UART problem: Dynamixel Serial Not found");
        // that UART doesn't exist on this platform
        //uart->printf("Setup UART problem: Dynamixel Serial Not found");
        return;
    }

    // check for Sightline protcol
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_SightLine, 0))) {
        uartTracker = _port;
        //uart->printf("Tracker UART started.\n\r");
        uartTracker->printf("Tracker UART started.\n\r");
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Setup UART problem: Tracker UART Not found");
        //uart->printf("Setup UART problem: Tracker UART Not found");
        // that UART doesn't exist on this platform
        return;
    }

    uart1Debug = nullptr;
    parser.uartDebug = nullptr;
    // check for ZAS Debug protcol
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_ZASDebug, 0))) {
        uart1Debug = _port;
        parser.uartDebug = _port;
        dynamixelParser.uartDebug = _port;
        //uart->printf("Debug UART started.\n\r");
        uart1Debug->printf("Debug UART started.\n\r");
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Setup UART problem: ZAS Debug Not found");
        //uart->printf("Setup UART problem: ZAS Debug Not found");
        // that UART doesn't exist on this platform
        return;
    }
    /*
    if (uart == nullptr) {
        gcs().send_text(MAV_SEVERITY_INFO, "Setup UART problem");
        // that UART doesn't exist on this platform
        return;
    }
    */
    /*
#ifdef UART4_AS_DEBUG
    uart->begin(57600);
#else
    uart->begin(1000000, 512, 16);
    dynamixelParser.inputUart = uart;
    //uart->begin(57600);
#endif
    */

    //gcs().send_text(MAV_SEVERITY_INFO, "Setup UART problem: Dynamixel Serial Not found");
    // that UART doesn't exist on this platform
    //AP_HAL::UARTDriver *uart2 = hal.serial(2);
    //uart2->begin(57600);
    //parser.uartDebug = uart;
    /*
#ifdef UART4_AS_DEBUG
    parser.uartDebug = uart;
    parser.uartDebug->printf("Debug UART started.\n\r");
#endif
    AP_HAL::UARTDriver *uart1 = hal.serial(1);
    uart1->begin(57600);
    uart1Debug = uart1;
    uart1Debug->printf("Debug UART1 started.\n\r");
    dynamixelParser.uartDebug = uart1;//uart1Debug;
    */

}
#endif // DIRECT_ASSIGN_SERIAL_MANAGER_4_0_7

uint8_t compute_checksum(uint8_t * command, uint8_t len) {
    uint16_t sum = 0, i = 0;
    while (i < len) {
        sum += command[i];
        i++;
    }
    sum = sum& 0xFF;
    uint8_t chksum = sum;
    chksum = ~chksum;
    return (uint8_t) chksum;
}



void tracker_update_thread_ap_mount_servo() {
    uint32_t avail; //, ret;
    //uint8_t cbuf[512];
    int16_t byteRead = 1;

    //uart1Debug->printf("Data1\r\n");
    //uartTracker->printf("Data2\r\n");

    do {
        //ret = uartTracker->read(cbuf);
        do {
            byteRead = uartTracker->read();
            if (byteRead != -1) {
                parser.buffer[parser.wb++] = (uint8_t) byteRead;
                parser.wb = (parser.wb >= parser.BUF_SIZE)?0:parser.wb;
            }
        } while (byteRead != -1);

        /*
        if (ret > 0){
            if (parser.uartDebug != nullptr){
                parser.uartDebug->write(cbuf, ret);
            }
        }
        */


        /*
        for (uint32_t i = 0; i < ret; i++) {
            parser.buffer[parser.wb++] = cbuf[i];
            parser.wb = (parser.wb >= parser.BUF_SIZE)?0:parser.wb;
        }
        */

        avail = uartTracker->available();
    } while (avail != 0);


    if (dynamixelParser.inputUart == nullptr)
        return;
    AP_HAL::UARTDriver *uart = dynamixelParser.inputUart;
    /*
    do {
        ret = dynamixelParser.inputUart->read(cbuf, 512);

        if (ret > 0){
            //if (dynamixelParser.inputUart != nullptr){
                dynamixelParser.inputUart->write(cbuf, ret);
            //}
        }

        for (uint32_t i = 0; i < ret; i++) {
            dynamixelParser.buffer[dynamixelParser.wb++] = cbuf[i];
        }

        //dynamixelParser.inputUart->printf("Read FBC: %li\n", ret);
        avail = uart->available();
    }while (avail != 0);
    */
    do {
        byteRead = uart->read();
        if (byteRead != -1) {
            dynamixelParser.buffer[dynamixelParser.wb++] = (uint8_t) byteRead;
        }
    } while (byteRead != -1);


}

static void get_some_uart_data() {

    if (uartTracker != nullptr) {
        tracker_update_thread_ap_mount_servo();
    }

    if (parser.wb != parser.rb) {
        parser.stateMachine();
    }
    if (dynamixelParser.wb != dynamixelParser.rb) {
        dynamixelParser.stateMachine();
    }
}

static void send_torque_enable() {
    if (dynamixelParser.inputUart == nullptr)
        return;
    AP_HAL::UARTDriver *uart = dynamixelParser.inputUart;
    //uint8_t command [] = {0xff ,0xff ,0x01 ,0x04 ,0x03 ,0x18 ,0x01 ,0xde};
    uint8_t command [] = {0xff,0xff,0xFE,8,0x83,
                          0x18,0x01, 1, 0x01,
                          2, 0x01,
                          0xde
                         };
    command[11] = compute_checksum(command + 2, 9);
    uart->write( (const uint8_t*) command, 12);
}

static void send_move_command(uint16_t pan_goalf __attribute__((unused)), uint16_t  tilt_goalf __attribute__((unused))) {
    if (dynamixelParser.inputUart == nullptr)
        return;
#ifdef UART4_AS_DEBUG
#else
    //if (parser.stabiliser == nullptr){
    pan_goalf -= poff;
    tilt_goalf -= toff;
    //}
    if (pan_goalf >= PAN_SERVO_LIMIT) {
        pan_goalf = PAN_SERVO_LIMIT;
    }
    if (tilt_goalf <= MIN_TILT_LIMIT) {
        tilt_goalf = MIN_TILT_LIMIT;
    }
    if (tilt_goalf >= TILT_SERVO_LIMIT) {
        tilt_goalf = TILT_SERVO_LIMIT;
    }
    
    AP_HAL::UARTDriver *uart = dynamixelParser.inputUart;
    if (uart == nullptr)
        return;
    // PAN is ID 1
    // Tilt is ID 2
    uint8_t ph, pl, th, tl;
    ph = pan_goalf >> 8;
    pl = pan_goalf & 0xff;
    th = tilt_goalf >> 8;
    tl = tilt_goalf & 0xff;
    uint8_t command [] = { 0xff, 0xff, 0xFE, 0x0e, 0x83,
                           0x1e, 0x04, 1, tl, th, 0x50, 0x02,
                           2, pl, ph, 0x50, 0x02,
                           0x0
                         };
    command[17]  = compute_checksum(command + 2, 15);
    uart->write( (const uint8_t*) command, 18);
#endif
}


static void send_execute() {
    if (dynamixelParser.inputUart == nullptr)
        return;
#ifdef UART4_AS_DEBUG
#else
    AP_HAL::UARTDriver *uart = dynamixelParser.inputUart;
    uint8_t action [] = {0xFF,0xFF, 0xFE, 0x02,0x05,0xFA};
    uart->write( (const uint8_t*) action, 6);
#endif
}


// init - performs any required initialisation for this instance
void AP_Mount_Servo::init() {
    if (_instance == 0) {
        _roll_idx = SRV_Channel::k_mount_roll;
        _tilt_idx = SRV_Channel::k_mount_tilt;
        _pan_idx  = SRV_Channel::k_mount_pan;
        _open_idx = SRV_Channel::k_mount_open;
    } else {
        // this must be the 2nd mount
        _roll_idx = SRV_Channel::k_mount2_roll;
        _tilt_idx = SRV_Channel::k_mount2_tilt;
        _pan_idx  = SRV_Channel::k_mount2_pan;
        _open_idx = SRV_Channel::k_mount2_open;
    }

    // check which servos have been assigned
    check_servo_map();
    gcs().send_text(MAV_SEVERITY_INFO, "Servo AP_Mount_Servo");
    setup_uart();
    
    if (dynamixelParser.inputUart != nullptr){
        dynamixelParser.inputUart->printf("Hello from INIT.");
        send_torque_enable();
        send_execute();
    }
    //send_move_command(40, 40);
    //send_execute();
    //AP_HAL::UARTDriver *uart2 = hal.serial(2);
    //uart2->begin(57600);
    //uartTracker = uart2;
}

/*
static uint16_t convert_angle_to_goal(int16_t angle, uint8_t pan_is_zero0){
    int32_t angle32 = angle;
    int32_t MAX_ANGLE = PAN_MAX_ANGLE;
    int32_t MIN_ANGLE = PAN_MIN_ANGLE;
    int32_t max_servo_limit = MAX_SERVO_LIMIT;

    if (pan_is_zero0 != 0){
        MIN_ANGLE = TILT_MIN_ANGLE;
        MAX_ANGLE = TILT_MAX_ANGLE;
        max_servo_limit = PAN_SERVO_LIMIT;
    }

    angle32 -= MIN_ANGLE;
    double an1 = (angle32*max_servo_limit)/(MAX_ANGLE - MIN_ANGLE);
    if (an1 >= MAX_SERVO_LIMIT){
        an1 = MAX_SERVO_LIMIT - 1;
    }
    if (an1 < 0){
        an1 = 1;
    }
    return (uint16_t) an1;
}
*/

uint16_t AP_Mount_Servo::getPanGoalFromServoAngles() {
    float min_angle = _state._pan_angle_min*0.01f;
    float max_angle = _state._pan_angle_max*0.01f;
    float m = -11.3;
    float c = 2048;
    float a = _angle_bf_output_deg.z;
    if (a < min_angle) {
        a = min_angle;
    }
    if (a > max_angle) {
        a = max_angle;
    }
    float y = (m*a) + c;
    return y;
}

uint16_t AP_Mount_Servo::getTiltGoalFromServoAngles() {
    float min_angle = _state._tilt_angle_min*0.01f;
    float max_angle = _state._tilt_angle_max*0.01f;
    float m = -11.3;
    float c = 2048;
    float a = _angle_bf_output_deg.y;
    if (a < min_angle) {
        a = min_angle;
    }
    if (a > max_angle) {
        a = max_angle;
    }
    float y = (m*a) + c;
    return y;
}

// update mount position - should be called periodically
void AP_Mount_Servo::update() {
    static bool mount_open = 0;     // 0 is closed

    // check servo map every three seconds to allow users to modify parameters
    uint32_t now = AP_HAL::millis();
    if (now - _last_check_servo_map_ms > 3000) {
        check_servo_map();
        _last_check_servo_map_ms = now;
    }

    switch(get_mode()) {
    // move mount to a "retracted position" or to a position where a fourth servo can retract the entire mount into the fuselage
    case MAV_MOUNT_MODE_RETRACT: {
        _angle_bf_output_deg = _state._retract_angles.get();
        break;
    }

    // move mount to a neutral position, typically pointing forward
    case MAV_MOUNT_MODE_NEUTRAL: {
        _angle_bf_output_deg = _state._neutral_angles.get();
        /*
        if (!parser.stabiliser->isCurrentlyTracking){
            send_move_command(2048, 2048);
            send_execute();
        }
        */
        static uint8_t hasGimbalBeenSetToNeutral = 0;
        if (hasGimbalBeenSetToNeutral != 1) {
            poff = 0;
            toff = 0;
            send_move_command(2048, 2048);
            send_execute();
            hasGimbalBeenSetToNeutral = 1;
        }

        break;
    }

    // point to the angles given by a mavlink message
    case MAV_MOUNT_MODE_MAVLINK_TARGETING: {
        // earth-frame angle targets (i.e. _angle_ef_target_rad) should have already been set by a MOUNT_CONTROL message from GCS
        stabilize();
        break;
    }

    // RC radio manual angle control, but with stabilization from the AHRS
    case MAV_MOUNT_MODE_RC_TARGETING: {
        // update targets using pilot's rc inputs
        update_targets_from_rc();
        stabilize();
        break;
    }

    // point mount to a GPS point given by the mission planner
    case MAV_MOUNT_MODE_GPS_POINT: {
        //gcs().send_text(MAV_SEVERITY_INFO, "Set ROI - 7");
        if(AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
            calc_angle_to_location(_state._roi_target, _angle_ef_target_rad, _flags.tilt_control, _flags.pan_control, false);
            stabilize();
        }
        break;
    }

    default:
        //do nothing
        break;
    }

    // move mount to a "retracted position" into the fuselage with a fourth servo
    bool mount_open_new = (get_mode() == MAV_MOUNT_MODE_RETRACT) ? 0 : 1;
    if (mount_open != mount_open_new) {
        mount_open = mount_open_new;
        move_servo(_open_idx, mount_open_new, 0, 1);
    }

    // write the results to the servos
    //move_servo(_roll_idx, _angle_bf_output_deg.x*10, _state._roll_angle_min*0.1f, _state._roll_angle_max*0.1f);
    //move_servo(_tilt_idx, _angle_bf_output_deg.y*10, _state._tilt_angle_min*0.1f, _state._tilt_angle_max*0.1f);
    //move_servo(_pan_idx,  _angle_bf_output_deg.z*10, _state._pan_angle_min*0.1f, _state._pan_angle_max*0.1f);
    if ( (_angle_bf_output_deg.z*10 < _state._pan_angle_min*0.1f) ) {
        _angle_bf_output_deg.z = _state._pan_angle_min*0.01f;
    }
    if ( (_angle_bf_output_deg.z*10 >= _state._pan_angle_max*0.1f) ) {
        _angle_bf_output_deg.z = _state._pan_angle_max*0.01f;
    }

    if ( (_angle_bf_output_deg.y*10 < _state._tilt_angle_min*0.1f) ) {
        _angle_bf_output_deg.y = _state._tilt_angle_min*0.01f;
    }
    if ( (_angle_bf_output_deg.y*10 >= _state._tilt_angle_max*0.1f) ) {
        _angle_bf_output_deg.y = _state._tilt_angle_max*0.01f;
    }

    //pan_goal = convert_angle_to_goal(_angle_bf_output_deg.z*10, 0);
    //tilt_goal = convert_angle_to_goal(_angle_bf_output_deg.y*10, 1);
    pan_goal = getPanGoalFromServoAngles();
    tilt_goal = getTiltGoalFromServoAngles();

    if (parser.stabiliser->isCurrentlyTracking) {
        pan_goal += parser.stabiliser->pan_correction;
        tilt_goal += parser.stabiliser->tilt_correction;
    } else if (get_mode() == MAV_MOUNT_MODE_NEUTRAL) {
        pan_goal = 2048;
        tilt_goal = 2048;
    }

    send_move_command(pan_goal, tilt_goal);
    send_execute();
    get_some_uart_data();
    dynamixelParser.send_read_position_command();
    /*
    if (dynamixelParser.uartDebug != nullptr){
        dynamixelParser.uartDebug->printf("CP: %i, P: %i  CT: %i T: %i\r\n",pan_goal, dynamixelParser.pan_value, tilt_goal, dynamixelParser.tilt_value);
    }
    */
    //tilt_goal = 2048;
    //last_pan_goal = 2048;
    //send_move_command(last_pan_goal, tilt_goal);
    //send_execute();
    //send_move_command(0,0);
#if 0

    if (last_pan_goal == INVALID_SERVO_GOAL) {
        last_pan_goal = 2048;
        tilt_goal = 2048;
    }

    if (last_pan_mov == INVALID_SERVO_GOAL) {
        last_pan_mov = 0;
    }
    last_pan_mov++;
    if (last_pan_mov >= 1) {
        if (inc_or_dec == 0) {
            last_pan_goal++;
            pan_mov_delta++;
        } else {
            last_pan_goal--;
            pan_mov_delta--;
        }
        pan_goal = 0;
        last_pan_mov = pan_goal;
    }
    if (last_pan_goal >= PAN_SERVO_LIMIT) {
        last_pan_goal = PAN_SERVO_LIMIT;
        inc_or_dec = 1;
    }

    if (last_pan_goal <= 0) {
        inc_or_dec = 0;
    }
    uart1Debug->printf("P: %i, %i\r\n\r\n", last_pan_mov, last_pan_goal);
#endif
#if 0
    if (c_track_update == 1) {
        int16_t pan_mov = ((CAMERA_WIDTH/2.0) - c_track_x); //*SERVO_PAN_PER_PIXEL;
        float tilt_mov = ((CAMERA_HEIGHT/2.0) - c_track_y)*SERVO_TILT_PER_PIXEL;
        if ((pan_mov ) > 5) {
            // We haven't seen any movement
            /*
            if ((pan_mov * last_pan_mov) < 0){
                pan_mov_delta = 0;
            }
            */
            if (last_pan_mov != INVALID_SERVO_GOAL) {
                int32_t prodmul = last_pan_mov;
                prodmul *= pan_mov;
                if (prodmul < 0) {
                    // We crossed over
                    pan_mov = last_pan_mov;
                } else {
                    pan_mov_delta++;
                }
            }
            last_pan_mov = pan_mov;
            //last_pan_mov = pan_mov;
            if (pan_mov < 0) {
                pan_mov -= pan_mov_delta;
            } else {
                pan_mov += pan_mov_delta;
            }
        } else {
            last_pan_mov = pan_mov;
        }
        if (pan_goal > pan_mov) {
            pan_goal -= (pan_mov*SERVO_PAN_PER_PIXEL);
        } else {
            pan_goal = 0;
        }
        if (tilt_goal > tilt_mov) {
            tilt_goal -= tilt_mov;
        } else {
            tilt_goal = 0;
        }
        uart1Debug->printf("P: %i, %i, %i, %i; T: %f, %i\r\nCX: %i, CY: %i\r\n\r\n", pan_mov, pan_goal, last_pan_mov, pan_mov_delta, tilt_mov, tilt_goal, c_track_x, c_track_y);

        send_move_command(pan_goal, tilt_goal);
        c_track_update = 0;
        last_pan_goal = pan_goal;
    } else {
        //pan_mov_delta =0;
        //send_move_command(pan_goal, tilt_goal);
    }
#endif
}

// set_mode - sets mount's mode
void AP_Mount_Servo::set_mode(enum MAV_MOUNT_MODE mode) {
    // record the mode change and return success
    _state._mode = mode;
}

// private methods

// check_servo_map - detects which axis we control using the functions assigned to the servos in the RC_Channel_aux
//  should be called periodically (i.e. 1hz or less)
void AP_Mount_Servo::check_servo_map() {
    _flags.roll_control = SRV_Channels::function_assigned(_roll_idx);
    _flags.tilt_control = SRV_Channels::function_assigned(_tilt_idx);
    _flags.pan_control = SRV_Channels::function_assigned(_pan_idx);
}

// send_mount_status - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
void AP_Mount_Servo::send_mount_status(mavlink_channel_t chan) {
    mavlink_msg_mount_status_send(chan, 0, 0, _angle_bf_output_deg.y*100, _angle_bf_output_deg.x*100, _angle_bf_output_deg.z*100);
}

// stabilize - stabilizes the mount relative to the Earth's frame
//  input: _angle_ef_target_rad (earth frame targets in radians)
//  output: _angle_bf_output_deg (body frame angles in degrees)
void AP_Mount_Servo::stabilize() {
    AP_AHRS &ahrs = AP::ahrs();
    // only do the full 3D frame transform if we are doing pan control
    if (_state._stab_pan) {
        Matrix3f m;                         ///< holds 3 x 3 matrix, var is used as temp in calcs
        Matrix3f cam;                       ///< Rotation matrix earth to camera. Desired camera from input.
        Matrix3f gimbal_target;             ///< Rotation matrix from plane to camera. Then Euler angles to the servos.
        m = ahrs.get_rotation_body_to_ned();
        m.transpose();
        cam.from_euler(_angle_ef_target_rad.x, _angle_ef_target_rad.y, _angle_ef_target_rad.z);
        gimbal_target = m * cam;
        gimbal_target.to_euler(&_angle_bf_output_deg.x, &_angle_bf_output_deg.y, &_angle_bf_output_deg.z);
        _angle_bf_output_deg.x  = _state._stab_roll ? degrees(_angle_bf_output_deg.x) : degrees(_angle_ef_target_rad.x);
        _angle_bf_output_deg.y  = _state._stab_tilt ? degrees(_angle_bf_output_deg.y) : degrees(_angle_ef_target_rad.y);
        _angle_bf_output_deg.z = degrees(_angle_bf_output_deg.z);
    } else {
        // otherwise base mount roll and tilt on the ahrs
        // roll/tilt attitude, plus any requested angle
        _angle_bf_output_deg.x = degrees(_angle_ef_target_rad.x);
        _angle_bf_output_deg.y = degrees(_angle_ef_target_rad.y);
        _angle_bf_output_deg.z = degrees(_angle_ef_target_rad.z);
        if (_state._stab_roll) {
            _angle_bf_output_deg.x -= degrees(ahrs.roll);
        }
        if (_state._stab_tilt) {
            _angle_bf_output_deg.y -= degrees(ahrs.pitch);
        }

        // lead filter
        const Vector3f &gyro = ahrs.get_gyro();

        if (_state._stab_roll && !is_zero(_state._roll_stb_lead) && fabsf(ahrs.pitch) < M_PI/3.0f) {
            // Compute rate of change of euler roll angle
            float roll_rate = gyro.x + (ahrs.sin_pitch() / ahrs.cos_pitch()) * (gyro.y * ahrs.sin_roll() + gyro.z * ahrs.cos_roll());
            _angle_bf_output_deg.x -= degrees(roll_rate) * _state._roll_stb_lead;
        }

        if (_state._stab_tilt && !is_zero(_state._pitch_stb_lead)) {
            // Compute rate of change of euler pitch angle
            float pitch_rate = ahrs.cos_pitch() * gyro.y - ahrs.sin_roll() * gyro.z;
            _angle_bf_output_deg.y -= degrees(pitch_rate) * _state._pitch_stb_lead;
        }
    }
}

// closest_limit - returns closest angle to 'angle' taking into account limits.  all angles are in degrees * 10
int16_t AP_Mount_Servo::closest_limit(int16_t angle, int16_t angle_min, int16_t angle_max) {
    // Make sure the angle lies in the interval [-180 .. 180[ degrees
    while (angle < -1800) angle += 3600;
    while (angle >= 1800) angle -= 3600;

    // Make sure the angle limits lie in the interval [-180 .. 180[ degrees
    while (angle_min < -1800) angle_min += 3600;
    while (angle_min >= 1800) angle_min -= 3600;
    while (angle_max < -1800) angle_max += 3600;
    while (angle_max >= 1800) angle_max -= 3600;

    // If the angle is outside servo limits, saturate the angle to the closest limit
    // On a circle the closest angular position must be carefully calculated to account for wrap-around
    if ((angle < angle_min) && (angle > angle_max)) {
        // angle error if min limit is used
        int16_t err_min = angle_min - angle + (angle<angle_min ? 0 : 3600);     // add 360 degrees if on the "wrong side"
        // angle error if max limit is used
        int16_t err_max = angle - angle_max + (angle>angle_max ? 0 : 3600);     // add 360 degrees if on the "wrong side"
        angle = err_min<err_max ? angle_min : angle_max;
    }

    return angle;
}

// move_servo - moves servo with the given id to the specified angle.  all angles are in degrees * 10
void AP_Mount_Servo::move_servo(uint8_t function_idx, float angle, float angle_min, float angle_max) {
    static int8_t count_print = 0;
    // saturate to the closest angle limit if outside of [min max] angle interval
    //int16_t servo_out = closest_limit(angle, angle_min, angle_max);
    //SRV_Channels::move_servo((SRV_Channel::Aux_servo_function_t)function_idx, servo_out, angle_min, angle_max);
    if (count_print >= 9) {
        if (uart1Debug != nullptr) {
            uart1Debug->printf("Mount Servo: [idx]  %i, [angle] %f, %f, %f ", function_idx, angle, angle_min, angle_max);
        }
        count_print = 0;
    }
    count_print++;

}
