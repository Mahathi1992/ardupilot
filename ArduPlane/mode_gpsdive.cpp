#include "mode.h"
#include "Plane.h"
#include <AP_Mount/AP_Mount_Servo.h>
#include <math.h>
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>
#include "SimpleMovingAverage.h"

#define HAL_QUADPLANE_ENABLED 1
#define APPROACH_WP 12.8374738 , 77.93949839, 200, Location::AltFrame::ABSOLUTE
#define ABORT_DIVE_ALTITUDE 100

#define PI 3.141592654
#define CONV_LT_DEG 10000000

#define FORCED_PITCH_TRACK3D_DIVE_ANGLE -1500  // it is probably in centidegrees
#define TARGET_AIRSPEED_DIVE 1000  // iT IS IN CM/SEC

#define DEFAULT_DIVE_PITCH_SATURATION -1500
#define DEFAULT_PITCH_OFFSET 0

#define APPROACH2

extern mavlink_channel_t channel_to_gcs;

Location approachWp(APPROACH_WP), targetWp(APPROACH_WP), abortWp(APPROACH_WP);
ModeGPSDive *trackGpsdObj = nullptr;
static SimpleMovingAverage<5, int32_t, int32_t> sma;
static bool isTargetValid = false;
static uint16_t approach_loiter_radius = 0, abort_dive_altitude = ABORT_DIVE_ALTITUDE;

// void send_zas_gpsd_parameters(mavlink_channel_t channel);
// void set_zas_gpsd_parameters(mavlink_channel_t channel, mavlink_zas_gpsd_parameters_t *gpsdive_msg);
void checkIfTargetIsValid();
double shiftCalculatedHeadingToNorthAs0(double heading);

void checkIfTargetIsValid(){
    if ( (approachWp.lng == 0) && (approachWp.lat == 0) && (approachWp.alt == 0) && 
        (targetWp.lng == 0) && (targetWp.lat == 0) && (targetWp.alt == 0)){
        isTargetValid = false;
    }
    isTargetValid = true;
}

// void set_zas_gpsd_parameters(mavlink_channel_t channel,mavlink_zas_gpsd_parameters_t *gpsdive_msg){
    
//     if (gpsdive_msg->loiter_direction == 100){
//         send_zas_gpsd_parameters(channel);
//         return;
//     }
    
//     approachWp.lng = gpsdive_msg->approach_longitude;
//     approachWp.lat = gpsdive_msg->approach_latitude;
//     approachWp.alt = gpsdive_msg->approach_altitude;
    
//     targetWp.lng = gpsdive_msg->target_longitude;
//     targetWp.lat = gpsdive_msg->target_latitude;
//     targetWp.alt = gpsdive_msg->target_altitude;
    
//     abortWp.lng = gpsdive_msg->abort_longitude;
//     abortWp.lat = gpsdive_msg->abort_latitude;
//     abortWp.alt = gpsdive_msg->abort_altitude;
    
    
//     if (trackGpsdObj != nullptr){
//         trackGpsdObj->target_heading = gpsdive_msg->target_heading;
//         trackGpsdObj->loiter_direction = gpsdive_msg->loiter_direction;
//         trackGpsdObj->dive_pitch_saturation = gpsdive_msg->dive_pitch_saturation;
//         trackGpsdObj->pitch_offset = gpsdive_msg->pitch_offset;
//         if (trackGpsdObj->loiter_direction == 0){
//             approachWp.loiter_ccw = 0;
//             targetWp.loiter_ccw = 0;
//             abortWp.loiter_ccw = 0;
//         } else {
//             approachWp.loiter_ccw = 1;
//             targetWp.loiter_ccw = 1;
//             abortWp.loiter_ccw = 1;
//         }
//         trackGpsdObj->target_heading += 180;
//         trackGpsdObj->target_heading *= 100;
//         trackGpsdObj->target_heading = wrap_360_cd(trackGpsdObj->target_heading);
//         trackGpsdObj->target_heading /= 100;
//     }
//     approach_loiter_radius = gpsdive_msg->approach_loiter_radius;
//     abort_dive_altitude = gpsdive_msg->abort_dive_altitude;
//     checkIfTargetIsValid();
//     gcs().send_text(MAV_SEVERITY_INFO, "Approach Alt is %li cms; lat: %li lon: %li", approachWp.alt,approachWp.lat,approachWp.lng);
//     gcs().send_text(MAV_SEVERITY_INFO, "Target Alt is %li cms; lat: %li lon: %li", targetWp.alt,targetWp.lat,targetWp.lng);
//     gcs().send_text(MAV_SEVERITY_INFO, "Abort Alt is %li cms; lat: %li lon: %li", abortWp.alt,abortWp.lat,abortWp.lng);
    
//     gcs().send_text(MAV_SEVERITY_INFO, "T. Head is %i cdegs; L. dir.: %i; D.P.S.: %i", gpsdive_msg->target_heading,gpsdive_msg->loiter_direction,gpsdive_msg->dive_pitch_saturation);
//     gcs().send_text(MAV_SEVERITY_INFO, "Pitch. Off.: %i cdegs; App. LR: %i; Abort DA: %i", gpsdive_msg->pitch_offset, approach_loiter_radius, abort_dive_altitude) ;
// }

// void send_zas_gpsd_parameters(mavlink_channel_t channel){
//     mavlink_zas_gpsd_parameters_t packet;
//     packet.approach_latitude = approachWp.lat;
//     packet.approach_longitude = approachWp.lng;
//     packet.approach_altitude = approachWp.alt;
    
//     packet.target_latitude = targetWp.lat;
//     packet.target_longitude = targetWp.lng;
//     packet.target_altitude = targetWp.alt;
    
//     packet.abort_latitude = abortWp.lat;
//     packet.abort_longitude = abortWp.lng;
//     packet.abort_altitude = abortWp.alt;
    
//     packet.approach_loiter_radius = approach_loiter_radius;
//     packet.abort_dive_altitude = abort_dive_altitude;
    
//     if (trackGpsdObj != nullptr){
//         packet.target_heading = trackGpsdObj->target_heading;
//         packet.loiter_direction = trackGpsdObj->loiter_direction;
//         packet.dive_pitch_saturation = trackGpsdObj->dive_pitch_saturation;
//         packet.pitch_offset = trackGpsdObj->pitch_offset;
//         /*
//         packet.dive_pitch_saturation = DEFAULT_DIVE_PITCH_SATURATION;
//         packet.pitch_offset = DEFAULT_PITCH_OFFSET;
//         */
//     }
//     mavlink_msg_zas_gpsd_parameters_send_struct(channel, &packet);
// }

bool ModeGPSDive::_enter(){
    //AP_HAL::UARTDriver *uart1 = hal.serial(4);
    //uart1->printf("Track Mode 3D.\r\n");
    if (!isTargetValid){
        gcs().send_text(MAV_SEVERITY_WARNING, "GPSD: TARGET NOT FOUND");
        return false;
    }
    plane.throttle_allows_nudging = true;
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = true;
    plane.guided_throttle_passthru = false;
    gcs().send_text(MAV_SEVERITY_INFO, "Approach Alt is %li cms; R: %i", approachWp.alt , approach_loiter_radius); //,approachWp.lat,approachWp.lng);
    /*
      when entering guided mode we set the target as the current
      location. This matches the behaviour of the copter code
    */

    plane.aparm.loiter_radius = approach_loiter_radius;
    //plane.guided_WP_loc = plane.current_loc;
    plane.guided_WP_loc = approachWp;

    mode3_state = State_ModeGPSD_None;
    plane.loiter.direction = (this->loiter_direction == 0)?0:1;
    plane.update_loiter(approach_loiter_radius);
    plane.set_guided_WP();
    trackGpsdObj = this;
    return true;
}

#if 1
void ModeGPSDive::fillDiveLocation(){
    mode3_state = State_ModeGPSD_DIVE;
#ifdef APPROACH2
    //targetWp.alt = approachWp.alt;
    //plane.loiter.direction = (this->loiter_direction == 0)?0:1;
    plane.next_WP_loc = targetWp;
    plane.guided_WP_loc = targetWp;
    //plane.update_loiter(approach_loiter_radius);
    plane.set_guided_WP();
    plane.set_flight_stage(AP_Vehicle::FixedWing::FLIGHT_LAND);
    mode3_state = State_ModeGPSD_DIVE;
    plane.roll_limit_cd = 50;
#else
    do_make_land();
#endif
    plane.auto_state.wp_is_land_approach = 1;
    plane.aparm.loiter_radius = 2; // Approach 1
}


void ModeGPSDive::do_make_land(){
    plane.set_next_WP(targetWp);

    // configure abort altitude and pitch
    // if NAV_LAND has an abort altitude then use it, else use last takeoff, else use 50m
    plane.auto_state.takeoff_altitude_rel_cm = 1000;

    if (plane.auto_state.takeoff_pitch_cd <= 0) {
        // If no takeoff command has ever been used, default to a conservative 10deg
        plane.auto_state.takeoff_pitch_cd = 1000;
    }

    // zero rangefinder state, start to accumulate good samples now
    memset(&(plane.rangefinder_state), 0, sizeof(plane.rangefinder_state));

    plane.landing.do_land_dive(plane.relative_altitude);

    // if we were in an abort we need to explicitly move out of the abort state, as it's sticky
    plane.set_flight_stage(AP_Vehicle::FixedWing::FLIGHT_LAND);
    plane.roll_limit_cd = 50;
}


bool ModeGPSDive::isPlaneInDive(){
#ifdef APPROACH2
    return (mode3_state == State_ModeGPSD_DIVE);
#else
    return false;
#endif
}

void ModeGPSDive::update_dive(){
    double distance = plane.current_loc.get_distance(targetWp);
    // Calculate forced pitch
    //double pitch = atan2f((plane.current_loc.alt - targetWp.alt)/100, distance); // Convert cm to meters
    double pitch = acosf(((plane.current_loc.alt - targetWp.alt)/100)/ distance); // Convert cm to meters
    // convert to degrees.
    pitch = (pitch*-18000)/PI; // Flip the Y axis, down is negative pitch.
    pitch += pitch_offset;
    
    if (pitch < dive_pitch_saturation)
        pitch = dive_pitch_saturation;
    commanded_pitch = (int32_t)pitch; // We are working in cdegs
    
    plane.guided_state.forced_throttle = 0;
    plane.guided_state.forced_rpy_cd.y = commanded_pitch;
    
    //plane.nav_pitch_cd = commanded_pitch;
    
    plane.pitch_limit_min_cd = dive_pitch_saturation;
}

#else
void ModeGPSDive::fillDiveLocation(){
    double distance = plane.current_loc.get_distance(targetWp);
    // Calculate forced pitch
    double pitch = atan2f((plane.current_loc.alt - targetWp.alt)/100, distance); // Convert cm to meters
    pitch += pitch_offset;
    // convert to degrees.
    pitch = (pitch*18000)/PI;
    
    if (pitch < dive_pitch_saturation)
        pitch = dive_pitch_saturation;
    commanded_pitch = sma((uint32_t)pitch); // We are working in cdegs
    
    plane.nav_pitch_cd = commanded_pitch;
    plane.auto_state.wp_is_land_approach = 1;
    plane.guided_WP_loc = targetWp;
    mode3_state = State_ModeGPSD_DIVE;
}
#endif

double shiftCalculatedHeadingToNorthAs0(double heading){
    //heading += 90;
    if (heading < 0){
        heading += 360;
    }
    if (heading >= 360){
        heading -= 360;
    }
    return heading;
}

bool ModeGPSDive::checkifTargetHeadingIsCorrect(){
    static int counter = 0;
    /*
    Vector2f groundSpeed = AP::ahrs().groundspeed_vector();
    double heading = 0;
    if (groundSpeed.y == 0){
        if (groundSpeed.x > 0){
            heading = 90;
        } else {
            heading = 270;
        }
    } else {
        heading = atan2f(groundSpeed.x, groundSpeed.y);
    }
    heading = (heading*18000)/PI;
    */
    double heading = wrap_360_cd(degrees(plane.ahrs.yaw)*100);
    heading /= 100;
    //heading = shiftCalculatedHeadingToNorthAs0(heading);
    //heading *= 100;
    if (counter >= 40){
        gcs().send_text(MAV_SEVERITY_WARNING, "GPSD:Hdg: %f, tgt: %d, df: %i", heading, target_heading, (int)fabs(heading - target_heading));
        counter = 0;
    } else {
        counter++;
    }
    return (fabs(heading - target_heading) < 5);
    //return false;
}

void ModeGPSDive::update(){ 
    static int counter = 0;
    
    switch (mode3_state){
        case State_ModeGPSD_None:
            // We are cruising to State_ModeGPSD_Approach_Alt
            plane.guided_WP_loc = approachWp;
            if (counter >= 40){
                gcs().send_text(MAV_SEVERITY_WARNING, "GPSD:Distance Appr.: %i", (int)plane.current_loc.get_distance(approachWp));
                counter = 0;
            } else {
                counter++;
            }
            if (plane.current_loc.get_distance(approachWp) < (approach_loiter_radius + 50)){
                gcs().send_text(MAV_SEVERITY_WARNING, "GPSD: ARRIVED AT APPROACH");
                mode3_state = State_ModeGPSD_Approach_Alt;
                counter = 0;
            }
            break;
        case State_ModeGPSD_Approach_Alt:
            // We reached Trackpoint 1
            // Check if we are at the correct altitude
            if (counter >= 40){
                gcs().send_text(MAV_SEVERITY_INFO, "GPSD: Alt %li, %li", plane.current_loc.alt , approachWp.alt);
                counter = 0;
            } else {
                counter++;
            }
            if (fabs(plane.current_loc.alt - approachWp.alt) < (10*100)){
                gcs().send_text(MAV_SEVERITY_WARNING, "GPSD: Altitude Attained");
                mode3_state = State_ModeGPSD_Hdg;
            }
            break;
        case State_ModeGPSD_Hdg:
            // Check that we have the currect heading for breakout
            if (checkifTargetHeadingIsCorrect()){
                gcs().send_text(MAV_SEVERITY_WARNING, "GPSD: HEADING CORRECT BREAKOUT");
                mode3_state = State_ModeGPSD_DIVE;
                fillDiveLocation();
            }
            break;
        case State_ModeGPSD_DIVE:
            update_dive();
            if (counter >= 80){
                gcs().send_text(MAV_SEVERITY_INFO, "T %li cms;C: %li,D: %f", targetWp.alt ,plane.current_loc.alt, plane.current_loc.get_distance(targetWp));
                counter = 0;
            } else {
                counter++;
            }
            //fillDiveLocation();
            break;
        case State_ModeGPSD_Abort:
            plane.guided_WP_loc = abortWp;
            if (plane.current_loc.get_distance(abortWp) < 10){
                mode3_state = State_ModeGPSD_None;
            }
            break;
        default:
            plane.guided_WP_loc = approachWp;
            mode3_state = State_ModeGPSD_None;
            break;
    }
    
#if HAL_QUADPLANE_ENABLED
    if (plane.auto_state.vtol_loiter && plane.quadplane.available()) {
        plane.quadplane.guided_update();
        return;
    }
#endif
    if (plane.auto_state.vtol_loiter && plane.quadplane.available()) {
        plane.quadplane.guided_update();
    } else {
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        plane.calc_throttle();
    }
    if (mode3_state == State_ModeGPSD_DIVE){
        if (counter == 40){
            gcs().send_text(MAV_SEVERITY_INFO, "LAND p:%li cp:%li, of: %i, st: %i\n", plane.guided_state.forced_rpy_cd.y, plane.nav_pitch_cd,pitch_offset, dive_pitch_saturation);
        }
    }
    //plane.calc_nav_roll();
    //plane.calc_nav_pitch();
    //plane.calc_throttle();
}

void ModeGPSDive::navigate(){
    /*
    uint16_t radius = 0;
    switch (mode3_state){
        case State_ModeGPSD_None:
            radius = approach_loiter_radius;
            break;
        case State_ModeGPSD_Trackpoint1:
            radius = 0;
            break;
        case State_ModeGPSD_DIVE:
            radius = 0;
            break;
        case State_ModeGPSD_Abort:
            radius = 0;
            break;
        default:
            break;
    }
    // Zero indicates to use WP_LOITER_RAD
    plane.update_loiter(radius);
    */
}

bool ModeGPSDive::handle_guided_request(Location target_loc)
{
    plane.guided_WP_loc = target_loc;

    // add home alt if needed
    if (plane.guided_WP_loc.relative_alt) {
        plane.guided_WP_loc.alt += plane.home.alt;
        plane.guided_WP_loc.relative_alt = 0;
    }

    plane.set_guided_WP();

    return true;
}
