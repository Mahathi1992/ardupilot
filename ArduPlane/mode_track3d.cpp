#include "mode.h"
#include "Plane.h"
#include <AP_Mount/AP_Mount_Servo.h>
#include <AP_Mount/TrackerStabiliser.h>
#include "AP_Mount/SightLine.h"
#include <math.h>
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>

#define HAL_QUADPLANE_ENABLED 1
#define APPROACH_WP 12.8374738 , 77.93949839, 200, Location::AltFrame::ABOVE_TERRAIN
#define ABORT_DIVE_ALTITUDE 100

#define PI 3.141592654
#define CONV_LT_DEG 10000000

#define FORCED_PITCH_TRACK3D_DIVE_ANGLE -1500  // it is probably in centidegrees
#define TARGET_AIRSPEED_DIVE 1000  // iT IS IN CM/SEC

static Location wp1Loc(APPROACH_WP);
static ModeTrack3D *track3DObject = nullptr;
static int16_t dive_airspeed = TARGET_AIRSPEED_DIVE;
static uint16_t abort_dive_altitude = ABORT_DIVE_ALTITUDE;
//static  = ModeTrack3D::State_Mode3_t::State_Mode3_None;

void set_zas_track3d_parameters(mavlink_zas_track3d_parameters_t *track3d_msg){
    wp1Loc.lng = track3d_msg->approach_longitude;
    wp1Loc.lat = track3d_msg->approach_latitude;
    dive_airspeed = track3d_msg->dive_airspeed;
    abort_dive_altitude = track3d_msg->abort_dive_altitude;
    if (track3DObject != nullptr){
        track3DObject->forced_pitch_angle = track3d_msg->dive_pitch;
    }
}

// void send_zas_track3d_parameters(mavlink_channel_t channel){
//     mavlink_zas_track3d_parameters_t packet;
//     packet.approach_latitude = wp1Loc.lat;
//     packet.approach_longitude = wp1Loc.lng;
//     packet.abort_dive_altitude = dive_airspeed;
//     packet.dive_pitch = FORCED_PITCH_TRACK3D_DIVE_ANGLE;
//     packet.dive_airspeed = abort_dive_altitude;
//     if (track3DObject != nullptr){
//         packet.dive_pitch = track3DObject->forced_pitch_angle;
//     }
//     mavlink_msg_zas_track3d_parameters_send_struct(channel, &packet);
// }

bool ModeTrack3D::_enter()
{
    //AP_HAL::UARTDriver *uart1 = hal.serial(4);
    //uart1->printf("Track Mode 3D.\r\n");
    plane.guided_throttle_passthru = false;
    /*
      when entering guided mode we set the target as the current
      location. This matches the behaviour of the copter code
    */

    //plane.guided_WP_loc = plane.current_loc;
    plane.guided_WP_loc = wp1Loc;

    mode3_state = State_Mode3_None;
    plane.set_guided_WP();
    forced_pitch_angle = FORCED_PITCH_TRACK3D_DIVE_ANGLE;
    track3DObject = this;
    return true;
}

void ModeTrack3D::fillDiveLocation(){
    Location current_loc = plane.current_loc;
    ParserSightLine* parserSight =  getSightLineParser();
    double bearing = parserSight->stabiliser->getBearing();
    double tilt = parserSight->stabiliser->getTiltInDegrees();
    int32_t alt;
    struct Coordinate initial, result;
    bool isValid = current_loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, alt);
    
    double distance = tanf(tilt);
    
    if ((fabs(distance) < 0.000001) || (!isValid)){
        plane.guided_WP_loc = wp1Loc;
        plane.set_guided_WP();
        mode3_state = State_Mode3_None;
        return;
    }
    distance = alt/ distance;
    initial.Y = current_loc.lat , initial.X = current_loc.lng , initial.Z = alt;
    initial.Y /= CONV_LT_DEG;
    initial.X /= CONV_LT_DEG;
    
    ModeTrack2D::GetTarget(initial, bearing, distance, 0, &result);
    Location end_location(result.Y*CONV_LT_DEG, result.X*CONV_LT_DEG, 0, Location::AltFrame::ABOVE_TERRAIN);
    diveLocation = end_location;
    plane.guided_WP_loc = diveLocation;
    mode3_state = State_Mode3_DIVE;
}

void ModeTrack3D::update(){
    
    switch (mode3_state){
        case State_Mode3_None:
            // We are cruising to State_Mode3_Trackpoint1
            plane.guided_WP_loc = wp1Loc;
            if (plane.current_loc.get_distance(wp1Loc) < 10){
                mode3_state = State_Mode3_Trackpoint1;
            }
            break;
        case State_Mode3_Trackpoint1:
            // We reached Trackpoint 1
            fillDiveLocation();
            break;
        case State_Mode3_DIVE:
            if (plane.current_loc.alt > abort_dive_altitude){
                ParserSightLine* parserSight =  getSightLineParser();
                if (!parserSight->stabiliser->isCurrentlyTracking){
                    // We lost tracking. Abort
                    mode3_state = State_Mode3_None;
                    // Go to APPROACH_WP
                    plane.guided_WP_loc = wp1Loc;
                } else {
                    fillDiveLocation();
                }
            }
            break;
        default:
            mode3_state = State_Mode3_None;
            break;
    }
#if HAL_QUADPLANE_ENABLED
    if (plane.auto_state.vtol_loiter && plane.quadplane.available()) {
        plane.quadplane.guided_update();
        return;
    }
#endif
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
    if (mode3_state == State_Mode3_DIVE){
        plane.target_airspeed_cm = dive_airspeed;
    }
}

void ModeTrack3D::navigate()
{
    // Zero indicates to use WP_LOITER_RAD
    plane.update_loiter(0);
}

bool ModeTrack3D::handle_guided_request(Location target_loc)
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
