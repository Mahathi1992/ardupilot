#include "mode.h"
#include "Plane.h"
#include <AP_Mount/AP_Mount_Servo.h>
#include <AP_Mount/TrackerStabiliser.h>
#include "AP_Mount/SightLine.h"
#include <math.h>
#include <stdio.h>

#define HAL_QUADPLANE_ENABLED 1

extern const AP_HAL::HAL& hal;

#define PI 3.141592654
#define CONV_LT_DEG 10000000


double ModeTrack2D::ToDegree(double radian){
  double degree = 180;
  return (degree*radian)/PI;
}

double ModeTrack2D::ToRadian(double degree){
  double radian = PI;
  return (degree*radian)/180;
}

void ModeTrack2D::GetTarget(struct Coordinate origin, double bearing, double distance, double altitude, struct Coordinate *result){
    double d = distance / 6371;
    double rlat = ToRadian(origin.Y);
    double rlon = ToRadian(origin.X);
    double rbearing = ToRadian(bearing);
    double lat2 = rlat + (d * cosf(rbearing));
    double dlat = lat2 - rlat;
    double dphi = log((tanf((lat2 / 2) + (PI / 4))) / (tanf((rlat / 2) + (PI / 4))));
    double q =
        fabs(dlat) > 0.0000000001
        ? dlat / dphi
        : cosf(rlat);
    double dlon = (d * sinf(rbearing)) / q;

    if (fabs(lat2) > PI / 2)
    {
        lat2 = lat2 > 0 ? PI : PI - lat2;
    }

    double lon2 = fmod((rlon + dlon + PI) , (2 * PI)) - PI;
    //printf("One: %f, Two: %f, Three: %f. \n", (rlon + dlon + PI) , (2 * PI), lon2);
    
    result->X = ToDegree(lon2);
    result->Y = ToDegree(lat2);
    result->Z = origin.Z + altitude;
    
    return;
}

bool ModeTrack2D::_enter(){
    //AP_HAL::UARTDriver *uart1 = hal.serial(4);
    //uart1->printf("Track Mode 2D.\r\n");
    plane.guided_throttle_passthru = false;
    calculate_track_endpoint();
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.guided_mode_enabled()) {
        /*
          if using Q_GUIDED_MODE then project forward by the stopping distance
        */
        plane.guided_WP_loc.offset_bearing(degrees(plane.ahrs.groundspeed_vector().angle()),
                                           plane.quadplane.stopping_distance());
    }
#endif

    plane.set_guided_WP();
    return true;
}

void ModeTrack2D::calculate_track_endpoint(){
    /*
      when entering guided mode we set the target as the current
      location. This matches the behaviour of the copter code
    */
    Location current_loc = plane.current_loc;
    ParserSightLine* parserSight =  getSightLineParser();
    double bearing = parserSight->stabiliser->getBearing();
    double tilt = parserSight->stabiliser->getTiltInDegrees();
    int32_t alt;
    struct Coordinate initial, result;
    bool isValid = current_loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, alt);
    
    double distance = tanf(tilt);
    
    if (!parserSight->stabiliser->isCurrentlyTracking){
        return;
    }
    
    if ((fabs(distance) < 0.000001) || (!isValid)){
        plane.guided_WP_loc = plane.current_loc;
        plane.set_guided_WP();
        return;
    }
    distance = alt/ distance;
    initial.Y = current_loc.lat , initial.X = current_loc.lng , initial.Z = alt;
    initial.Y /= CONV_LT_DEG;
    initial.X /= CONV_LT_DEG;
    
    GetTarget(initial, bearing, distance, 0, &result);
    Location end_location(result.Y*CONV_LT_DEG, result.X*CONV_LT_DEG, alt, Location::AltFrame::ABOVE_TERRAIN);
    
    
    
    //plane.guided_WP_loc = plane.current_loc;
    plane.guided_WP_loc = end_location;

}

void ModeTrack2D::update()
{
#if HAL_QUADPLANE_ENABLED
    if (plane.auto_state.vtol_loiter && plane.quadplane.available()) {
        plane.quadplane.guided_update();
        return;
    }
#endif
    calculate_track_endpoint();
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}

void ModeTrack2D::navigate()
{
    // Zero indicates to use WP_LOITER_RAD
    plane.update_loiter(0);
}

bool ModeTrack2D::handle_guided_request(Location target_loc)
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
