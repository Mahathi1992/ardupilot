#include "mode.h"
#include "Plane.h"
#include <AP_Mount/AP_Mount_Servo.h>
#include <AP_Mount/TrackerStabiliser.h>
#include "AP_Mount/SightLine.h"
#include <math.h>
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>


#define HAL_QUADPLANE_ENABLED 1
#define PI 3.141592654
#define CONV_LT_DEG 10000000

#define FORCED_PITCH_TRACK3D_DIVE_ANGLE -1500  // it is probably in centidegrees
#define TARGET_AIRSPEED_DIVE 1000  // iT IS IN CM/SEC


bool ModeVBGT::_enter()
{
    //AP_HAL::UARTDriver *uart1 = hal.serial(4);
    //uart1->printf("Track Mode 3D.\r\n");
    plane.guided_throttle_passthru = false;
    /*
      when entering guided mode we set the target as the current
      location. This matches the behaviour of the copter code
    */

    //plane.guided_WP_loc = plane.current_loc;
    //plane.guided_WP_loc = wp1Loc;

    //plane.set_guided_WP();
    return true;
}

void ModeVBGT::update(){
#if HAL_QUADPLANE_ENABLED
    if (plane.auto_state.vtol_loiter && plane.quadplane.available()) {
        plane.quadplane.guided_update();
        return;
    }
#endif
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}

void ModeVBGT::navigate()
{
    // Zero indicates to use WP_LOITER_RAD
    plane.update_loiter(0);
}

bool ModeVBGT::handle_guided_request(Location target_loc)
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
