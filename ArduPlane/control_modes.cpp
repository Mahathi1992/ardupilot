#include "Plane.h"

#include "quadplane.h"
#include "qautotune.h"

#if HAL_OS_POSIX_IO
#include <stdio.h>
#endif

//const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static void setup_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == nullptr) {
        gcs().send_text(MAV_SEVERITY_INFO, "Setup UART problem: %s", name);
        // that UART doesn't exist on this platform
        return;
    }
    uart->begin(57600);
}

static void test_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    uint8_t command [] = {0xff ,0xff ,0x01 ,0x04 ,0x03 ,0x18 ,0x01 ,0xde
                          ,0xff ,0xff ,0x01 ,0x05 ,0x03 ,0x1e ,0x64 ,0x00 ,0x74 };
    if (uart == nullptr) {
        gcs().send_text(MAV_SEVERITY_INFO, "Test UART problem: %s", name);
        // that UART doesn't exist on this platform
        return;
    }

    uart->printf("Hello on UART %s\n",name);
    printf("Printf statement(test_uart)\n");
    //int ret = uart->write((const uint8_t*)"DATA OUT ON SERIAL.\r\n",21);
    int ret = uart->write( (const uint8_t*) command , 17);
    gcs().send_text(MAV_SEVERITY_INFO, "For Serial %s, write returned: %i.",name, ret);
}

Mode *Plane::mode_from_mode_num(const enum Mode::Number num)
{
    Mode *ret = nullptr;
    switch (num) {
    case Mode::Number::MANUAL:
        ret = &mode_manual;
        break;
    case Mode::Number::CIRCLE:
        ret = &mode_circle;
        break;
    case Mode::Number::STABILIZE:
        ret = &mode_stabilize;
        break;
    case Mode::Number::TRAINING:
        ret = &mode_training;
        break;
    case Mode::Number::ACRO:
        ret = &mode_acro;
        break;
    case Mode::Number::FLY_BY_WIRE_A:
        ret = &mode_fbwa;
        break;
    case Mode::Number::FLY_BY_WIRE_B:
        ret = &mode_fbwb;
        break;
    case Mode::Number::CRUISE:
        ret = &mode_cruise;
        break;
    case Mode::Number::AUTOTUNE:
        ret = &mode_autotune;
        break;
    case Mode::Number::AUTO:
        ret = &mode_auto;
        break;
    case Mode::Number::RTL:
        ret = &mode_rtl;
        break;
    case Mode::Number::LOITER:
        ret = &mode_loiter;
        break;
    case Mode::Number::AVOID_ADSB:
        ret = &mode_avoidADSB;
        break;
    case Mode::Number::GUIDED:
        ret = &mode_guided;
        break;
        
    case Mode::Number::GIMBALNAV:
        ret = &mode_gimbalnav;
        break;
        
    case Mode::Number::TRACK2D:
        ret = &mode_track2d;
        break;
        
    case Mode::Number::TRACK3D:
        ret = &mode_track3d;
        break;
        
    case Mode::Number::GPSDIVE:
        ret = &mode_gpsdive;
        break;
        
    case Mode::Number::VBTG:
        ret = &mode_vbgt;
        break;
        
    case Mode::Number::INITIALISING:
        ret = &mode_initializing;
        break;
    case Mode::Number::QSTABILIZE:
        ret = &mode_qstabilize;
        break;
    case Mode::Number::QHOVER:
        ret = &mode_qhover;
        break;
    case Mode::Number::QLOITER:
        ret = &mode_qloiter;
        break;
    case Mode::Number::QLAND:
        ret = &mode_qland;
        break;
    case Mode::Number::QRTL:
        ret = &mode_qrtl;
        break;
    case Mode::Number::QACRO:
        ret = &mode_qacro;
        break;
    case Mode::Number::QAUTOTUNE:
        ret = &mode_qautotune;
        break;
    case Mode::Number::TAKEOFF:
        ret = &mode_takeoff;
        break;
    }
    return ret;
}

void Plane::read_control_switch()
{
    static bool switch_debouncer;
    uint8_t switchPosition = readSwitch();

    // If switchPosition = 255 this indicates that the mode control channel input was out of range
    // If we get this value we do not want to change modes.
    if(switchPosition == 255) return;

    if (failsafe.rc_failsafe || failsafe.throttle_counter > 0) {
        // when we are in rc_failsafe mode then RC input is not
        // working, and we need to ignore the mode switch channel
        return;
    }

    if (millis() - failsafe.last_valid_rc_ms > 100) {
        // only use signals that are less than 0.1s old.
        return;
    }

    // we look for changes in the switch position. If the
    // RST_SWITCH_CH parameter is set, then it is a switch that can be
    // used to force re-reading of the control switch. This is useful
    // when returning to the previous mode after a failsafe or fence
    // breach. This channel is best used on a momentary switch (such
    // as a spring loaded trainer switch).
    if (oldSwitchPosition != switchPosition ||
        (g.reset_switch_chan != 0 &&
         RC_Channels::get_radio_in(g.reset_switch_chan-1) > RESET_SWITCH_CHAN_PWM)) {

        if (switch_debouncer == false) {
            // this ensures that mode switches only happen if the
            // switch changes for 2 reads. This prevents momentary
            // spikes in the mode control channel from causing a mode
            // switch
            switch_debouncer = true;
            return;
        }

        set_mode_by_number((enum Mode::Number)flight_modes[switchPosition].get(), ModeReason::RC_COMMAND);

        oldSwitchPosition = switchPosition;
    }

    if (g.reset_mission_chan != 0 &&
        RC_Channels::get_radio_in(g.reset_mission_chan-1) > RESET_SWITCH_CHAN_PWM) {
        mission.start();
        prev_WP_loc = current_loc;
    }

    switch_debouncer = false;

#if PARACHUTE == ENABLED
    if (g.parachute_channel > 0) {
        if (RC_Channels::get_radio_in(g.parachute_channel-1) >= 1700) {
            parachute_manual_release();
        }
    }
#endif
}

uint8_t Plane::readSwitch(void)
{
    uint16_t pulsewidth = RC_Channels::get_radio_in(g.flight_mode_channel - 1);
    if (pulsewidth <= 900 || pulsewidth >= 2200) return 255;            // This is an error condition
    if (pulsewidth <= 1230) return 0;
    if (pulsewidth <= 1360) return 1;
    if (pulsewidth <= 1490) return 2;
    if (pulsewidth <= 1620) return 3;
    if (pulsewidth <= 1749) return 4;              // Software Manual
    return 5;                                                           // Hardware Manual
}

void Plane::reset_control_switch()
{
    oldSwitchPosition = 254;
    read_control_switch();
}

/*
  called when entering autotune
 */
void Plane::autotune_start(void)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Started UART");
    /*
    setup_uart(hal.serial(0), "SERIAL0");  // console
    setup_uart(hal.serial(1), "SERIAL1");  // telemetry 1
    setup_uart(hal.serial(2), "SERIAL2");  // telemetry 2
    setup_uart(hal.serial(3), "SERIAL3");  // 1st GPS
    setup_uart(hal.serial(4), "SERIAL4");  // 2nd GPS
    setup_uart(hal.serial(5), "SERIAL5");  //
    setup_uart(hal.serial(6), "SERIAL6");  //
    hal.serial(6)->set_flow_control(AP_HAL::UARTDriver::flow_control::FLOW_CONTROL_DISABLE);
    setup_uart(hal.serial(7), "SERIAL7");  // Debug UART // ACM2
    setup_uart(hal.serial(8), "SERIAL8");  // Debug UART // ACM2
    setup_uart(hal.serial(9), "SERIAL9");  // Debug UART // ACM2
    setup_uart(hal.serial(10), "SERIAL10");  // Debug UART // ACM2
    

    for(int i=0; i<2; i++){
        test_uart(hal.serial(0), "SERIAL0");
        test_uart(hal.serial(1), "SERIAL1");
        test_uart(hal.serial(2), "SERIAL2");
        test_uart(hal.serial(3), "SERIAL3");
        test_uart(hal.serial(4), "SERIAL4");
        test_uart(hal.serial(5), "SERIAL5");
        test_uart(hal.serial(6), "SERIAL6");
        test_uart(hal.serial(7), "SERIAL7");
        test_uart(hal.serial(8), "SERIAL");
        test_uart(hal.serial(9), "SERIAL9");
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Stopped UART");
    */
    gcs().send_text(MAV_SEVERITY_INFO, "Started autotune");
    rollController.autotune_start();
    pitchController.autotune_start();
}

/*
  called when exiting autotune
 */
void Plane::autotune_restore(void)
{
    rollController.autotune_restore();
    pitchController.autotune_restore();
    gcs().send_text(MAV_SEVERITY_INFO, "Stopped autotune");
}

/*
  enable/disable autotune for AUTO modes
 */
void Plane::autotune_enable(bool enable)
{
    if (enable) {
        autotune_start();
    } else {
        autotune_restore();
    }
}

/*
  are we flying inverted?
 */
bool Plane::fly_inverted(void)
{
    if (control_mode == &plane.mode_manual) {
        return false;
    }
    if (inverted_flight) {
        // controlled with aux switch
        return true;
    }
    if (control_mode == &mode_auto && auto_state.inverted_flight) {
        return true;
    }
    return false;
}
