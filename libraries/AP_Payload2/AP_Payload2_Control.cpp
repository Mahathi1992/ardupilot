#include <AP_Payload2/AP_Payload2_Control.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

//constructor
AP_Payload2_Control::AP_Payload2_Control() {
    if (_singleton != nullptr) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Payload2 must be singleton");
#endif
        return;
    }
    isChannelSet = false;
    isSerialInit = false;
    _singleton = this;
}


void AP_Payload2_Control::init() {
    //const AP_SerialManager& serial_manager = AP::serialmanager();
    
    _port = AP::serialmanager().get_payload2_uart();
    if ((_port != nullptr )) {
        hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Payload2_Control::tick, void));
    }
    if (_port != nullptr){
        _port->printf("AP_Payload2 INIT.\r\n");
    }
    //_port = nullptr;
}

void AP_Payload2_Control::get_uart_data() {

    uint8_t data;
    int16_t numc;
    uint8_t isDataReady = 0;

    numc = _port->available();

    if (numc < 0 ){
        return;
    }
   
    if (!isSerialInit){
        _port->begin(115200,128,128);
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        isSerialInit = true;
    }

    if(numc>0){
        for(int i=0;i<sizeof(numc);i++){
            data = _port->read();
            uint8_t val = data;
            parser.buffer[parser.wb++] = val;
        }
        isDataReady = 1;
    }
    
    if (isDataReady){
        parser.stateMachine();
    }
}

void AP_Payload2_Control::send_warhead_status(mavlink_channel_t chan)
{
    Teensy2Message message = AP::payload2_control()->get_teensy_message();
    mavlink_msg_zmotion_payload2_response_send(chan, PAYLOAD2_COMMANDS, PAYLOAD2_UNUSED, 0, message.chassisID, message.chassisHealth, message.pid, message.msg);
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AP_Payload2: Warhead status being sent to GCS");
}

void AP_Payload2_Control::tick(void) {
    uint32_t now = AP_HAL::millis();

    if (now - _last_frame_ms > 200) {
        this->get_uart_data();
        _last_frame_ms = now;
        // static uint16_t counter = 0;
        // counter++;
        // if (counter > 100)
        // {
        //     counter = 0;
        //     gcs().send_text(MAV_SEVERITY_INFO, "Warhead ticking at 5Hz");
        // }
    }
    if (_port == nullptr)
        return;
}

// singleton instance
AP_Payload2_Control *AP_Payload2_Control::_singleton;

namespace AP {

AP_Payload2_Control *payload2_control() {
    return AP_Payload2_Control::get_singleton();
}

};