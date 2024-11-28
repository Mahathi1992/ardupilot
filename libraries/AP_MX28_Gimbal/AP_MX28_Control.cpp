#include <AP_Payload2/AP_MX28_Control.h>
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
#define ENABLE_STAGE1 1
#define ENABLE_STAGE2 1

void AP_Payload2_Control::get_uart_data() {
    uint32_t avail = 0 ;
#if ENABLE_STAGE1 
    int16_t byteRead = -1;
#endif    
    uint8_t isDataReady = 0;

    if (!isSerialInit){
        _port->begin(115200,128,128);
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        isSerialInit = true;
    }
    //_port->write((const uint8_t*)"-A-" , 3);
    isDataReady = 0;
    do {
#if ENABLE_STAGE1
        do {
            byteRead = _port->read();
            if (byteRead != -1) {
#if ENABLE_STAGE2
                uint8_t val = (uint8_t) byteRead;
                parser.buffer[parser.wb++] = val;
                //_port->write( (const uint8_t *) &val, 1);
                isDataReady = 1;
#endif
            }
        } while (byteRead != -1);

        avail = _port->available();
#endif
    } while (avail != 0);
    if (isDataReady){
        parser.stateMachine();
    }
}

void AP_Payload2_Control::tick(void) {
    uint32_t now = AP_HAL::millis();

    if (now - _last_frame_ms > 1000) {
        _last_frame_ms = now;
    }
    if (_port == nullptr)
        return;

    this->get_uart_data();
}

// singleton instance
AP_Payload2_Control *AP_Payload2_Control::_singleton;

namespace AP {

AP_Payload2_Control *payload2_control() {
    return AP_Payload2_Control::get_singleton();
}

};