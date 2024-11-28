/*
 * Author       : Mahathi
 * Description  : mx28 gimbal Control functionality through serial port
 * Date         : 28-11-2024
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>


class AP_MX28_Control {
public:
    //constructor
    AP_MX28_Control();

    /* Do not allow copies */
    AP_MX28_Control(const AP_MX28_Control &other) = delete;
    AP_MX28_Control &operator=(const AP_MX28_Control&) = delete;
	// get singleton instance
    static AP_MX28_Control *get_singleton() {
           return _singleton;
       }

    void init();

public:

    // tick - main call to send updates to transmitter
    void tick(void);

    AP_HAL::UARTDriver *_port;              // UART used to send data to receiver
    uint32_t _last_frame_ms;
    mavlink_channel_t chan;
    bool isChannelSet;
    bool isSerialInit;

protected:
    static AP_MX28_Control *_singleton;
    
    void get_uart_data();

};

namespace AP {
AP_MX28_Control *mx28_control();
};
