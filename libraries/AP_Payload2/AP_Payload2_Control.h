/*
 * Author       : Akshay Mathur <akshay.mathur@theorax-dynamics.com>
 * Description  : Payload2 Control functionality through serial port
 * Date         : 24-11-2022
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include  <AP_Payload2/TeensyPayload2Parser.h>


class AP_Payload2_Control {
public:
    //constructor
    AP_Payload2_Control();

    /* Do not allow copies */
    AP_Payload2_Control(const AP_Payload2_Control &other) = delete;
    AP_Payload2_Control &operator=(const AP_Payload2_Control&) = delete;
	// get singleton instance
    static AP_Payload2_Control *get_singleton() {
           return _singleton;
       }

    void init();

    Teensy2Message& get_teensy_message(){ return  parser.getTeensy2Message();}

    void send_warhead_status(mavlink_channel_t chan);


public:

    // tick - main call to send updates to transmitter
    void tick(void);

    AP_HAL::UARTDriver *_port;              // UART used to send data to receiver
    uint32_t _last_frame_ms;
    // mavlink_channel_t chan;
    bool isChannelSet;
    bool isSerialInit;

protected:
    static AP_Payload2_Control *_singleton;
    TeensyPayload2Parser parser;
    
    void get_uart_data();

};

namespace AP {
AP_Payload2_Control *payload2_control();
};
