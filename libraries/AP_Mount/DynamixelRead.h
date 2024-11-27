#pragma once

#include "AP_Mount_Servo.h"

#define MAX_PAYLOAD_LENGTH_DYNAMIXEL 40
#define TILT_ID 1
#define PAN_ID 2
#define SERVO_MAX 4096

enum DynamixelParserState_t { DynamixelParserState_None = 0, DynamixelParserState_Sync1, DynamixelParserState_Sync2, DynamixelParserState_Id,
    DynamixelParserState_Length,  DynamixelParserState_Error, DynamixelParserState_Payload, DynamixelParserState_CRC, DynamixelParserState_Max};

class DynamixelParser {
public:
    DynamixelParser();
    void reset(){
        wb = 0; rb = 0;
        state = DynamixelParserState_None;
    }
    void stateMachine();
    void resetPresentPositionStatus(){
        bulk_read_present_position_status = 0;
        is_any_present_value_pending = 1;
    }
    uint8_t are_present_positions_known(){
        return (bulk_read_present_position_status & 0x3);
    }
    void send_read_position_command();
public:
    uint8_t rb, wb;
    const uint8_t SYNC1;
    const uint8_t SYNC2;
    uint8_t id, length, crc, error;
    int8_t  nBytesLeft;
    uint8_t payload[MAX_PAYLOAD_LENGTH_DYNAMIXEL], buffer[0x100];
    uint16_t pan_value, tilt_value;
    uint8_t bulk_read_present_position_status, is_any_present_value_pending;
    AP_HAL::UARTDriver *uartDebug, *inputUart;
    enum DynamixelParserState_t state;
protected:
    void pan_value_received(){
        bulk_read_present_position_status |= 0x01;
        if (are_present_positions_known()){
            is_any_present_value_pending = 0;
        }
    }
    void tilt_value_received(){
        bulk_read_present_position_status |= 0x02;
        if (are_present_positions_known()){
            is_any_present_value_pending = 0;
        }
    }
    void execute();
    
};


uint8_t compute_checksum(uint8_t * command, uint8_t len);
