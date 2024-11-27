#include "DynamixelRead.h"

DynamixelParser::DynamixelParser(): SYNC1(0xFF), SYNC2(0xFF){
    reset();
    resetPresentPositionStatus();
    inputUart = nullptr;
}

void DynamixelParser::stateMachine(){
    uint8_t v;
    while (rb != wb){
        v = buffer[rb++];
//         if (parser.uartDebug != nullptr){
//             parser.uartDebug->write(&v, 1);
//         }
        switch (state){
            case DynamixelParserState_None:
                //uartDebug->write((const uint8_t*)"-N0-",4);
                if (v == SYNC1)
                    state = DynamixelParserState_Sync1;
                break;
            case DynamixelParserState_Sync1:
                //uartDebug->write((const uint8_t*)"-S1-",4);
                if (v == SYNC2)
                    state = DynamixelParserState_Sync2;
                else 
                    state = DynamixelParserState_None;
                break;
            case DynamixelParserState_Sync2:
                //uartDebug->write((const uint8_t*)"-S2-",4);
                id = v;
                state = DynamixelParserState_Id;
                break;
            case DynamixelParserState_Id:
                //uartDebug->write((const uint8_t*)"-ID-",4);
                length = v;
                nBytesLeft = v;
                state = DynamixelParserState_Length;
                break;
            case DynamixelParserState_Length:
                //uartDebug->write((const uint8_t*)"-LN-",4);
                error  = v;
                nBytesLeft--;
                state = DynamixelParserState_Error;
                break;
            case DynamixelParserState_Error:
                //uartDebug->write((const uint8_t*)"-ER-",4);
                payload[length - nBytesLeft - 1] = v;
                nBytesLeft--;
                if (nBytesLeft <= 0){
                    state = DynamixelParserState_Payload;
                }
                break;
            case DynamixelParserState_Payload:
                //uartDebug->write((const uint8_t*)"-PL-",4);
                // We have recieved full message, this is crc;
                crc = v;
                execute();
                state = DynamixelParserState_None;
                break;
            case DynamixelParserState_CRC:
                //uartDebug->write((const uint8_t*)"-CR-",4);
                state = DynamixelParserState_None;
                break;
            case DynamixelParserState_Max:
                //uartDebug->write((const uint8_t*)"-MX-",4);
                state = DynamixelParserState_None;
                break;
            default:
                //uartDebug->write((const uint8_t*)"-DF-",4);
                state = DynamixelParserState_None;
                break;
        }
    }
}

void DynamixelParser::execute(){
    uint8_t low, high;
    /*
    if (!is_any_present_value_pending){
        return;
    }
    */
    low = payload[0];
    high= payload[1];
    if (id == TILT_ID){
        tilt_value = high;
        tilt_value <<= 8;
        tilt_value += low;
        tilt_value_received();
        if (uartDebug != nullptr){
            //uartDebug->printf("tilt: %i\n", tilt_value);
        }
    } else if (id == PAN_ID){
        pan_value = high;
        pan_value <<= 8;
        pan_value += low;
        pan_value_received();
        if (uartDebug != nullptr){
            //uartDebug->printf("pan: %i\n", pan_value);
        }
    }
}


void DynamixelParser::send_read_position_command(){
    uint8_t command [] = { 
        0xff, 0xff, 0xFE , 0x09 , 0x92,   // sync1,2; broadcast, length total, instruction
        0x00,                             // Begin
        0x02, 0x01, 0x24,                 // length: 2, address 1, messge: 0x24 (present position)
        0x02, 0x02, 0x24,                 // length: 2, address 2, messge: 0x24 (present position)
        0x0 };
    if (inputUart == nullptr)
        return;
    command[12]  = compute_checksum(command + 2, 10);
    inputUart->write( (const uint8_t*) command , 13);
}


