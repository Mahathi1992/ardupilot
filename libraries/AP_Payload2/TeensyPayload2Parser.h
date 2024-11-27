#pragma once

#include <stdint.h>

#define BUF_SIZE_VAL 0x100

struct Teensy2Message {
    uint8_t header, pid, msg, footer;
    uint16_t checksum;
    uint8_t payload[6];
    void prepare_payload();
    bool verify_checksum();
    uint16_t checksum_payload();
};

enum Teensy2MessageState_t { Teensy2MessageState_None = 0, Teensy2MessageState_Header, Teensy2MessageState_PID, Teensy2MessageState_MSG, Teensy2MessageState_Footer, Teensy2MessageState_Checksum, Teensy2MessageState_Max};

class TeensyPayload2Parser {
public:
    TeensyPayload2Parser();
    void parse();
    void stateMachine();
    Teensy2Message & getTeensy2Message(){ return incoming; }
protected:
    void execute();
public:
    uint8_t buffer[BUF_SIZE_VAL];
    uint8_t wb, rb;
protected:
    enum Teensy2MessageState_t state;
    const uint8_t HEADER, FOOTER;
    Teensy2Message incoming;
};