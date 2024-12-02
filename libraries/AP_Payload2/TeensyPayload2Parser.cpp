#include  <AP_Payload2/TeensyPayload2Parser.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Payload2/AP_Payload2_Control.h>

void send_payload2_message();

void send_payload2_message() {
    // Get the message.
    Teensy2Message message = AP::payload2_control()->get_teensy_message();
    static mavlink_zmotion_payload2_response_t response;
    
    response.seq_identifier = 0;
    response.zmotion_cmd_type = PAYLOAD2_COMMANDS;
    response.actual_cmd = PAYLOAD2_UNSED;
    response.response1 = message.chassisID;
    response.response2 = message.chassisHealth;
    response.response3 = message.pid;
    response.response4 = message.msg;
    
    
    GCS_MAVLINK *link = gcs().chan(0);
    if (link != nullptr){
        mavlink_msg_zmotion_payload2_response_send_struct(link->get_chan(), &response);
    }
    
    if (AP::payload2_control()->isChannelSet){
        mavlink_msg_zmotion_payload2_response_send_struct(AP::payload2_control()->chan, &response);
    }
}

uint16_t Teensy2Message::checksum_payload() {
    uint16_t accumulator = 0;
    accumulator = header + pid + msg + footer;
    return accumulator;
}

void Teensy2Message::prepare_payload() {
    uint16_t chksum;
    payload[0] = header;
    payload[1] = chassisID;
    payload[2] = chassisHealth;
    payload[3] = pid;
    payload[4] = msg;
    payload[5] = footer;
    chksum = checksum_payload();
    payload[6] = chksum>>8;
    payload[7] = chksum&0x00ff;
}

bool Teensy2Message::verify_checksum() {
    uint16_t chksum = checksum_payload();
    checksum = payload[6];
    checksum <<= 8;
    checksum += payload[7];
    // gcs().send_text(MAV_SEVERITY_INFO, "RCVC: %i, c: %i , ac: %i" , payload[7], chksum, checksum);
    return (chksum == checksum);
}

TeensyPayload2Parser::TeensyPayload2Parser():HEADER(0xff), FOOTER(0xFE) {
    rb = 0;
    wb = 0;
    state = Teensy2MessageState_None;
}

void TeensyPayload2Parser::execute() {
    gcs().send_text(MAV_SEVERITY_WARNING, "RCV MSG: %i,%i,%i,%i" , incoming.payload[0], incoming.payload[1], incoming.payload[2], incoming.payload[3]);
    send_payload2_message();
}

void TeensyPayload2Parser::stateMachine() {
    uint8_t v;
    while (rb != wb) {
        v = buffer[rb++];
        switch (state) {
        case Teensy2MessageState_None:
            if (v == HEADER) {
                gcs().send_text(MAV_SEVERITY_INFO, "Header: %i" , v);
                state = Teensy2MessageState_CID;
                incoming.header = HEADER;
            }
            break;
        case Teensy2MessageState_CID:
            incoming.chassisID = v;
            state = Teensy2MessageState_C_Health;
            gcs().send_text(MAV_SEVERITY_INFO, "Chassis ID: %i" , v);
            break;
        case Teensy2MessageState_C_Health:
            incoming.chassisHealth = v;
            state = Teensy2MessageState_PID;
            gcs().send_text(MAV_SEVERITY_INFO, "Chassis Health: %i" , v);
            break;
        case Teensy2MessageState_PID:
            incoming.pid = v;
            state = Teensy2MessageState_MSG;
            gcs().send_text(MAV_SEVERITY_INFO, "Payload ID: %i" , v);
            break;
        case Teensy2MessageState_MSG:
            incoming.msg = v;
            state = Teensy2MessageState_Footer;
            gcs().send_text(MAV_SEVERITY_INFO, "Payload MSG: %i" , v);
            break;
        case Teensy2MessageState_Footer:
            incoming.footer = v;
            if (v == FOOTER) {
                state = Teensy2MessageState_Checksum;
                gcs().send_text(MAV_SEVERITY_INFO, "Footer: %i" , v);
            } else {
                state = Teensy2MessageState_None;
                // gcs().send_text(MAV_SEVERITY_INFO, "RCVM2: %i" , v);
            }
            break;
        case Teensy2MessageState_Checksum:
            incoming.payload[6] = v;
            state = Teensy2MessageState_Checksum_Last;
            // gcs().send_text(MAV_SEVERITY_INFO, "RCVF: %i" , v);
            break;
        case Teensy2MessageState_Checksum_Last:
            incoming.payload[7] = v;
            if (incoming.verify_checksum()) {
                // checksum matched. Do something
                execute();
            }
            state = Teensy2MessageState_None;
            break;
        case Teensy2MessageState_Max:
        default:
            state = Teensy2MessageState_None;
            rb--;
            break;
        }
    }
}