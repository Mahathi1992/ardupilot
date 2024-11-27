#include "AP_Mount_Servo.h"
#include "SightLine.h"
#include "TrackerStabiliser.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;
extern AP_HAL::UARTDriver *uart2Tracker, *uart1Debug;
static mavlink_zas_track_parameters_t zas_track_parameters;

void set_zas_track_parameters(mavlink_zas_track_parameters_t *params){
    mavlink_zas_track_parameters_t *packet = &zas_track_parameters;
    packet->max_misses_before_tracking_lost = params->max_misses_before_tracking_lost;
    packet->step_value = params->step_value;
    packet->confidence_threshold = params->confidence_threshold;
    packet->center_offset_pixel_delta = params->center_offset_pixel_delta;
    packet->cam0_height = params->cam0_height;
    packet->cam0_width = params->cam0_width;
    packet->cam1_height = params->cam1_height;
    packet->cam1_width = params->cam1_width;
    
    /*
    hal.serial(2)->printf("Setting1: Max misses: %li, Step Value: %li, Confidence Threshold: %i, Center Offset: %i, Cam0 W: %i, H: %i, Cam1 W: %i, Cam1 H: %i \n",
            packet->max_misses_before_tracking_lost, packet->step_value , packet->confidence_threshold , packet->center_offset_pixel_delta , 
            packet->cam0_height, packet->cam0_width, packet->cam1_height, packet->cam1_width
        );
    
    
    hal.serial(2)->printf("Setting2: Max misses: %li, Step Value: %li, Confidence Threshold: %i, Center Offset: %i, Cam0 W: %i, H: %i, Cam1 W: %i, Cam1 H: %i \n",
            params->max_misses_before_tracking_lost, params->step_value , params->confidence_threshold , params->center_offset_pixel_delta , 
            params->cam0_height, params->cam0_width, params->cam1_height, params->cam1_width
        );
        */
    
    applyTrackParametersUpdate(params);
}

static const uint8_t crc8_Table[ ] = {
     0,  94, 188, 226,  97,  63, 221, 131, 194, 156, 126,  32, 163, 253,  31,  65,
   157, 195,  33, 127, 252, 162,  64,  30,  95,   1, 227, 189,  62,  96, 130, 220,
    35, 125, 159, 193,  66,  28, 254, 160, 225, 191,  93,   3, 128, 222,  60,  98,
   190, 224,   2,  92, 223, 129,  99,  61, 124,  34, 192, 158,  29,  67, 161, 255,
    70,  24, 250, 164,  39, 121, 155, 197, 132, 218,  56, 102, 229, 187,  89,   7,
   219, 133, 103,  57, 186, 228,   6,  88,  25,  71, 165, 251, 120,  38, 196, 154,
   101,  59, 217, 135,   4,  90, 184, 230, 167, 249,  27,  69, 198, 152, 122,  36,
   248, 166,  68,  26, 153, 199,  37, 123,  58, 100, 134, 216,  91,   5, 231, 185,
   140, 210,  48, 110, 237, 179,  81,  15,  78,  16, 242, 172,  47, 113, 147, 205,
    17,  79, 173, 243, 112,  46, 204, 146, 211, 141, 111,  49, 178, 236,  14,  80,
   175, 241,  19,  77, 206, 144, 114,  44, 109,  51, 209, 143,  12,  82, 176, 238,
    50, 108, 142, 208,  83,  13, 239, 177, 240, 174,  76,  18, 145, 207,  45, 115,
   202, 148, 118,  40, 171, 245,  23,  73,   8,  86, 180, 234, 105,  55, 213, 139,
    87,   9, 235, 181,  54, 104, 138, 212, 149, 203,  41, 119, 244, 170,  72,  22,
   233, 183,  85,  11, 136, 214,  52, 106,  43, 117, 151, 201,  74,  20, 246, 168,
   116,  42, 200, 150,  21,  75, 169, 247, 182, 232,  10,  84, 215, 137, 107,  53
};

static uint8_t crcSightLine(uint8_t *payload, uint8_t header, uint16_t length){
    uint8_t crc = 0x01;
    uint16_t i = 0;
    crc = crc ^ header;
    crc = crc8_Table[crc];
    while (i < length){
        crc = crc ^ payload[i];
        crc = crc8_Table[crc];
        i++;
    }
    return crc;
}

static uint16_t join16SL(uint8_t h, uint8_t l){
    uint16_t v = l;
    v <<= 8;
    v += h;
    return v;
}

ParserSightLine::ParserSightLine(): SYNC1(0x51), SYNC2(0xAC), BUF_SIZE(PARSE_SIGHT_LINE_BUFFER){
    wb = 0; rb = 0;
    for (int i = 0; i < SLTrackerPackets_Max; i++){
        functions[i] = nullptr;
        //lengths[i] = 0;
    }
    functions[SLTrackerPackets_TrackingPositions] = &ParserSightLine::trackingPositions;
    //functions[SLTrackerPackets_TrackingPosition] = &ParserSightLine::trackingPosition43;
    //lengths[SLTrackerPackets_TrackingPositions] = 0xff;
    state = SLTrackerStates_None;
    prevState = state;
    uartDebug = nullptr;
    
    track_v_height = CAMERA_HEIGHT;
    track_v_width = CAMERA_WIDTH;
    c_track_update = 0;
    stabiliser = new SightLineStabilise(CAMERA_WIDTH, CAMERA_HEIGHT, CONFIDENCE_THRESHOLD);
}


void ParserSightLine::trackingPosition43(uint8_t *packetpayload, uint16_t length){
    if (length == 0)
        return;
    if (packetpayload == nullptr)
        return;
    position43.trackingCol = (int16_t)join16SL(packetpayload[0],packetpayload[1]);
    position43.trackingRow = (int16_t)join16SL(packetpayload[2],packetpayload[3]);
    position43.sceneCol = (int16_t)join16SL(packetpayload[4],packetpayload[5]);
    position43.sceneRow = (int16_t)join16SL(packetpayload[6],packetpayload[7]);
    position43.isUpdated = 0;
    if (position43.isUpdated == 0){
        position43.isUpdated = 1;
    }
}

void ParserSightLine::trackingPositions(uint8_t *packetpayload, uint16_t length){
    SLATrackingPositions_t positions;
    if (length == 0)
        return;
    if (packetpayload == nullptr)
        return;
    positions.cameraIndex = packetpayload[0];
    positions.numTracks = packetpayload[1];
    
    if (length < (15* packetpayload[1])){
        return;
    }
    
    uint8_t tracks = 0;
    uint16_t delta = 0;
    while (tracks < positions.numTracks){
        delta = (tracks* 15);
        positions.track[tracks].index = packetpayload[2 + delta];
        positions.track[tracks].col = (int16_t)join16SL(packetpayload[2 + delta +1] , packetpayload[2 + delta +2]);
        positions.track[tracks].row = (int16_t)join16SL(packetpayload[2 + delta +3] , packetpayload[2 + delta +4]);
        positions.track[tracks].wide = (int16_t)join16SL(packetpayload[2 + delta +5] , packetpayload[2 + delta +6]);
        positions.track[tracks].high = (int16_t)join16SL(packetpayload[2 + delta +7] , packetpayload[2 + delta +8]);
        positions.track[tracks].velCol8 = (int16_t)join16SL(packetpayload[2 + delta +9] , packetpayload[2 + delta +10]);
        positions.track[tracks].velRow8 = (int16_t)join16SL(packetpayload[2 + delta +11] , packetpayload[2 + delta +12]);
        positions.track[tracks].confidence = packetpayload[2 + delta + 13];
        positions.track[tracks].flags = packetpayload[2 + delta  + 14];
        positions.track[tracks].obstructed = positions.track[tracks].confidence & 0x80;
        positions.track[tracks].confidence = positions.track[tracks].confidence & ~0x80;
        positions.track[tracks].isPrimary = positions.track[tracks].flags & 0x1;
        positions.track[tracks].isSelected = positions.track[tracks].flags & 0x3;
        
        if (positions.track[tracks].isPrimary){
            c_track_x = positions.track[tracks].col + (positions.track[tracks].wide/2);
            c_track_y = positions.track[tracks].row + (positions.track[tracks].high/2);
            c_to_track = (positions.track[tracks].confidence > CONFIDENCE_THRESHOLD)?1:0;
            c_track_update = 1;
        }
        
        if (uartDebug != nullptr){
            uartDebug->printf("[%i] C: %i, R: %i , W: %i , H: %i , C: %i , P: %i, S: %i\r\n", tracks ,
                              positions.track[tracks].col, positions.track[tracks].row, positions.track[tracks].wide, positions.track[tracks].high,
                              positions.track[tracks].confidence , positions.track[tracks].isPrimary ,positions.track[tracks].isSelected);
        }
        
        if (positions.track[tracks].isPrimary){
            stabiliser->feed(positions.track[tracks].col, positions.track[tracks].row, positions.track[tracks].wide, positions.track[tracks].high, positions.track[tracks].confidence);
        }
        
        //AP_HAL::UARTDriver *uart1 = hal.serial(1);
        //uart1->begin(57600);
        //uart1Debug = uart1;
        if (uart1Debug != nullptr){
            uart1Debug->printf("[%i] C: %i, R: %i , W: %i , H: %i , C: %i , P: %i, S: %i\r\n", tracks ,
                              positions.track[tracks].col, positions.track[tracks].row, positions.track[tracks].wide, positions.track[tracks].high,
                              positions.track[tracks].confidence , positions.track[tracks].isPrimary ,positions.track[tracks].isSelected);
        }
        tracks++;
    }
}

void ParserSightLine::execute(){
    if (header >= SLTrackerPackets_Max)
        return;
    if (functions[header] == nullptr)
        return;
    (this->*functions[header])(payload, packetLength);
}

void ParserSightLine::stateMachine(){
    uint8_t v;
    while (rb != wb){
        v = buffer[rb++];
        rb = (rb >= BUF_SIZE)?0:rb;
        
//         if (parser.uartDebug != nullptr){
//             parser.uartDebug->write(&v, 1);
//         }
        switch (state){
            case SLTrackerStates_None:
                if (v == SYNC1){
                    state = SLTrackerStates_Sync1;
                }
                break;
            case  SLTrackerStates_Sync1: 
                if (v == SYNC2){
                    state = SLTrackerStates_Sync2;
                } else {
                    state = SLTrackerStates_None;
                }
                break;
            case SLTrackerStates_Sync2: 
                packetLength = v;
                if ( (v & 0x80) ){
                    state = SLTrackerStates_LengthLow;
                } else {
                    state = SLTrackerStates_LengthHigh;
                    packetLength -= 2;
                }
                break;
            case SLTrackerStates_LengthLow:
            {
                packetLength = (packetLength & ~0xff80);
                uint16_t tempLength = v;
                tempLength = v << 7;
                packetLength = (packetLength | tempLength);
                packetLength -= 2;
                state = SLTrackerStates_LengthHigh;
            }
                break;
            case SLTrackerStates_LengthHigh:
                bytesLeft = packetLength;
                header = v;
                state = SLTrackerStates_Payload;
                if (uartDebug != nullptr){
                    uartDebug->printf("id: %i, len: %i\r\n", header, packetLength);
                }
                if ((header == 0xff) || (packetLength > 512)){
                    if (uartDebug != nullptr){
                        uartDebug->printf("Rejected: id: %i, len: %i\r\n", header, packetLength);
                    }
                    state = SLTrackerStates_None;
                }
                break;
            case  SLTrackerStates_Payload:
                if (packetLength == 0){
                    execute();
                    state = SLTrackerStates_Execute;
/*                    
                    if (parser.uartDebug != nullptr){
                        parser.uartDebug->write((uint8_t*)"-P-", 3);
                    }*/
                    break;
                }
                payload[packetLength - bytesLeft] = v;
                bytesLeft--;
                if (bytesLeft == 0){
                    execute();
                    state = SLTrackerStates_Execute;
                }/*
                if (parser.uartDebug != nullptr){
                    parser.uartDebug->write((uint8_t*)"-p-", 3);
                }*/
                break;
            case  SLTrackerStates_Execute:
                // Calculate CRC
            {
                uint8_t crc = crcSightLine(payload, header, packetLength);
                if (uartDebug != nullptr){
                    if (crc != v){
                        uartDebug->printf("CRC FAIL: [e]: 0x%X , [r]: 0x%X\n\r", crc, v);
                    } else {
                        uartDebug->printf("CRC pass:0x%X\n\r", crc);
                    }
                }
            }
                state = SLTrackerStates_None;
                break;
            case  SLTrackerStates_Max:
                state = SLTrackerStates_None;
                if (uartDebug != nullptr){
                    uartDebug->write((uint8_t*)"-M-", 3);
                }
                break;
        }
    }
}