#pragma once

#include "AP_Mount_Servo.h"

#define CONFIDENCE_THRESHOLD 50

#define CAMERA_WIDTH 1920
#define CAMERA_HEIGHT 1080


typedef struct {
  uint8_t cameraIndex;
  uint8_t numTracks;
  struct {
    uint8_t index;
    int16_t col;
    int16_t row;
    int16_t wide;
    int16_t high;
    int16_t velCol8;
    int16_t velRow8;
    uint8_t confidence;
    uint8_t flags;
    uint8_t obstructed, isPrimary, isSelected;
  } track[5];
  uint64_t timeStamp;
  uint32_t frameId;
} SLATrackingPositions_t;

typedef struct {
  int16_t trackingCol;
  int16_t trackingRow;
  int16_t sceneCol;
  int16_t sceneRow;
  int16_t offsetCol;
  int16_t offsetRow;
  uint8_t trackingConfidence;
  uint8_t isUpdated;
} SLATrackingPosition_t;


enum SLTrackerPackets_t {SLTrackerPackets_None = 0, SLTrackerPackets_TrackingPosition = 0x43, SLTrackerPackets_TrackingPositions = 0x51, SLTrackerPackets_Max = 0xfe};
enum SLTrackerStates_t { SLTrackerStates_None = 0, SLTrackerStates_Sync1 , SLTrackerStates_Sync2, SLTrackerStates_LengthLow, SLTrackerStates_LengthHigh, SLTrackerStates_Payload, SLTrackerStates_Execute , SLTrackerStates_Max };

class ParserSightLine;

typedef void (ParserSightLine::*functionList_t )(uint8_t *packetpayload, uint16_t length);

#define PARSE_SIGHT_LINE_BUFFER 1024

class ParserSightLine {
    const uint8_t SYNC1, SYNC2;
public:
    ParserSightLine();
    void reset(){
        wb = 0; rb = 0;
    }
    void stateMachine();
    void execute();
public:
    uint8_t buffer[PARSE_SIGHT_LINE_BUFFER], payload[0x100];
    uint16_t wb , rb;
    uint8_t header;
    uint16_t packetLength;
    uint16_t bytesLeft;
    functionList_t functions[SLTrackerPackets_Max];
    enum SLTrackerStates_t state, prevState;
    AP_HAL::UARTDriver *uartDebug;
    const uint16_t BUF_SIZE;
    int16_t c_track_x, c_track_y, c_to_track, track_v_width, track_v_height, c_track_update;
    SLATrackingPosition_t position43;
    class SightLineStabilise *stabiliser;
public:
    void trackingPositions(uint8_t *packetpayload, uint16_t length);
    void trackingPosition43(uint8_t *packetpayload, uint16_t length);
};

void applyTrackParametersUpdate(mavlink_zas_track_parameters_t *params);
