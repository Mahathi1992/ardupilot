#pragma once

#define MAX_MISSES_BEFORE_TRACKING_LOST 10
#define STEP_VALUE 2
#define CENTER_OFFSET_PIXEL_DELTA 50

class SightLineStabilise {
public:
    SightLineStabilise(int32_t camera_width, int32_t camera_height, int16_t confidence_threshold_limit){
        half_width = camera_width/2;
        half_height = camera_height/2;
        this->confidence_threshold = confidence_threshold_limit;
        center_offset_pixel_delta = CENTER_OFFSET_PIXEL_DELTA;
        step_value = STEP_VALUE;
        max_misses_before_tracking_lost = MAX_MISSES_BEFORE_TRACKING_LOST;
        offset_pan = 0; offset_tilt = 0;
    }
    void feed(int16_t x,int16_t y,int16_t wx,int16_t wy,int16_t confidence);
    void emptyfeed();
    void checkIsTracking();
    double getBearing();
    double getTiltInDegrees();
    void setTrackParameters(mavlink_zas_track_parameters_t *params);
    void setOffsets(int16_t pan_offset_value, int16_t tilt_offset_value);
protected:
    void beginTracking();
    void finishTracking();
    int16_t getPanCorrection();
    int16_t getTiltCorrection();
public:
    uint16_t max_misses_before_tracking_lost, center_offset_pixel_delta;
    uint16_t step_value;
    uint16_t cam0_h, cam0_w, cam1_h, cam1_w;
public:
    bool isCurrentlyTracking;
    int16_t half_height, half_width, cx, cy;
    int16_t isPrimary, confidence_threshold;
    int16_t pan_correction, tilt_correction;
    int16_t last_pan_step, last_tilt_step;
    int16_t missed_tracking;
    int16_t offset_pan, offset_tilt;
    mavlink_zas_track_parameters_t *zas_track_parameters;
};


struct Coordinate {
  double X; //longitudeInDegrees;
  double Y; //latitudeInDegrees;
  double Z; //altitudeInMeters;
};
