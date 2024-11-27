#include "AP_Mount_Servo.h"
#include "TrackerStabiliser.h"
#include <GCS_MAVLink/GCS.h>


//extern const AP_HAL::HAL& hal;
extern AP_HAL::UARTDriver *uart1Debug;

void SightLineStabilise::beginTracking(){
    pan_correction = 0; tilt_correction = 0;
    isCurrentlyTracking = true;
    missed_tracking = 0;
    last_pan_step = 0, last_tilt_step = 0;
}

void SightLineStabilise::finishTracking(){
    pan_correction = 0; tilt_correction = 0;
    isCurrentlyTracking = false;
    missed_tracking = 0;
}

void SightLineStabilise::feed(int16_t x, int16_t y,int16_t wx,int16_t wy,int16_t confidence){
    if (confidence > confidence_threshold){
        if (!isCurrentlyTracking){
            beginTracking();
        }
        cx = x + (wx/2);
        cy = y + (wy/2);
    } else {
        emptyfeed();
        return;
    }
    pan_correction += getPanCorrection();
    tilt_correction += getTiltCorrection();
}

void SightLineStabilise::setOffsets(int16_t pan_offset_value, int16_t tilt_offset_value){
    offset_pan = pan_offset_value; offset_tilt = tilt_offset_value;
}

void SightLineStabilise::emptyfeed(){
    missed_tracking++;
    checkIsTracking();
}

void SightLineStabilise::checkIsTracking(){
    if (missed_tracking > max_misses_before_tracking_lost){
        finishTracking();
    }
}

int16_t SightLineStabilise::getPanCorrection(){
    int16_t delta = cx - half_width;
    int16_t currentStep = 0;
    if (delta < 0){
        // We need to move Right;
        currentStep = (-1*step_value);
    } else {
        currentStep = step_value;
    }
    
    if (abs(delta) < center_offset_pixel_delta){
        /// We are very nearly at the center.
        return 0;
    }
    return currentStep;
}

int16_t SightLineStabilise::getTiltCorrection(){
    int16_t delta = cy - half_height;
    int16_t currentStep = 0;
    if (delta < 0){
        // We need to move Right;
        currentStep = step_value;
    } else {
        currentStep = (-1*step_value);
    }
    
    if (abs(delta) < center_offset_pixel_delta){
        /// We are very nearly at the center.
        return 0;
    }
    return currentStep;
}

double SightLineStabilise::getBearing(){
    int16_t correction = pan_correction;
    float m = -11.3;
    float c = 2048 + offset_pan;
    float  ymc = correction - c;
    float x = ymc/m;
    return x;
}

double SightLineStabilise::getTiltInDegrees(){
    int16_t correction = tilt_correction;
    float m = -11.3;
    float c = 2048 + offset_tilt;
    float  ymc = correction - c;
    float x = ymc/m;
    return x;
}

void SightLineStabilise::setTrackParameters(mavlink_zas_track_parameters_t *params){
    this->zas_track_parameters = params;
    center_offset_pixel_delta = params->center_offset_pixel_delta;
    step_value = params->step_value;
    max_misses_before_tracking_lost = params->max_misses_before_tracking_lost;
    confidence_threshold = params->confidence_threshold;
    cam0_h = params->cam0_height;
    cam0_w = params->cam0_width;
    cam1_h = params->cam1_height;
    cam1_w = params->cam1_width;
    if (uart1Debug != nullptr){
      uart1Debug->printf("Apply4: Max misses: %i, Step Value: %i, Confidence Threshold: %i, Center Offset: %i, Cam0 W: %i, H: %i, Cam1 W: %i, Cam1 H: %i \n",
	      this->max_misses_before_tracking_lost, this->step_value , this->confidence_threshold , this->center_offset_pixel_delta , 
	      cam0_w, cam0_h, cam1_w, cam1_h
	  );
    }
}

