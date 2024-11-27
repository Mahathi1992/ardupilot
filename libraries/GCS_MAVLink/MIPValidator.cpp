#include "MIPValidator.h"

MIPValidator::MIPValidator(){
    reset();
}

void MIPValidator::reset(){
    int i =0;
    wpCount = 0;
    numWpReceived = 0;
    isFailed = false;
    isComplete = false;
    isValid = false;
    for(i = 0; i < MAX_WAYPOINTS_ZAS; i++){
        wpCheck[i] = 0;
    }
}

void MIPValidator::touch(int itemid){
    if (itemid >= wpCount){
        isFailed = true;
        return;
    }
    wpCheck[itemid] = 1;
    numWpReceived++;
}

bool MIPValidator::status(){
    return isValid;
}

void MIPValidator::validate(){
    int i = 0;
    isValid = false;
    if (wpCount >= MAX_WAYPOINTS_ZAS)
        return;
    if (numWpReceived > wpCount)
        return;
    for (i = 0; i < numWpReceived; i++){
        if (wpCheck[i] != 1)
            return;
    }
    if (isFailed)
        return;
    isValid  = true;
}

bool MIPValidator::getCompletionStatus(){
    if (wpCount != numWpReceived)
        return false;
    validate();
    isComplete = isValid;
    return isValid;
}

void MIPValidator::setMissionWaypoints(int maxWp){
    if (maxWp >= MAX_WAYPOINTS_ZAS)
        return;
    wpCount = maxWp;
}