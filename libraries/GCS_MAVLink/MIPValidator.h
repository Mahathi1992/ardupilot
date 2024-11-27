#pragma once
#include <stdint.h>

#define MAX_WAYPOINTS_ZAS (0x100)

class MIPValidator {
public:
    MIPValidator();
    void reset();
    void touch(int itemid);
    bool status();
    void validate();
    bool getCompletionStatus();
    void setMissionWaypoints(int maxWp);
public:
    int wpCount;
    int numWpReceived;
protected:
    uint8_t wpCheck[MAX_WAYPOINTS_ZAS];
    bool isFailed, isComplete, isValid;
};