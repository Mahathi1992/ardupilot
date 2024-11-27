#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Common/Location.h>
#include <stdint.h>

class Mode
{
public:

    /* Do not allow copies */
    Mode(const Mode &other) = delete;
    Mode &operator=(const Mode&) = delete;

    // Auto Pilot modes
    // ----------------
    enum Number : uint8_t {
        MANUAL        = 0,
        CIRCLE        = 1,
        STABILIZE     = 2,
        TRAINING      = 3,
        ACRO          = 4,
        FLY_BY_WIRE_A = 5,
        FLY_BY_WIRE_B = 6,
        CRUISE        = 7,
        AUTOTUNE      = 8,
        AUTO          = 10,
        RTL           = 11,
        LOITER        = 12,
        TAKEOFF       = 13,
        AVOID_ADSB    = 14,
        GUIDED        = 15,
        INITIALISING  = 16,
        QSTABILIZE    = 17,
        QHOVER        = 18,
        QLOITER       = 19,
        QLAND         = 20,
        QRTL          = 21,
        QAUTOTUNE     = 22,
        QACRO         = 23,
        GIMBALNAV     = 26,
        TRACK2D      = 27,
        TRACK3D      = 28,
        GPSDIVE     = 29,
        VBTG        = 30,
    };

    // Constructor
    Mode();

    // enter this mode, always returns true/success
    bool enter();

    // perform any cleanups required:
    void exit();

    // returns a unique number specific to this mode
    virtual Number mode_number() const = 0;

    // returns full text name
    virtual const char *name() const = 0;

    // returns a string for this flightmode, exactly 4 bytes
    virtual const char *name4() const = 0;

    //
    // methods that sub classes should override to affect movement of the vehicle in this mode
    //

    // convert user input to targets, implement high level control for this mode
    virtual void update() = 0;

    // true for all q modes
    virtual bool is_vtol_mode() const { return false; }

protected:

    // subclasses override this to perform checks before entering the mode
    virtual bool _enter() { return true; }

    // subclasses override this to perform any required cleanup when exiting the mode
    virtual void _exit() { return; }
};


class ModeAcro : public Mode
{
public:

    Mode::Number mode_number() const override { return Mode::Number::ACRO; }
    const char *name() const override { return "ACRO"; }
    const char *name4() const override { return "ACRO"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeAuto : public Mode
{
public:

    Number mode_number() const override { return Number::AUTO; }
    const char *name() const override { return "AUTO"; }
    const char *name4() const override { return "AUTO"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
    void _exit() override;
};


class ModeAutoTune : public Mode
{
public:

    Number mode_number() const override { return Number::AUTOTUNE; }
    const char *name() const override { return "AUTOTUNE"; }
    const char *name4() const override { return "ATUN"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
    void _exit() override;
};

class ModeGuided : public Mode
{
public:

    Number mode_number() const override { return Number::GUIDED; }
    const char *name() const override { return "GUIDED"; }
    const char *name4() const override { return "GUID"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};


class ModeTrack2D : public Mode {
public:

    Number mode_number() const override { return Number::TRACK2D; }
    const char *name() const override { return "TRACK2D"; }
    const char *name4() const override { return "TR2D"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate();

    virtual bool is_guided_mode() const { return true; }

    bool allows_throttle_nudging() const{ return true; }

    bool does_auto_navigation() const { return true; }

    bool does_auto_throttle() const { return false; }

    // handle a guided target request from GCS
    bool handle_guided_request(Location target_loc) ;
    
    static double ToDegree(double radian);
    static double ToRadian(double degree);
    static void GetTarget(struct Coordinate origin, double bearing, double distance, double altitude, struct Coordinate *result);

protected:

    bool _enter() override;
    void calculate_track_endpoint();
};



class ModeTrack3D : public Mode {
public:

    Number mode_number() const override { return Number::TRACK3D; }
    const char *name() const override { return "TRACK3D"; }
    const char *name4() const override { return "TR3D"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() ;

    virtual bool is_guided_mode() const  { return true; }

    bool allows_throttle_nudging() const  { return true; }

    bool does_auto_navigation() const  { return true; }

    bool does_auto_throttle() const  { return false; }

    // handle a guided target request from GCS
    bool handle_guided_request(Location target_loc) ;

protected:

    bool _enter() override;
public:
    
    enum State_Mode3_t { State_Mode3_None = 0, State_Mode3_Trackpoint1 , State_Mode3_DIVE};
    Location diveLocation;
    void fillDiveLocation();
    int32_t forced_pitch_angle;
    enum  ModeTrack3D::State_Mode3_t mode3_state;
};

class ModeGimabalNav : public Mode {
public:

    Number mode_number() const override { return Number::GIMBALNAV; }
    const char *name() const override { return "GIMBALNAV"; }
    const char *name4() const override { return "GNAV"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() ;

    virtual bool is_guided_mode() const  { return true; }

    bool allows_throttle_nudging() const  { return true; }

    bool does_auto_navigation() const  { return true; }

    bool does_auto_throttle() const  { return false; }

    // handle a guided target request from GCS
    bool handle_guided_request(Location target_loc) ;

protected:

    bool _enter() override;
};

class ModeGPSDive : public Mode {
public:

    Number mode_number() const override { return Number::GPSDIVE; }
    const char *name() const override { return "GPSDive"; }
    const char *name4() const override { return "GPSD"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() ;

    virtual bool is_guided_mode() const  { return true; }

    bool allows_throttle_nudging() const  { return true; }

    bool does_auto_navigation() const  { return true; }

    bool does_auto_throttle() const  { return false; }

    // handle a guided target request from GCS
    bool handle_guided_request(Location target_loc) ;
    bool checkifTargetHeadingIsCorrect();
    bool isPlaneInDive();
    void do_make_land();
    
    enum State_ModeGPSD_t { State_ModeGPSD_None = 0, State_ModeGPSD_Approach_Alt, State_ModeGPSD_Hdg , State_ModeGPSD_DIVE, State_ModeGPSD_Abort};

    uint16_t target_heading;
    uint8_t loiter_direction;
    int16_t dive_pitch_saturation, pitch_offset;
    int32_t commanded_pitch;
protected:
    Location diveLocation;
    void fillDiveLocation();
    void update_dive();
    bool _enter() override;
    enum  ModeGPSDive::State_ModeGPSD_t mode3_state;
};

class ModeVBGT : public Mode {
public:

    Number mode_number() const override { return Number::VBTG; }
    const char *name() const override { return "VideoTerminalGuidance"; }
    const char *name4() const override { return "VBGT"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() ;

    virtual bool is_guided_mode() const  { return true; }

    bool allows_throttle_nudging() const  { return true; }

    bool does_auto_navigation() const  { return true; }

    bool does_auto_throttle() const  { return false; }

    // handle a guided target request from GCS
    bool handle_guided_request(Location target_loc) ;

protected:

    bool _enter() override;
};



class ModeCircle: public Mode
{
public:

    Number mode_number() const override { return Number::CIRCLE; }
    const char *name() const override { return "CIRCLE"; }
    const char *name4() const override { return "CIRC"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeLoiter : public Mode
{
public:

    Number mode_number() const override { return Number::LOITER; }
    const char *name() const override { return "LOITER"; }
    const char *name4() const override { return "LOIT"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeManual : public Mode
{
public:

    Number mode_number() const override { return Number::MANUAL; }
    const char *name() const override { return "MANUAL"; }
    const char *name4() const override { return "MANU"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
    void _exit() override;
};


class ModeRTL : public Mode
{
public:

    Number mode_number() const override { return Number::RTL; }
    const char *name() const override { return "RTL"; }
    const char *name4() const override { return "RTL "; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeStabilize : public Mode
{
public:

    Number mode_number() const override { return Number::STABILIZE; }
    const char *name() const override { return "STABILIZE"; }
    const char *name4() const override { return "STAB"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeTraining : public Mode
{
public:

    Number mode_number() const override { return Number::TRAINING; }
    const char *name() const override { return "TRAINING"; }
    const char *name4() const override { return "TRAN"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeInitializing : public Mode
{
public:

    Number mode_number() const override { return Number::INITIALISING; }
    const char *name() const override { return "INITIALISING"; }
    const char *name4() const override { return "INIT"; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

protected:

    bool _enter() override;
};

class ModeFBWA : public Mode
{
public:

    Number mode_number() const override { return Number::FLY_BY_WIRE_A; }
    const char *name() const override { return "FLY_BY_WIRE_A"; }
    const char *name4() const override { return "FBWA"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    bool _enter() override;

protected:

};

class ModeFBWB : public Mode
{
public:

    Number mode_number() const override { return Number::FLY_BY_WIRE_B; }
    const char *name() const override { return "FLY_BY_WIRE_B"; }
    const char *name4() const override { return "FBWB"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeCruise : public Mode
{
public:

    Number mode_number() const override { return Number::CRUISE; }
    const char *name() const override { return "CRUISE"; }
    const char *name4() const override { return "CRUS"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeAvoidADSB : public Mode
{
public:

    Number mode_number() const override { return Number::AVOID_ADSB; }
    const char *name() const override { return "AVOID_ADSB"; }
    const char *name4() const override { return "AVOI"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeQStabilize : public Mode
{
public:

    Number mode_number() const override { return Number::QSTABILIZE; }
    const char *name() const override { return "QSTABILIZE"; }
    const char *name4() const override { return "QSTB"; }

    bool is_vtol_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // used as a base class for all Q modes
    bool _enter() override;

protected:

};

class ModeQHover : public Mode
{
public:

    Number mode_number() const override { return Number::QHOVER; }
    const char *name() const override { return "QHOVER"; }
    const char *name4() const override { return "QHOV"; }

    bool is_vtol_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeQLoiter : public Mode
{
public:

    Number mode_number() const override { return Number::QLOITER; }
    const char *name() const override { return "QLOITER"; }
    const char *name4() const override { return "QLOT"; }

    bool is_vtol_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeQLand : public Mode
{
public:

    Number mode_number() const override { return Number::QLAND; }
    const char *name() const override { return "QLAND"; }
    const char *name4() const override { return "QLND"; }

    bool is_vtol_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeQRTL : public Mode
{
public:

    Number mode_number() const override { return Number::QRTL; }
    const char *name() const override { return "QRTL"; }
    const char *name4() const override { return "QRTL"; }

    bool is_vtol_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeQAcro : public Mode
{
public:

    Number mode_number() const override { return Number::QACRO; }
    const char *name() const override { return "QACO"; }
    const char *name4() const override { return "QACRO"; }

    bool is_vtol_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeQAutotune : public Mode
{
public:

    Number mode_number() const override { return Number::QAUTOTUNE; }
    const char *name() const override { return "QAUTOTUNE"; }
    const char *name4() const override { return "QATN"; }

    bool is_vtol_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
    void _exit() override;
};


class ModeTakeoff: public Mode
{
public:
    ModeTakeoff();

    Number mode_number() const override { return Number::TAKEOFF; }
    const char *name() const override { return "TAKEOFF"; }
    const char *name4() const override { return "TKOF"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // var_info for holding parameter information
    static const struct AP_Param::GroupInfo var_info[];
    
protected:
    AP_Int16 target_alt;
    AP_Int16 target_dist;
    AP_Int16 level_alt;
    AP_Int8 level_pitch;

    bool takeoff_started;
    Location start_loc;

    bool _enter() override;
};
