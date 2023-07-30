//
// Created by Jeon Hyeonbae on 2023/07/25.
//
#pragma once

#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_PID.h>
#include "FH_PrecLand.h"

class FH_PrecLand_Backend
{
public:
    // Constructor
    FH_PrecLand_Backend(const FH_PrecLand& frontend, FH_PrecLand::precland_state& state) :
        _frontend(frontend),
        _state(state) {}

    // destructor
    virtual ~FH_PrecLand_Backend() {}

    // perform any required initialisation of backend
    virtual void init() = 0;

    // retrieve updates from sensor
    virtual void update() = 0;

    // provides a unit vector towards the target in body frame
    //  returns same as have_los_meas()
    virtual bool get_los_body(Vector3f& dir_body) = 0;

    // returns system time in milliseconds of last los measurement
    virtual uint32_t los_meas_time_ms() = 0;

    // return true if there is a valid los measurement available
    virtual bool have_los_meas() = 0;

    // returns distance to target in meters (0 means distance is not known)
    virtual float distance_to_target() { return 0.0f; };

    // parses a mavlink message from the companion computer
    virtual void handle_msg(const mavlink_landing_target_t &packet, uint32_t timestamp_ms) {};

    // get bus parameter
    int8_t get_bus(void) const { return _frontend._bus.get(); }
    
protected:
    const FH_PrecLand&  _frontend;          // reference to precision landing front end
    FH_PrecLand::precland_state &_state;    // reference to this instances state
};
