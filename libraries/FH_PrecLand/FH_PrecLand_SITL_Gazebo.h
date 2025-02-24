#pragma once

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <AP_Math/AP_Math.h>
#include <FH_PrecLand/FH_PrecLand_Backend.h>
#include <AP_IRLock/AP_IRLock_SITL_Gazebo.h>

/*
 * AC_PrecLand_SITL_Gazebo - implements precision landing using target
 * vectors provided Gazebo via a network socket
 */

class FH_PrecLand_SITL_Gazebo : public FH_PrecLand_Backend
{
public:

    // Constructor
    FH_PrecLand_SITL_Gazebo(const FH_PrecLand& frontend, FH_PrecLand::precland_state& state);

    // perform any required initialisation of backend
    void init() override;

    // retrieve updates from sensor
    void update() override;

    // provides a unit vector towards the target in body frame
    //  returns same as have_los_meas()
    bool get_los_body(Vector3f& ret) override;

    // returns system time in milliseconds of last los measurement
    uint32_t los_meas_time_ms() override;

    // return true if there is a valid los measurement available
    bool have_los_meas() override;

private:
    AP_IRLock_SITL_Gazebo irlock;

    Vector3f            _los_meas_body;         // unit vector in body frame pointing towards target
    bool                _have_los_meas;         // true if there is a valid measurement from the camera
    uint32_t            _los_meas_time_ms;      // system time in milliseconds when los was measured
};

#endif
