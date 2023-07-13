#include "Copter.h"

/*
 * Init and run calls for stabilize flight mode
 */

bool FLY_HIGH::init(bool ignore_checks)
{
        return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void FLY_HIGH::run()
{
    gcs().send_text(MAV_SEVERITY_INFO,"run does work");
}