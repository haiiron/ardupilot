#include "mode.h"
#include "Plane.h"

void ModeTraining::update()
{
    gcs().send_text(MAV_SEVERITY_INFO,"Training mode");
}


void ModeTraining::run()
{

}
