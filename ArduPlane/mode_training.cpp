#include "mode.h"
#include "Plane.h"

void ModeTraining::update()
{
//    gcs().send_text(MAV_SEVERITY_INFO,"Training mode - Precland 호출 테스트");
}


void ModeTraining::run()
{
    precland_run();
}
