// 2023/07/05 ~ 作
// Created by Haiiron
// 경운대학교 Flyhigh 임무 수행을 위한 모드
//

#include "mode.h"
#include "Plane.h"


// Mode::Mode() : ahrs(plane.ahrs) , quadplane(plane.quadplane), pos_control(plane.quadplane.pos_control), attitude_control(plane.quadplane.attitude_control), loiter_nav(plane.quadplane.loiter_nav), poscontrol(plane.quadplane.poscontrol)

bool ModeFlyHigh_mission::_enter()
{
    // mission storage는 24비트의 고도를 저장함을 인지할 것 (~ +/- 83km)
//    int32_t altitude;
    int32_t lat = 0, lng = 0;

    Location ekf_origin;
     if (AP::ahrs().get_origin(ekf_origin))
    {
        gcs().send_text(MAV_SEVERITY_INFO,"ahrs get origin");
        lat = ekf_origin.lat;
        lng = ekf_origin.lng;
    }

    gcs().send_text(MAV_SEVERITY_INFO,"ahrs data :: lat is %ld , lng is %ld",lat,lng);
    return true;
}

void ModeFlyHigh_mission::update()
{
    gcs().send_text(MAV_SEVERITY_INFO,"Flyhigh call");
}

void ModeFlyHigh_mission::run()
{
    static int r_i = 0; r_i++; //run 함수 호출 횟수 측정용
    hal.console->printf("hal... run on {{run}} function!!!\n holy shit!!!\n");
    gcs().send_text(MAV_SEVERITY_INFO,"gcs... is work well on {{run}}!!! worked %d times",r_i);
 }