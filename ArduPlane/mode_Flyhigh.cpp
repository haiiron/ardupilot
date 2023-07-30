// 2023/07/05 ~ 作
// Created by Haiiron
// 경운대학교 Flyhigh 임무 수행을 위한 모드
//

#include "mode.h"
#include "Plane.h"


// Mode::Mode() : ahrs(plane.ahrs) , quadplane(plane.quadplane), pos_control(plane.quadplane.pos_control), attitude_control(plane.quadplane.attitude_control), loiter_nav(plane.quadplane.loiter_nav), poscontrol(plane.quadplane.poscontrol)

bool ModeFlyhigh::_enter()
{
    gcs().send_text(MAV_SEVERITY_INFO,"Flyhigh mode is for precland");
    plane.precland_statemachine.init(); //precland 구현
	return true;
}

void ModeFlyhigh::update()
{  }

void ModeFlyhigh::run()
{
    static int r_i = 0; r_i++; //run 함수 호출 횟수 측정용
    gcs().send_text(MAV_SEVERITY_INFO,"gcs... is work well on {{run}}!!! worked %d times",r_i);
 }
