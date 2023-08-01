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
    gcs().send_text(MAV_SEVERITY_INFO,"23-07-31 developed by haiiron : qrtl 호출한 착륙모드");
    
    plane.precland_statemachine.init(); //precland 구현

    plane.mode_qrtl._enter();

	return true;
}

void ModeFlyhigh::update()
{  plane.mode_qstabilize.update(); }

void ModeFlyhigh::run()
{ plane.mode_qrtl.run(); }
