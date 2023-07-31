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
    gcs().send_text(MAV_SEVERITY_INFO,"23-07-31 developed by haiiron : qland와 동일한 착륙모드");
    
    plane.precland_statemachine.init(); //precland 구현

    plane.mode_qloiter._enter();
    quadplane.throttle_wait = false;
    quadplane.setup_target_position();
    poscontrol.set_state(QuadPlane::QPOS_LAND_DESCEND);
    poscontrol.pilot_correction_done = false;
    quadplane.last_land_final_agl = plane.relative_ground_altitude(plane.g.rangefinder_landing);
    quadplane.landing_detect.lower_limit_start_ms = 0;
    quadplane.landing_detect.land_start_ms = 0;
#if AP_LANDINGGEAR_ENABLED
    plane.g2.landing_gear.deploy_for_landing();
#endif
#if AP_FENCE_ENABLED
    plane.fence.auto_disable_fence_for_landing();
#endif
	return true;
}

void ModeFlyhigh::update()
{  plane.mode_qstabilize.update(); }

void ModeFlyhigh::run()
{ plane.mode_qloiter.run(); }
