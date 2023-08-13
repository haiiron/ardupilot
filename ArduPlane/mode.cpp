#include "Plane.h"

Mode::Mode() :
    ahrs(plane.ahrs)
#if HAL_QUADPLANE_ENABLED
    , quadplane(plane.quadplane),
    pos_control(plane.quadplane.pos_control),
    attitude_control(plane.quadplane.attitude_control),
    loiter_nav(plane.quadplane.loiter_nav),
    poscontrol(plane.quadplane.poscontrol)
#endif
{
}

void Mode::exit()
{
    // call sub-classes exit
    _exit();
    // stop autotuning if not AUTOTUNE mode
    if (plane.control_mode != &plane.mode_autotune){
        plane.autotune_restore();
    }

}

bool Mode::enter()
{
#if AP_SCRIPTING_ENABLED
    // reset nav_scripting.enabled
    plane.nav_scripting.enabled = false;
#endif

    // cancel inverted flight
    plane.auto_state.inverted_flight = false;
    
    // cancel waiting for rudder neutral
    plane.takeoff_state.waiting_for_rudder_neutral = false;

    // don't cross-track when starting a mission
    plane.auto_state.next_wp_crosstrack = false;

    // reset landing check
    plane.auto_state.checked_for_autoland = false;

    // zero locked course
    plane.steer_state.locked_course_err = 0;
    plane.steer_state.locked_course = false;

    // reset crash detection
    plane.crash_state.is_crashed = false;
    plane.crash_state.impact_detected = false;

    // reset external attitude guidance
    plane.guided_state.last_forced_rpy_ms.zero();
    plane.guided_state.last_forced_throttle_ms = 0;

#if OFFBOARD_GUIDED == ENABLED
    plane.guided_state.target_heading = -4; // radians here are in range -3.14 to 3.14, so a default value needs to be outside that range
    plane.guided_state.target_heading_type = GUIDED_HEADING_NONE;
    plane.guided_state.target_airspeed_cm = -1; // same as above, although an airspeed of -1 is rare on plane.
    plane.guided_state.target_alt = -1; // same as above, although a target alt of -1 is rare on plane.
    plane.guided_state.last_target_alt = 0;
#endif

#if AP_CAMERA_ENABLED
    plane.camera.set_is_auto_mode(this == &plane.mode_auto);
#endif

    // zero initial pitch and highest airspeed on mode change
    plane.auto_state.highest_airspeed = 0;
    plane.auto_state.initial_pitch_cd = ahrs.pitch_sensor;

    // disable taildrag takeoff on mode change
    plane.auto_state.fbwa_tdrag_takeoff_mode = false;

    // start with previous WP at current location
    plane.prev_WP_loc = plane.current_loc;

    // new mode means new loiter
    plane.loiter.start_time_ms = 0;

    // record time of mode change
    plane.last_mode_change_ms = AP_HAL::millis();

    // set VTOL auto state
    plane.auto_state.vtol_mode = is_vtol_mode();
    plane.auto_state.vtol_loiter = false;

    // initialize speed variable used in AUTO and GUIDED for DO_CHANGE_SPEED commands
    plane.new_airspeed_cm = -1;

#if HAL_QUADPLANE_ENABLED
    quadplane.mode_enter();
#endif

    bool enter_result = _enter();

    if (enter_result) {
        // -------------------
        // these must be done AFTER _enter() because they use the results to set more flags

        // start with throttle suppressed in auto_throttle modes
        plane.throttle_suppressed = does_auto_throttle();
#if HAL_ADSB_ENABLED
        plane.adsb.set_is_auto_mode(does_auto_navigation());
#endif

        // reset steering integrator on mode change
        plane.steerController.reset_I();

        // update RC failsafe, as mode change may have necessitated changing the failsafe throttle
        plane.control_failsafe();

#if AP_FENCE_ENABLED
        // pilot requested flight mode change during a fence breach indicates pilot is attempting to manually recover
        // this flight mode change could be automatic (i.e. fence, battery, GPS or GCS failsafe)
        // but it should be harmless to disable the fence temporarily in these situations as well
        plane.fence.manual_recovery_start();
#endif
    }

    return enter_result;
}

bool Mode::is_vtol_man_throttle() const
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.tailsitter.is_in_fw_flight() &&
        plane.quadplane.assisted_flight) {
        // We are a tailsitter that has fully transitioned to Q-assisted forward flight.
        // In this case the forward throttle directly drives the vertical throttle so
        // set vertical throttle state to match the forward throttle state. Confusingly the booleans are inverted,
        // forward throttle uses 'does_auto_throttle' whereas vertical uses 'is_vtol_man_throttle'.
        return !does_auto_throttle();
    }
#endif
    return false;
}

void Mode::update_target_altitude()
{
    Location target_location;

    if (plane.landing.is_flaring()) {
        // during a landing flare, use TECS_LAND_SINK as a target sink
        // rate, and ignores the target altitude
        plane.set_target_altitude_location(plane.next_WP_loc);
    } else if (plane.landing.is_on_approach()) {
        plane.landing.setup_landing_glide_slope(plane.prev_WP_loc, plane.next_WP_loc, plane.current_loc, plane.target_altitude.offset_cm);
        plane.landing.adjust_landing_slope_for_rangefinder_bump(plane.rangefinder_state, plane.prev_WP_loc, plane.next_WP_loc, plane.current_loc, plane.auto_state.wp_distance, plane.target_altitude.offset_cm);
    } else if (plane.landing.get_target_altitude_location(target_location)) {
        plane.set_target_altitude_location(target_location);
#if HAL_SOARING_ENABLED
    } else if (plane.g2.soaring_controller.is_active() && plane.g2.soaring_controller.get_throttle_suppressed()) {
        // Reset target alt to current alt, to prevent large altitude errors when gliding.
        plane.set_target_altitude_location(plane.current_loc);
        plane.reset_offset_altitude();
#endif
    } else if (plane.reached_loiter_target()) {
        // once we reach a loiter target then lock to the final
        // altitude target
        plane.set_target_altitude_location(plane.next_WP_loc);
    } else if (plane.target_altitude.offset_cm != 0 && 
               !plane.current_loc.past_interval_finish_line(plane.prev_WP_loc, plane.next_WP_loc)) {
        // control climb/descent rate
        plane.set_target_altitude_proportion(plane.next_WP_loc, 1.0f-plane.auto_state.wp_proportion);

        // stay within the range of the start and end locations in altitude
        plane.constrain_target_altitude_location(plane.next_WP_loc, plane.prev_WP_loc);
    } else {
        plane.set_target_altitude_location(plane.next_WP_loc);
    }

    plane.altitude_error_cm = plane.calc_altitude_error_cm();
}

// returns true if the vehicle can be armed in this mode
bool Mode::pre_arm_checks(size_t buflen, char *buffer) const
{
    if (!_pre_arm_checks(buflen, buffer)) {
        if (strlen(buffer) == 0) {
            // If no message is provided add a generic one
            hal.util->snprintf(buffer, buflen, "mode not armable");
        }
        return false;
    }

    return true;
}

// Auto and Guided do not call this to bypass the q-mode check.
bool Mode::_pre_arm_checks(size_t buflen, char *buffer) const
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.enabled() && !is_vtol_mode() &&
            plane.quadplane.option_is_set(QuadPlane::OPTION::ONLY_ARM_IN_QMODE_OR_AUTO)) {
        hal.util->snprintf(buffer, buflen, "not Q mode");
        return false;
    }
#endif
    return true;
}

void Mode::run()
{
    // Direct stick mixing functionality has been removed, so as not to remove all stick mixing from the user completely
    // the old direct option is now used to enable fbw mixing, this is easier than doing a param conversion.
    if ((plane.g.stick_mixing == StickMixing::FBW) || (plane.g.stick_mixing == StickMixing::DIRECT_REMOVED)) {
        plane.stabilize_stick_mixing_fbw();
    }
    plane.stabilize_roll();
    plane.stabilize_pitch();
    plane.stabilize_yaw();
}

// Reset rate and steering controllers
void Mode::reset_controllers()
{
    // reset integrators
    plane.rollController.reset_I();
    plane.pitchController.reset_I();
    plane.yawController.reset_I();

    // reset steering controls
    plane.steer_state.locked_course = false;
    plane.steer_state.locked_course_err = 0;
}

bool Mode::is_taking_off() const
{ return (plane.flight_stage == AP_FixedWing::FlightStage::TAKEOFF); }

//FIXME: [part of flyhigh precland]
void Mode::flyhigh_precland_land_run(bool pause_descent)
{
    if(!pause_descent)
    {
        gcs().send_text(MAV_SEVERITY_INFO,"is land ? : %s",quadplane.check_land_complete() ? "true" : "false");

        const uint32_t now = AP_HAL::millis();
    
    if (quadplane.tailsitter.in_vtol_transition(now))
    {
        // VTOL 전환 실행 FW 컨트롤러 FW 풀업 단계에서 Tailsitters
        Mode::run();
        return;
    }

    if(poscontrol.get_state() < QuadPlane::QPOS_LAND_FINAL && quadplane.check_land_final())
    {
        poscontrol.set_state(QuadPlane::QPOS_LAND_FINAL);
        quadplane.setup_target_position();
#if AP_ICENGINE_ENABLED
        // IC engine 종료 (IC engine이 할당 되어있다면.)
        if (quadplane.land_icengine_cut != 0)
        { plane.g2.ice_control.engine_control(0, 0, 0); }
#endif  // AP_ICENGINE_ENABLED
    }
    float height_above_ground = plane.relative_ground_altitude(plane.g.rangefinder_landing);
    float descent_rate_cms = quadplane.landing_descent_rate_cms(height_above_ground);

    if (poscontrol.get_state() == QuadPlane::QPOS_LAND_FINAL && !quadplane.option_is_set(QuadPlane::OPTION::DISABLE_GROUND_EFFECT_COMP))
    { ahrs.set_touchdown_expected(true); }

    pos_control->land_at_climb_rate_cm(-descent_rate_cms, descent_rate_cms>0);
    quadplane.check_land_complete();

//FIXME: 이 부분 호출 안되었을 떄랑 될 떄 어떻게 동작하는가 테스트하기!

    quadplane.run_z_controller();

    // Stabilize with fixed wing surfaces
    plane.stabilize_roll();
    plane.stabilize_pitch();
    }


}

/*
// 착륙을 재시도하기 위해 사전 착륙 상태 기계가 명령하는 위치로 이동
// 통과된 위치는 m 단위의 NED
void Mode::precland_retry_position(const Vector3f &retry_pos)
{
    if (!plane.failsafe.radio) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            AP::logger().Write_Event(LogEvent::LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            if (!set_mode(Mode::Number::LOITER, ModeReason::THROTTLE_LAND_ESCAPE)) {
                set_mode(Mode::Number::ALT_HOLD, ModeReason::THROTTLE_LAND_ESCAPE);
            }
        }

        // allow user to take control during repositioning. Note: copied from land_run_horizontal_control()
        // To-Do: this code exists at several different places in slightly different forms and that should be fixed
        if (g.land_repositioning) {
            float target_roll = 0.0f;
            float target_pitch = 0.0f;
            // convert pilot input to lean angles
            get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

            // record if pilot has overridden roll or pitch
            if (!is_zero(target_roll) || !is_zero(target_pitch)) {
                if (!copter.ap.land_repo_active) {
                    AP::logger().Write_Event(LogEvent::LAND_REPO_ACTIVE);
                }
                // this flag will be checked by prec land state machine later and any further landing retires will be cancelled
                copter.ap.land_repo_active = true;
            }
        }
    }

    Vector3p retry_pos_NEU{retry_pos.x, retry_pos.y, retry_pos.z * -1.0f};
    // pos controller expects input in NEU cm's
    retry_pos_NEU = retry_pos_NEU * 100.0f;
    pos_control->input_pos_xyz(retry_pos_NEU, 0.0f, 1000.0f);

    // run position controllers
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // call attitude controller
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());

}
*/

// precland statemachine 실행. precision landing이 필요한 모드에서 호출할 것.
// 이 기능은 착륙 전부터 precland 실패, 재시도 및 실패 안전 조치에 이르기까지 모든 것을 처리
void Mode::precland_run()
{
        // 필요한 경우 나중에 재시도 pos로 업데이트
        Vector3f retry_pos;

        switch (plane.precland_statemachine.update(retry_pos))
        {    
            case FH_PrecLand_StateMachine::Status::RETRYING: // 다른 위치로 가서 착륙을 재시도
                gcs().send_text(MAV_SEVERITY_INFO,"Status::RETRYING");
                gcs().send_text(MAV_SEVERITY_INFO,"Vec x : %f ",retry_pos.x);
                gcs().send_text(MAV_SEVERITY_INFO,"Vec x : %f ",retry_pos.y);
                gcs().send_text(MAV_SEVERITY_INFO,"Vec x : %f ",retry_pos.z);
                break;

            case FH_PrecLand_StateMachine::Status::FAILSAFE:
            {
                // we have hit a failsafe. Failsafe can only mean two things, we either want to stop permanently till user takes over or land
                switch (plane.precland_statemachine.get_failsafe_actions())
                {
                    case FH_PrecLand_StateMachine::FailSafeAction::DESCEND:
                        // 사전 착륙 목표물은 확실히 보이지 않음, 일반 하강.
                        gcs().send_text(MAV_SEVERITY_INFO,"FailSafeAction::DESCEND");
                        flyhigh_precland_land_run();
                        break;
                    case FH_PrecLand_StateMachine::FailSafeAction::HOLD_POS:
                        // 이 인수에서 "true"를 보내면 하강이 중지.
                        gcs().send_text(MAV_SEVERITY_INFO,"FailSafeAction::HOLD_POS");
                        break;
                }
            break;
            }
            
            case FH_PrecLand_StateMachine::Status::ERROR:
                // 일어나서는 안 됨, 버그발생.
                gcs().send_text(MAV_SEVERITY_INFO,"Status::ERROR <Bug>");
                FALLTHROUGH;
            
            case FH_PrecLand_StateMachine::Status::DESCEND:
                // 랜드 컨트롤러를 실행. 이것은 전방의 목표물이 보이면 목표물을 향해 내려감.
                // 그렇지 않으면 수직으로 하강
                gcs().send_text(MAV_SEVERITY_INFO,"Target detect landing down");
                flyhigh_precland_land_run();
                break;
        }
}