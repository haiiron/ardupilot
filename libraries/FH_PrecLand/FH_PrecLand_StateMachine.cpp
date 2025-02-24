//
// Created by Jeon Hyeonbae on 2023/07/25.
//

#include "FH_PrecLand_StateMachine.h"
#include <FH_PrecLand/FH_PrecLand.h>
#include <AP_AHRS/AP_AHRS.h>

static const float    MAX_POS_ERROR_M          = 0.75f;  // Maximum possition error for retry locations
static const uint32_t FAILSAFE_INIT_TIMEOUT_MS = 10000;   // Timeout in ms before failsafe measures are started. During this period vehicle is completely stopped to give user the time to take over
static const float    RETRY_OFFSET_ALT_M       = 1.5f;  // This gets added to the altitude of the retry location

// state machine 초기화, 모드를 전환할 때마다 호출.
void FH_PrecLand_StateMachine::init()
{
    FH_PrecLand *_precland = FH::fh_precland();
    if (_precland == nullptr) { return; } // precland enabled 안되어있음.

    // precland is not enabled, prec land state machine methods should not be called!
    if (!_precland->enabled()) { return; }

    // init is only called ONCE per mode change. So in a particuar mode we can retry only a finite times.
    // The counter will be reset if the statemachine is called from a different mode
    _retry_count = 0;
    reset_failed_landing_statemachine(); // reset every other statemachine
}

// Reset the landing statemachines. This needs to be called everytime the landing target is back in sight.
// So that if the landing target goes out of sight again, we can start the failed landing procedure back from the beginning stage
void FH_PrecLand_StateMachine::reset_failed_landing_statemachine()
{
    landing_target_lost_action = TargetLostAction::INIT;
    _retry_state = RetryLanding::INIT;
    failsafe_initialized = false;
}
// 사전 착륙 상태 기계를 실행합니다. 사전 착륙 중에 다음 네 가지 시나리오가 발생할 수 있습니다:
// 1. 타겟 실종
// 2. 타겟이 보이지 않으나 착륙 진행 하고자 함.
// 3. 타겟 할당, 착륙 진행가능
// 4. 센서가 범위를 벗어남
// 위의 모든 시나리오를 다룸, 수행해야 하는 작업을 반환
// 매개 변수: 착륙을 재시도해야 할 경우 Vector 3f "retry_pos_m"이 필요한 위치로 입력됨.
FH_PrecLand_StateMachine::Status FH_PrecLand_StateMachine::update(Vector3f &retry_pos_m)
{

    // grab the current status of Landing Target
    FH_PrecLand *_precland = FH::fh_precland();
    if (_precland == nullptr) { return Status::ERROR; } // should never happen

    if (!_precland->enabled()) // precland 비 활성화, precland state machine 호출하지 아니함.
    { return Status::ERROR; }

    FH_PrecLand::TargetState precland_target_state =  _precland->get_target_state();

    switch (precland_target_state)
    {
        case FH_PrecLand::TargetState::TARGET_RECENTLY_LOST:
            // we have lost the target but had it in sight at least once recently
            // action will depend on what user wants
            return get_target_lost_actions(retry_pos_m);

        case FH_PrecLand::TargetState::TARGET_NEVER_SEEN:
            // we have no clue where we are supposed to be landing
            // let user decide how strict our failsafe actions need to be
            return Status::FAILSAFE;

        case FH_PrecLand::TargetState::TARGET_OUT_OF_RANGE:
            // The target isn't in sight, but we can't run any fail safe measures or do landing retry
            // Therefore just descend for now, and check again later if retry is allowed
        case FH_PrecLand::TargetState::TARGET_FOUND:
            // no action required, target is in sight
            reset_failed_landing_statemachine();
            return Status::DESCEND;
    }

    return Status::ERROR; // 여기까지 올 수 없음.
}


// Target is lost (i.e we had it in sight some time back), this method helps decide on what needs to be done next
// The chosen action depends on user set landing strictness and will be returned by this function
// Parameters: Vector3f "retry_pos_m" is filled with the required location if we need to retry landing.
FH_PrecLand_StateMachine::Status FH_PrecLand_StateMachine::get_target_lost_actions(Vector3f &retry_pos_m)
{
    FH_PrecLand *_precland = FH::fh_precland();
    if (_precland == nullptr) { return Status::ERROR; } // should never happen

    switch (landing_target_lost_action)
    {
        case TargetLostAction::INIT:
        {
            // figure out how strict the user is with the landing
            const RetryStrictness strictness =_precland->get_retry_strictness();
            switch (strictness)
            {
                case RetryStrictness::NORMAL:
                case RetryStrictness::VERY_STRICT:
                    // We eventually want to retry landing, but lets descend for some time and hope the target gets in sight
                    // If not, we will retry landing
                    landing_target_lost_action = TargetLostAction::DESCEND;
                    break;
                case RetryStrictness::NOT_STRICT:
                    // User just wants to land, prec land isn't a priority
                    landing_target_lost_action = TargetLostAction::LAND_VERTICALLY;
                    break;
            }
            // at this stage we will be descending no matter what
            // so no special action required
            return Status::DESCEND;
        }

        case TargetLostAction::DESCEND:
            if (AP_HAL::millis() - _precland->get_last_valid_target_ms() >=_precland->get_min_retry_time_sec() * 1000) {
                // we have descended for some time and the target still isn't in sight
                // lets retry
                landing_target_lost_action = TargetLostAction::RETRY_LANDING;
                _retry_state = RetryLanding::INIT;
            }
            return Status::DESCEND; // still descending, no other action

        case TargetLostAction::RETRY_LANDING:
            // retry the landing by going to another position
            return retry_landing(retry_pos_m);

        case TargetLostAction::LAND_VERTICALLY:
            // Just land vertically
            // we will not be retrying to any location here on, so return false
            return Status::DESCEND;
    }

    // should never reach here, all cases are handled above
    return Status::ERROR;
}

// Retry landing based on a previously known location of the landing target
// Returns the action that should be taken by the vehicle
// Vector3f "retry_pos_m" is filled with the required location.
FH_PrecLand_StateMachine::Status FH_PrecLand_StateMachine::retry_landing(Vector3f &retry_pos_m)
{
    FH_PrecLand *_precland = FH::fh_precland();

    if (_precland == nullptr)
    { return Status::ERROR; } // should never happen

    if (_precland->get_max_retry_allowed() == 0)
    {  return Status::FAILSAFE; } // user does not want retry

    if (_retry_count > _precland->get_max_retry_allowed()) {
        // we have exhausted the amount of times vehicle was allowed to retry landing
        // do failsafe measure so the vehicle isn't stuck in a constant loop
        return Status::FAILSAFE;
    }

    // get the retry position. This depends on what retry behavior has been set by user
    Vector3f go_to_pos;
    const RetryAction retry_action = _precland->get_retry_behaviour();
    if (retry_action == RetryAction::GO_TO_TARGET_LOC)
    { _precland->get_last_detected_landing_pos(go_to_pos); }

    else if (retry_action == RetryAction::GO_TO_LAST_LOC)
    { _precland->get_last_vehicle_pos_when_target_detected(go_to_pos); }

    // add a little bit offset so the vehicle climbs slightly higher than where it was
    // remember this is "D" frame and in meters's
    go_to_pos.z -= RETRY_OFFSET_ALT_M;

    switch (_retry_state)
    {
        case RetryLanding::INIT:
            // Init the Retry
            _retry_count ++;
            _retry_state = RetryLanding::IN_PROGRESS;
            // inform the user what we are doing
            if (_retry_count <= _precland->get_max_retry_allowed())
            { gcs().send_text(MAV_SEVERITY_INFO, "PrecLand: Retrying"); }
            retry_pos_m = go_to_pos;
            return Status::RETRYING;

        case RetryLanding::IN_PROGRESS:
        {
            // continue converging towards the target till we are close by
            retry_pos_m = go_to_pos;
            Vector3f pos;
            if (!AP::ahrs().get_relative_position_NED_origin(pos))
            { return Status::ERROR; }
            const float dist_to_target = (go_to_pos-pos).length();
            if ((dist_to_target < MAX_POS_ERROR_M))
            { _retry_state = RetryLanding::DESCEND; } // 이전에 감지된 착륙 지점에 거의 도달

            return Status::RETRYING;
        }

        case RetryLanding::DESCEND:
        {
            // descend a little bit before completing the retry
            // This will descend to the original height of where landing target was first detected
            Vector3f pos;
            if (!AP::ahrs().get_relative_position_NED_origin(pos))
            { return Status::ERROR; }
            // z_target is in "D" frame
            const float z_target = go_to_pos.z + RETRY_OFFSET_ALT_M;
            retry_pos_m = Vector3f{pos.x, pos.y, z_target};
            if (fabsf(pos.z - retry_pos_m.z) < MAX_POS_ERROR_M)
            { // we have descended to the original height where we started the climb from
                _retry_state = RetryLanding::COMPLETE;
                gcs().send_text(MAV_SEVERITY_INFO, "PrecLand: Retry Completed");
            }
            return Status::RETRYING;
        }

        case RetryLanding::COMPLETE:
            // Vehicle has completed a retry, and most likely the landing location still isn't sight
            // we have no choice but to force a failsafe action
            return Status::FAILSAFE;
    }

    // should never reach here
    return Status::ERROR;
}

// This is only called when the current status of the state machine returns "failsafe" and will return the action that the vehicle should do
// At the moment this method only allows you to stop in air permanently, or land vertically
// Failsafe will only trigger as a last resort
FH_PrecLand_StateMachine::FailSafeAction FH_PrecLand_StateMachine::get_failsafe_actions()
{
    FH_PrecLand *_precland = FH::fh_precland();
    
    // should never happen, just descend
    if (_precland == nullptr) { return FailSafeAction::DESCEND; }

    if (!failsafe_initialized)
    {
        failsafe_start_ms = AP_HAL::millis(); // start the timer
        failsafe_initialized = true;
        gcs().send_text(MAV_SEVERITY_INFO, "PrecLand: Failsafe Measures");
    }

    // Depending on the strictness we will either land vertically, wait for some time and then land vertically, not land at all
    const RetryStrictness strictness= _precland->get_retry_strictness();
    switch (strictness)
    {
        case RetryStrictness::VERY_STRICT:
            // user does not want to land on anything but the target
            // stop landing (hover)
            return FailSafeAction::HOLD_POS;

        case RetryStrictness::NORMAL:
            if (AP_HAL::millis() - failsafe_start_ms < FAILSAFE_INIT_TIMEOUT_MS)
            {
                // stop the vehicle for at least a few seconds before descending
                // this might give user the chance to take over
                // we do not want to be too linent in landing vertically because of the strictness set by the user
                return FailSafeAction::HOLD_POS;
            }
            // failsafe 수직착륙
            return FailSafeAction::DESCEND;

        case RetryStrictness::NOT_STRICT:
            // User wants to prioritize landing over staying in the air
            return FailSafeAction::DESCEND;
    }
    return FailSafeAction::DESCEND; // 여기까지 올 일은 없음.. (온다면 failsafe 수직착륙)
}
