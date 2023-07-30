//
// Created by Jeon Hyeonbae on 2023/07/25.
//
#pragma once

#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

// 착륙 대상의 상태가 무엇인지 지속적으로 모니터링 함.
// 타겟이 시야에 들어오지 않는 경우 매개변수에 따라 타겟을 다시 시야로 가져오기 위한 어떤 조치를 취할 수 있는지 결정.
// 최근 타겟을 잃어버린 경우 기체는 마지막으로 알려진 목표물 위치로 이동/타겟이 마지막으로 감지된 위치로 이동하여 착륙을 재시도
// 타겟이 어디에 있는지 알 수 없는 경우 안전 장치 활성화
// 안전 장치 조치에는 완전 정지(호버링, 수직으로 착륙하는 것이 포함될 수 있음.
class FH_PrecLand_StateMachine {
public:

    // Constructor
    FH_PrecLand_StateMachine() { init(); };

    // 복사방지
    FH_PrecLand_StateMachine(const FH_PrecLand_StateMachine &other) = delete;
    FH_PrecLand_StateMachine &operator=(const FH_PrecLand_StateMachine&) = delete;

    // 상태 머신을 초기화, 기체가 모드를 전환할 때마다 호출.
    void init();

    // precland state machine의 현재 상태
    enum class Status: uint8_t {
        ERROR = 0,               // Unknown error
        DESCEND,                 // 별도의 조치가 필요하지 않음, 수직으로 하강하기.
        RETRYING,                // 기체가 착륙을 재시도하고자 함.
        FAILSAFE                 // failsafe로 전환
    };

    // FailSafe 작동 방식
    enum class FailSafeAction: uint8_t {
        HOLD_POS = 0,            // 현재 포지션 위치
        DESCEND                  // 수직하강
    };

    // Prec Land State Machine을 실행. Prec Landing 중 다음의 네 가지 시나리오가 발생할 수 있음.
    // 1. 타겟이 시야에 들어왔지만 현재는 못찾음. 2. 타겟이 시야에 들어오지 않았으나 사용자가 착륙을 원함.
    // 3. 타겟이 시야에 들어오고, 지속 착륙 가능 4. 센서가 범위를 벗어남.
    // 본 메서드는 위의 모든 시나리오를 처리.
    // 기체가 수행해야 하는 작업 반환
    // 매개변수: Vector3f "retry_pos_m"은 착륙을 재시도해야 하는 경우 필요한 위치로 입력됨.
    Status update(Vector3f &retry_pos_m);

    // state machine의 현재 상태가 "failsafe"를 반환하고 차량이 수행해야 하는 작업을 반환할 때만 호출.
    // 이 방법을 사용하면 공중에서 영구적으로 정지하거나 수직으로 착지할 수만 있음.
    // Failsafe는 최후의 수단으로 트리거.
    FailSafeAction get_failsafe_actions();

    // Prec Landing에 대해 사용자가 원하는 엄격함.
    enum class RetryStrictness: uint8_t {
        NOT_STRICT = 0,         // 이는 Copter 4.1 이하의 동작, 기체는 타겟이 시야에 있는지 여부에 관계없이 최대한 빨리 착륙.(4.1 에서만..?인건가 뭐지?)
        NORMAL,                 // 기체는 실패한 정밀 착륙을 재시도. 타겟을 찾지 못하면 수직착륙.
        VERY_STRICT             // 타겟을 찾지 못하면 차량이 착륙하지 않는다는 점을 제외하면 위와 동일함.
    };

    //수행해야 하는 재시도 조치사항
    enum class RetryAction: uint8_t {
        GO_TO_LAST_LOC = 0,     // 착륙 목표물이 감지된 최근의 위치로 이동
        GO_TO_TARGET_LOC        // 감지된 착륙 대상 위치로 이동
    };

private:

    // 목표물 소실 (얼마 전에 목표물을 발견하였을 경우). 다음에 수행해야 할 작업을 결정하는 데 도움
    // 선택한 작업은 사용자 설정 착륙 엄격도에 따라 다르며 이 기능에 의해 반환됨.
    // 매개 변수: 착륙을 재시도해야 할 경우 "retry_pos_m"의 값은 필요한 위치로 채워짐.
    Status get_target_lost_actions(Vector3f &retry_pos_m);

    // Retry landing based on a previously known location of the landing target
    // Returns the action that should be taken by the vehicle
    // Vector3f "retry_pos_m" is filled with the required location.
    Status retry_landing(Vector3f &retry_pos_m);

    // Reset the landing statemachine. This needs to be called everytime the landing target is back in sight.
    // So that if the landing target goes out of sight again, we can start the failed landing procedure back from the beginning stage
    void reset_failed_landing_statemachine();

    // State machine for action to do when Landing target is lost (after it was in sight a while back)
    enum class TargetLostAction: uint8_t {
        INIT = 0,               // Decide on what action needs to be taken
        DESCEND,                // Descend for sometime (happens if we have just lost the target)
        LAND_VERTICALLY,        // Land vertically
        RETRY_LANDING,          // Retry landing (only possible if we had the landing target in sight sometime during the flight)
    };

    TargetLostAction landing_target_lost_action;  // Current action being done in the Lost Landing target state machine

    // State Machine for landing retry
    enum class RetryLanding : uint8_t {
        INIT = 0,               // Init the retry statemachine. This would involve increasing the retry counter (so we how many times we have already retried)
        IN_PROGRESS,            // Retry in progress, we wait for the vehicle to get close to the target location
        DESCEND,                // Descend to the original height from where we had started the retry
        COMPLETE                // Retry completed. We try failsafe measures after this
    };
    RetryLanding _retry_state;   // Current action being done in the Landing retry state machine
    uint8_t _retry_count;       // Total number of retires done in this mode

    bool failsafe_initialized;  // True if failsafe has been initalized
    uint32_t failsafe_start_ms; // timestamp of when failsafe was triggered

};