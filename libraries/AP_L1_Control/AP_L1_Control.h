#pragma once

/// @file    AP_L1_Control.h
/// @brief   L1 Control algorithm. This is a instance of an
/// AP_Navigation class

/*
 * Originally written by Brandon Jones 2013
 *
 *  Modified by Paul Riseborough 2013 to provide:
 *  - Explicit control over frequency and damping
 *  - Explicit control over track capture angle
 *  - Ability to use loiter radius smaller than L1 length
 */

#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <AP_Navigation/AP_Navigation.h>
#include <AP_TECS/AP_TECS.h>
#include <AP_Common/Location.h>

class AP_L1_Control : public AP_Navigation {
public:
    AP_L1_Control(AP_AHRS &ahrs, const AP_TECS *tecs)
        : _ahrs(ahrs)
        , _tecs(tecs)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* 클레스 중복 방지 */
    CLASS_NO_COPY(AP_L1_Control);

    /* 정의 및 단위는 AP_Navigation.h를 참조할 것 */
    
    int32_t nav_roll_cd(void) const override;
    float lateral_acceleration(void) const override;

    // 원하는 트랙 heading 각도를 반환 (centi-degrees)
    int32_t nav_bearing_cd(void) const override;

    // heading 오류 각도 반환 (centi-degrees) + ve는 트랙 왼쪽임
    int32_t bearing_error_cd(void) const override;

    float crosstrack_error(void) const override { return _crosstrack_error; }
    float crosstrack_error_integrator(void) const override { return _L1_xtrack_i; }

    int32_t target_bearing_cd(void) const override;
    
    // 회전 거리
    float turn_distance(float wp_radius) const override;
    float turn_distance(float wp_radius, float turn_angle) const override;
    
    float loiter_radius(const float loiter_radius) const override;
    
    // 웨이포인트 탐색을 위한 L1 컨트롤 업데이트
    void update_waypoint(const class Location &prev_WP, const class Location &next_WP, float dist_min = 0.0f) override;
    // loitering을 위한 L1 컨트롤 업데이트
    void update_loiter(const class Location &center_WP, float radius, int8_t loiter_direction) override;
    
    // 헤딩 홀드 내비게이션을 위한 L1 컨트롤 업데이트
    void update_heading_hold(int32_t navigation_heading_cd) override;

    // 현재 헤딩의 수평 비행에 대한 L1 제어 업데이트
    void update_level_flight(void) override;
    
    bool reached_loiter_target(void) override;

    // NAVL1_PERIOD 설정
    void set_default_period(float period)
    { _L1_period.set_default(period); }

    void set_data_is_stale(void) override
    { _data_is_stale = true; }
    
    bool data_is_stale(void) const override
    { return _data_is_stale; }

    // NAVl1_* 사용자 설정 가능한 매개 변수
    static const struct AP_Param::GroupInfo var_info[];

    void set_reverse(bool reverse) override
    { _reverse = reverse; }

private:
    // AHRS 개체 참조
    AP_AHRS &_ahrs;

    // SpdHgtControl 개체 포인터
    const AP_TECS *_tecs;

    // 비행에 필요한 m/s 단위의 측면 가속도
    //L1 기준점(+ve는 우측)
    float _latAccDem;

    // 동적으로 업데이트되는 L1 추적 거리 (미터 단위)
    float _L1_dist;

    // 기체가 WayPoint를 선회하기 시작했을 때 True
    bool _WPcircle;

    // L1 지점까지의 bearing 라디안 단위의 각도
    float _nav_bearing;

    // bearing 오차 라디안 단위의 각도 +ve는 트랙 왼쪽
    float _bearing_error;

    // 미터 단위의 크로스트랙 오류
    float _crosstrack_error;

    // 최근(마지막) 업데이트 에서의 centi-degrees 단위의 목표 bearing
    int32_t _target_bearing_cd;

    // L1 추적 루프 주기(초)
    AP_Float _L1_period;
    // L1 추적 루프 댐핑 비율
    AP_Float _L1_damping;

    // cross-track velocity의 이전 값
    float _last_Nu;

    // 경유지 추적에 있어서 우유부단한 결정을 방지
    void _prevent_indecision(float &Nu);

    // integral feedback to correct crosstrack error. Used to ensure xtrack converges to zero.
    // For tuning purposes it's helpful to clear the integrator when it changes so a _prev is used
    // 크로스트랙 오류를 수정하기 위한 통합 피드백, x트랙이 0으로 수렴하는지 확인하는 데 사용
    // 튜닝 목적으로 적분기가 변경될 때 _prev를 사용하면 적분기를 삭제하는 것이 도움됨
    float _L1_xtrack_i = 0;
    AP_Float _L1_xtrack_i_gain;
    float _L1_xtrack_i_gain_prev = 0;
    uint32_t _last_update_waypoint_us;
    bool _data_is_stale = true;

    AP_Float _loiter_bank_limit;

    bool _reverse = false;
    float get_yaw() const;
    int32_t get_yaw_sensor() const;
};
