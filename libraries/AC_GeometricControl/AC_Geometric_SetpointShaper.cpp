#include "AC_Geometric_SetpointShaper.h"

namespace {

constexpr float SETTLE_POS_XY_M = 0.05f;
constexpr float SETTLE_POS_Z_M = 0.02f;
constexpr float SETTLE_VEL_MS = 0.05f;
constexpr float SETTLE_ACCEL_MSS = 0.05f;
constexpr float JERK_FROM_ACCEL_RATIO = 2.0f;

float jerk_from_accel(float accel_max)
{
    return accel_max * JERK_FROM_ACCEL_RATIO;
}

bool should_match_target(float position_error,
                         float reference_velocity,
                         float target_velocity,
                         float reference_acceleration,
                         float target_acceleration)
{
    return fabsf(position_error) <= SETTLE_POS_Z_M &&
           fabsf(reference_velocity - target_velocity) <= SETTLE_VEL_MS &&
           fabsf(reference_acceleration - target_acceleration) <= SETTLE_ACCEL_MSS;
}

bool should_match_target_xy(const Vector2f& position_error,
                            const Vector2f& reference_velocity,
                            const Vector2f& target_velocity,
                            const Vector2f& reference_acceleration,
                            const Vector2f& target_acceleration)
{
    return position_error.length() <= SETTLE_POS_XY_M &&
           (reference_velocity - target_velocity).length() <= SETTLE_VEL_MS &&
           (reference_acceleration - target_acceleration).length() <= SETTLE_ACCEL_MSS;
}

}

void AC_Geometric_SetpointShaper::reset()
{
    _pos_ref_ned_m.zero();
    _vel_ref_ned_ms.zero();
    _accel_ref_ned_mss.zero();
    _initialized = false;
}

void AC_Geometric_SetpointShaper::init_from_state(const AC_Geometric_State& state)
{
    // Start every shaped segment from the measured vehicle state to avoid a
    // jump when geometric control is enabled during flight.
    _pos_ref_ned_m = state.position_ned_m;
    _vel_ref_ned_ms = state.velocity_ned_ms;
    _accel_ref_ned_mss.zero();

    _initialized = true;
}

void AC_Geometric_SetpointShaper::shape_xy(const AC_Geometric_Target& raw_target, float dt)
{
    const float vel_max_ms = MAX(_limits.vel_xy_max_ms, 0.0f);
    const float accel_max_mss = MAX(_limits.accel_xy_max_mss, 0.0f);
    const float jerk_max_msss = jerk_from_accel(accel_max_mss);
    if (!is_positive(vel_max_ms) || !is_positive(accel_max_mss) || !is_positive(jerk_max_msss) || !is_positive(dt)) {
        // Disabled or invalid limits mean "snap to target" for the reference.
        _pos_ref_ned_m.x = raw_target.position_ned_m.x;
        _pos_ref_ned_m.y = raw_target.position_ned_m.y;
        _vel_ref_ned_ms.x = 0.0f;
        _vel_ref_ned_ms.y = 0.0f;
        _accel_ref_ned_mss.x = 0.0f;
        _accel_ref_ned_mss.y = 0.0f;
        return;
    }

    const Vector2f pos_ref_xy{_pos_ref_ned_m.x, _pos_ref_ned_m.y};
    const Vector2f vel_ref_xy{_vel_ref_ned_ms.x, _vel_ref_ned_ms.y};
    const Vector2f goal_xy{raw_target.position_ned_m.x, raw_target.position_ned_m.y};
    const Vector2f raw_vel_xy{raw_target.velocity_ned_ms.x, raw_target.velocity_ned_ms.y};
    const Vector2f raw_accel_xy{raw_target.accel_ned_mss.x, raw_target.accel_ned_mss.y};
    const Vector2f accel_ref_xy{_accel_ref_ned_mss.x, _accel_ref_ned_mss.y};
    const Vector2f error_xy = goal_xy - pos_ref_xy;
    if (should_match_target_xy(error_xy, vel_ref_xy, raw_vel_xy,
                               accel_ref_xy, raw_accel_xy)) {
        _pos_ref_ned_m.x = raw_target.position_ned_m.x;
        _pos_ref_ned_m.y = raw_target.position_ned_m.y;
        _vel_ref_ned_ms.x = raw_target.velocity_ned_ms.x;
        _vel_ref_ned_ms.y = raw_target.velocity_ned_ms.y;
        _accel_ref_ned_mss.x = raw_target.accel_ned_mss.x;
        _accel_ref_ned_mss.y = raw_target.accel_ned_mss.y;
        return;
    }

    Vector2p next_pos_xy = pos_ref_xy.topostype();
    Vector2f next_vel_xy = vel_ref_xy;
    Vector2f next_accel_xy{_accel_ref_ned_mss.x, _accel_ref_ned_mss.y};
    const Vector2p goal_xy_p = goal_xy.topostype();
    shape_pos_vel_accel_xy(goal_xy_p, raw_vel_xy, raw_accel_xy,
                           next_pos_xy, next_vel_xy, next_accel_xy,
                           vel_max_ms, accel_max_mss, jerk_max_msss, dt, true);

    Vector2f limit_xy;
    limit_xy.zero();
    Vector2f error_zero_xy;
    error_zero_xy.zero();
    update_pos_vel_accel_xy(next_pos_xy, next_vel_xy, next_accel_xy, dt, limit_xy, error_zero_xy, error_zero_xy);

    const Vector2f next_error_xy = goal_xy - next_pos_xy.tofloat();
    if (should_match_target_xy(next_error_xy, next_vel_xy, raw_vel_xy,
                               next_accel_xy, raw_accel_xy)) {
        _pos_ref_ned_m.x = raw_target.position_ned_m.x;
        _pos_ref_ned_m.y = raw_target.position_ned_m.y;
        _vel_ref_ned_ms.x = raw_target.velocity_ned_ms.x;
        _vel_ref_ned_ms.y = raw_target.velocity_ned_ms.y;
        _accel_ref_ned_mss.x = raw_target.accel_ned_mss.x;
        _accel_ref_ned_mss.y = raw_target.accel_ned_mss.y;
        return;
    }

    _pos_ref_ned_m.x = next_pos_xy.x;
    _pos_ref_ned_m.y = next_pos_xy.y;
    _vel_ref_ned_ms.x = next_vel_xy.x;
    _vel_ref_ned_ms.y = next_vel_xy.y;
    _accel_ref_ned_mss.x = next_accel_xy.x;
    _accel_ref_ned_mss.y = next_accel_xy.y;
}

void AC_Geometric_SetpointShaper::shape_z(const AC_Geometric_Target& raw_target, float dt)
{
    const float accel_max_mss = MAX(_limits.accel_z_max_mss, 0.0f);
    const float jerk_max_msss = jerk_from_accel(accel_max_mss);
    if (!is_positive(accel_max_mss) || !is_positive(jerk_max_msss) || !is_positive(dt)) {
        // Disabled or invalid limits mean "snap to target" for the reference.
        _pos_ref_ned_m.z = raw_target.position_ned_m.z;
        _vel_ref_ned_ms.z = 0.0f;
        _accel_ref_ned_mss.z = 0.0f;
        return;
    }

    const float goal_z_ned_m = raw_target.position_ned_m.z;
    const float error_z_m = goal_z_ned_m - _pos_ref_ned_m.z;
    if (should_match_target(error_z_m,
                            _vel_ref_ned_ms.z,
                            raw_target.velocity_ned_ms.z,
                            _accel_ref_ned_mss.z,
                            raw_target.accel_ned_mss.z)) {
        _pos_ref_ned_m.z = goal_z_ned_m;
        _vel_ref_ned_ms.z = raw_target.velocity_ned_ms.z;
        _accel_ref_ned_mss.z = raw_target.accel_ned_mss.z;
        return;
    }

    // NED positive Z is down, so negative Z error means upward travel.
    const bool moving_up = is_negative(error_z_m) ||
                           (is_zero(error_z_m) && is_negative(raw_target.velocity_ned_ms.z));
    const float vel_max_ms = moving_up ? MAX(_limits.vel_up_max_ms, 0.0f) : MAX(_limits.vel_down_max_ms, 0.0f);
    if (!is_positive(vel_max_ms)) {
        _pos_ref_ned_m.z = goal_z_ned_m;
        _vel_ref_ned_ms.z = 0.0f;
        _accel_ref_ned_mss.z = 0.0f;
        return;
    }

    postype_t next_pos_z_m = _pos_ref_ned_m.z;
    float next_vel_z_ms = _vel_ref_ned_ms.z;
    float next_accel_z_mss = _accel_ref_ned_mss.z;
    shape_pos_vel_accel(static_cast<postype_t>(goal_z_ned_m), raw_target.velocity_ned_ms.z, raw_target.accel_ned_mss.z,
                        next_pos_z_m, next_vel_z_ms, next_accel_z_mss,
                        -vel_max_ms, vel_max_ms,
                        -accel_max_mss, accel_max_mss,
                        jerk_max_msss, dt, true);

    update_pos_vel_accel(next_pos_z_m, next_vel_z_ms, next_accel_z_mss, dt, 0.0f, 0.0f, 0.0f);
    if (should_match_target(goal_z_ned_m - next_pos_z_m,
                            next_vel_z_ms,
                            raw_target.velocity_ned_ms.z,
                            next_accel_z_mss,
                            raw_target.accel_ned_mss.z)) {
        _pos_ref_ned_m.z = goal_z_ned_m;
        _vel_ref_ned_ms.z = raw_target.velocity_ned_ms.z;
        _accel_ref_ned_mss.z = raw_target.accel_ned_mss.z;
        return;
    }

    _pos_ref_ned_m.z = next_pos_z_m;
    _vel_ref_ned_ms.z = next_vel_z_ms;
    _accel_ref_ned_mss.z = next_accel_z_mss;
}

void AC_Geometric_SetpointShaper::update(const AC_Geometric_State& state,
                                         const AC_Geometric_Target& raw_target,
                                         float dt,
                                         AC_Geometric_Target& shaped_target)
{
    if (!_initialized) {
        init_from_state(state);
    }

    shaped_target = raw_target;
    shape_xy(raw_target, dt);
    shape_z(raw_target, dt);

    shaped_target.position_ned_m = _pos_ref_ned_m;
    shaped_target.velocity_ned_ms = _vel_ref_ned_ms;
    shaped_target.accel_ned_mss = _accel_ref_ned_mss;
}
