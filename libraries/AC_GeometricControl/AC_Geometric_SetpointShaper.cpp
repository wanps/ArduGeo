#include "AC_Geometric_SetpointShaper.h"

namespace {

Vector2f limit_vector_length(Vector2f value, float max_length)
{
    // Preserve direction while bounding the vector magnitude.
    const float length = value.length();
    if (is_positive(max_length) && length > max_length) {
        value *= max_length / length;
    }
    return value;
}

float sign_not_zero(float value)
{
    return is_negative(value) ? -1.0f : 1.0f;
}

float yaw_from_velocity_xy(const Vector3f& velocity_ned_ms)
{
    return atan2f(velocity_ned_ms.y, velocity_ned_ms.x);
}

float yaw_rate_from_velocity_accel_xy(const Vector3f& velocity_ned_ms,
                                      const Vector3f& accel_ned_mss)
{
    // Same idea as AC_PosControl::calculate_yaw_and_rate_yaw(): remove the
    // forward acceleration component and convert the lateral acceleration into
    // a planar turn rate.
    const Vector2f velocity_xy = velocity_ned_ms.xy();
    const Vector2f accel_xy = accel_ned_mss.xy();
    const float speed_ms = velocity_xy.length();
    if (!is_positive(speed_ms)) {
        return 0.0f;
    }

    const float accel_forward_mss = (accel_xy.x * velocity_xy.x + accel_xy.y * velocity_xy.y) / speed_ms;
    const Vector2f accel_turn_xy = accel_xy - velocity_xy * (accel_forward_mss / speed_ms);
    float yaw_rate_rads = accel_turn_xy.length() / speed_ms;
    if ((accel_turn_xy.y * velocity_xy.x - accel_turn_xy.x * velocity_xy.y) < 0.0f) {
        yaw_rate_rads = -yaw_rate_rads;
    }
    return yaw_rate_rads;
}

}

void AC_Geometric_SetpointShaper::reset()
{
    _pos_ref_ned_m.zero();
    _vel_ref_ned_ms.zero();
    _accel_ref_ned_mss.zero();
    _yaw_ref_rad = 0.0f;
    _yaw_rate_ref_rads = 0.0f;
    _yaw_accel_ref_radss = 0.0f;
    _initialized = false;
}

void AC_Geometric_SetpointShaper::init_from_state(const AC_Geometric_State& state)
{
    // Start every shaped segment from the measured vehicle state to avoid a
    // jump when geometric control is enabled during flight.
    _pos_ref_ned_m = state.position_ned_m;
    _vel_ref_ned_ms = state.velocity_ned_ms;
    _accel_ref_ned_mss.zero();

    float roll_rad;
    float pitch_rad;
    state.attitude_body_to_ned.to_euler(roll_rad, pitch_rad, _yaw_ref_rad);
    _yaw_rate_ref_rads = state.omega_body_rads.z;
    _yaw_accel_ref_radss = 0.0f;
    _initialized = true;
}

void AC_Geometric_SetpointShaper::shape_xy(const Vector3f& goal_ned_m, float dt)
{
    const float vel_max_ms = MAX(_limits.vel_xy_max_ms, 0.0f);
    const float accel_max_mss = MAX(_limits.accel_xy_max_mss, 0.0f);
    if (!is_positive(vel_max_ms) || !is_positive(accel_max_mss) || !is_positive(dt)) {
        // Disabled or invalid limits mean "snap to target" for the reference.
        _pos_ref_ned_m.x = goal_ned_m.x;
        _pos_ref_ned_m.y = goal_ned_m.y;
        _vel_ref_ned_ms.x = 0.0f;
        _vel_ref_ned_ms.y = 0.0f;
        _accel_ref_ned_mss.x = 0.0f;
        _accel_ref_ned_mss.y = 0.0f;
        return;
    }

    const Vector2f pos_ref_xy{_pos_ref_ned_m.x, _pos_ref_ned_m.y};
    const Vector2f vel_ref_xy{_vel_ref_ned_ms.x, _vel_ref_ned_ms.y};
    const Vector2f goal_xy{goal_ned_m.x, goal_ned_m.y};
    const Vector2f error_xy = goal_xy - pos_ref_xy;
    const float distance_m = error_xy.length();

    Vector2f vel_desired_xy;
    vel_desired_xy.zero();
    if (is_positive(distance_m)) {
        // Braking-distance shaper: the reference speed is never higher than
        // the speed from which it can stop within the remaining distance.
        const float vel_stop_ms = safe_sqrt(2.0f * accel_max_mss * distance_m);
        vel_desired_xy = error_xy * (MIN(vel_max_ms, vel_stop_ms) / distance_m);
    }

    // Acceleration limiting is applied by limiting the velocity increment.
    const Vector2f delta_vel_xy = limit_vector_length(vel_desired_xy - vel_ref_xy, accel_max_mss * dt);
    const Vector2f accel_xy = delta_vel_xy / dt;
    const Vector2f next_pos_xy = pos_ref_xy + vel_ref_xy * dt + accel_xy * (0.5f * sq(dt));
    const Vector2f next_vel_xy = vel_ref_xy + delta_vel_xy;

    _pos_ref_ned_m.x = next_pos_xy.x;
    _pos_ref_ned_m.y = next_pos_xy.y;
    _vel_ref_ned_ms.x = next_vel_xy.x;
    _vel_ref_ned_ms.y = next_vel_xy.y;
    _accel_ref_ned_mss.x = accel_xy.x;
    _accel_ref_ned_mss.y = accel_xy.y;
}

void AC_Geometric_SetpointShaper::shape_z(float goal_z_ned_m, float dt)
{
    const float accel_max_mss = MAX(_limits.accel_z_max_mss, 0.0f);
    if (!is_positive(accel_max_mss) || !is_positive(dt)) {
        // Disabled or invalid limits mean "snap to target" for the reference.
        _pos_ref_ned_m.z = goal_z_ned_m;
        _vel_ref_ned_ms.z = 0.0f;
        _accel_ref_ned_mss.z = 0.0f;
        return;
    }

    const float error_z_m = goal_z_ned_m - _pos_ref_ned_m.z;
    // NED positive Z is down, so negative Z error means upward travel.
    const float vel_max_ms = is_negative(error_z_m) ? MAX(_limits.vel_up_max_ms, 0.0f) : MAX(_limits.vel_down_max_ms, 0.0f);
    if (!is_positive(vel_max_ms)) {
        _pos_ref_ned_m.z = goal_z_ned_m;
        _vel_ref_ned_ms.z = 0.0f;
        _accel_ref_ned_mss.z = 0.0f;
        return;
    }

    const float vel_stop_ms = safe_sqrt(2.0f * accel_max_mss * fabsf(error_z_m));
    const float vel_desired_ms = sign_not_zero(error_z_m) * MIN(vel_max_ms, vel_stop_ms);
    const float delta_vel_ms = constrain_float(vel_desired_ms - _vel_ref_ned_ms.z, -accel_max_mss * dt, accel_max_mss * dt);
    const float accel_mss = delta_vel_ms / dt;

    _pos_ref_ned_m.z += _vel_ref_ned_ms.z * dt + 0.5f * accel_mss * sq(dt);
    _vel_ref_ned_ms.z += delta_vel_ms;
    _accel_ref_ned_mss.z = accel_mss;
}

void AC_Geometric_SetpointShaper::shape_yaw(float yaw_goal_rad, float yaw_rate_goal_rads, float dt)
{
    const float yaw_rate_max_rads = MAX(_limits.yaw_rate_max_rads, 0.0f);
    const float yaw_accel_max_radss = MAX(_limits.yaw_accel_max_radss, 0.0f);
    if (!is_positive(yaw_rate_max_rads) || !is_positive(yaw_accel_max_radss) || !is_positive(dt)) {
        // With shaping disabled, pass explicit yaw references through exactly.
        _yaw_ref_rad = yaw_goal_rad;
        _yaw_rate_ref_rads = yaw_rate_goal_rads;
        _yaw_accel_ref_radss = 0.0f;
        return;
    }

    const float yaw_error_rad = wrap_PI(yaw_goal_rad - _yaw_ref_rad);
    // Angular counterpart of the position braking-distance shaper.
    const float yaw_rate_stop_rads = safe_sqrt(2.0f * yaw_accel_max_radss * fabsf(yaw_error_rad));
    const float yaw_rate_correction_rads = sign_not_zero(yaw_error_rad) * MIN(yaw_rate_max_rads, yaw_rate_stop_rads);
    const float yaw_rate_desired_rads = constrain_float(yaw_rate_goal_rads + yaw_rate_correction_rads,
                                                        -yaw_rate_max_rads,
                                                        yaw_rate_max_rads);
    const float delta_rate_rads = constrain_float(yaw_rate_desired_rads - _yaw_rate_ref_rads,
                                                  -yaw_accel_max_radss * dt,
                                                  yaw_accel_max_radss * dt);
    _yaw_accel_ref_radss = delta_rate_rads / dt;
    _yaw_ref_rad = wrap_PI(_yaw_ref_rad + _yaw_rate_ref_rads * dt + 0.5f * _yaw_accel_ref_radss * sq(dt));
    _yaw_rate_ref_rads += delta_rate_rads;
}

void AC_Geometric_SetpointShaper::shape_yaw_from_trajectory(float dt)
{
    const float vel_xy_max_ms = MAX(_limits.vel_xy_max_ms, 0.0f);
    const float min_yaw_speed_ms = MAX(vel_xy_max_ms * 0.05f, 0.05f);
    if (_vel_ref_ned_ms.xy().length() <= min_yaw_speed_ms) {
        // At low speed the horizontal velocity direction is ill-defined; hold
        // the previous yaw reference and command no yaw-rate.
        _yaw_rate_ref_rads = 0.0f;
        _yaw_accel_ref_radss = 0.0f;
        return;
    }

    const float yaw_goal_rad = yaw_from_velocity_xy(_vel_ref_ned_ms);
    const float yaw_rate_goal_rads = yaw_rate_from_velocity_accel_xy(_vel_ref_ned_ms, _accel_ref_ned_mss);
    shape_yaw(yaw_goal_rad, yaw_rate_goal_rads, dt);
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
    shape_xy(raw_target.position_ned_m, dt);
    shape_z(raw_target.position_ned_m.z, dt);

    shaped_target.position_ned_m = _pos_ref_ned_m;
    shaped_target.velocity_ned_ms = _vel_ref_ned_ms;
    shaped_target.accel_ned_mss = _accel_ref_ned_mss;
    if (raw_target.yaw_from_trajectory) {
        // GoToLocation yaw-follow: derive heading from the same shaped
        // reference that feeds the geometric position channel.
        shape_yaw_from_trajectory(dt);
        shaped_target.yaw_rad = _yaw_ref_rad;
        shaped_target.yaw_rate_rads = _yaw_rate_ref_rads;
        shaped_target.omega_body_rads.z = _yaw_rate_ref_rads;
    } else if (_limits.yaw_enabled) {
        // Explicit yaw commands can optionally be shaped through the same
        // angular velocity and acceleration limits.
        shape_yaw(raw_target.yaw_rad, raw_target.yaw_rate_rads, dt);
        shaped_target.yaw_rad = _yaw_ref_rad;
        shaped_target.yaw_rate_rads = _yaw_rate_ref_rads;
        shaped_target.omega_body_rads.z = _yaw_rate_ref_rads;
    } else {
        // When yaw shaping is disabled, keep the caller's yaw/yaw-rate command
        // and update the internal yaw cache for the next shaped segment.
        _yaw_ref_rad = raw_target.yaw_rad;
        _yaw_rate_ref_rads = raw_target.yaw_rate_rads;
        _yaw_accel_ref_radss = 0.0f;
    }
}
