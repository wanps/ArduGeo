#include "AC_Geometric_YawShaper.h"

namespace {

constexpr float JERK_FROM_ACCEL_RATIO = 2.0f;

float jerk_from_accel(float accel_max)
{
    return accel_max * JERK_FROM_ACCEL_RATIO;
}

float yaw_from_velocity_xy(const Vector3f& velocity_ned_ms)
{
    return atan2f(velocity_ned_ms.y, velocity_ned_ms.x);
}

float yaw_rate_from_velocity_accel_xy(const Vector3f& velocity_ned_ms,
                                      const Vector3f& accel_ned_mss)
{
    // Same idea as AC_PosControl::calculate_yaw_and_rate_yaw(): remove the
    // forward acceleration component and convert lateral acceleration into a
    // planar turn rate.
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

void AC_Geometric_YawShaper::reset()
{
    _yaw_ref_rad = 0.0f;
    _yaw_rate_ref_rads = 0.0f;
    _yaw_accel_ref_radss = 0.0f;
    _initialized = false;
}

void AC_Geometric_YawShaper::init_from_state(const AC_Geometric_State& state)
{
    float roll_rad;
    float pitch_rad;
    state.attitude_body_to_ned.to_euler(roll_rad, pitch_rad, _yaw_ref_rad);
    _yaw_rate_ref_rads = state.omega_body_rads.z;
    _yaw_accel_ref_radss = 0.0f;
    _initialized = true;
}

void AC_Geometric_YawShaper::pass_through(const AC_Geometric_Target& raw_target, AC_Geometric_Target& shaped_target)
{
    shaped_target.yaw_rad = raw_target.yaw_rad;
    shaped_target.yaw_rate_rads = raw_target.yaw_rate_rads;
    shaped_target.omega_body_rads.z = raw_target.yaw_rate_rads;

    _yaw_ref_rad = raw_target.yaw_rad;
    _yaw_rate_ref_rads = raw_target.yaw_rate_rads;
    _yaw_accel_ref_radss = 0.0f;
}

void AC_Geometric_YawShaper::shape_yaw(float yaw_goal_rad, float yaw_rate_goal_rads, float dt)
{
    const float yaw_rate_max_rads = MAX(_limits.yaw_rate_max_rads, 0.0f);
    const float yaw_accel_max_radss = MAX(_limits.yaw_accel_max_radss, 0.0f);
    const float yaw_jerk_max_radsss = jerk_from_accel(yaw_accel_max_radss);
    if (!is_positive(yaw_rate_max_rads) || !is_positive(yaw_accel_max_radss) || !is_positive(yaw_jerk_max_radsss) || !is_positive(dt)) {
        _yaw_ref_rad = yaw_goal_rad;
        _yaw_rate_ref_rads = yaw_rate_goal_rads;
        _yaw_accel_ref_radss = 0.0f;
        return;
    }

    shape_angle_vel_accel(yaw_goal_rad, yaw_rate_goal_rads, 0.0f,
                          _yaw_ref_rad, _yaw_rate_ref_rads, _yaw_accel_ref_radss,
                          -yaw_rate_max_rads, yaw_rate_max_rads,
                          yaw_accel_max_radss, yaw_jerk_max_radsss, dt, true);
    _yaw_ref_rad = wrap_PI(_yaw_ref_rad + _yaw_rate_ref_rads * dt + 0.5f * _yaw_accel_ref_radss * sq(dt));
    _yaw_rate_ref_rads += _yaw_accel_ref_radss * dt;
}

void AC_Geometric_YawShaper::shape_yaw_from_trajectory(const Vector3f& velocity_ref_ned_ms,
                                                       const Vector3f& accel_ref_ned_mss,
                                                       float dt)
{
    const float min_yaw_speed_ms = MAX(_limits.trajectory_min_speed_ms, 0.05f);
    if (velocity_ref_ned_ms.xy().length() <= min_yaw_speed_ms) {
        // At low speed the horizontal velocity direction is ill-defined; hold
        // the previous yaw reference and command no yaw-rate.
        _yaw_rate_ref_rads = 0.0f;
        _yaw_accel_ref_radss = 0.0f;
        return;
    }

    const float yaw_goal_rad = yaw_from_velocity_xy(velocity_ref_ned_ms);
    const float yaw_rate_goal_rads = yaw_rate_from_velocity_accel_xy(velocity_ref_ned_ms, accel_ref_ned_mss);
    shape_yaw(yaw_goal_rad, yaw_rate_goal_rads, dt);
}

bool AC_Geometric_YawShaper::update(const AC_Geometric_State& state,
                                    const AC_Geometric_Target& raw_target,
                                    const Vector3f& velocity_ref_ned_ms,
                                    const Vector3f& accel_ref_ned_mss,
                                    float dt,
                                    AC_Geometric_Target& shaped_target)
{
    if (!_initialized) {
        init_from_state(state);
    }

    if (raw_target.yaw_from_trajectory) {
        shape_yaw_from_trajectory(velocity_ref_ned_ms, accel_ref_ned_mss, dt);
    } else if (_limits.explicit_yaw_enabled) {
        shape_yaw(raw_target.yaw_rad, raw_target.yaw_rate_rads, dt);
    } else {
        pass_through(raw_target, shaped_target);
        return false;
    }

    shaped_target.yaw_rad = _yaw_ref_rad;
    shaped_target.yaw_rate_rads = _yaw_rate_ref_rads;
    shaped_target.omega_body_rads.z = _yaw_rate_ref_rads;
    return true;
}
