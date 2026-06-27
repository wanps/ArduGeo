#pragma once

#include "AC_Geometric_Types.h"

// Independent yaw reference shaper for the geometric SE(3) path. It follows
// ArduPilot's angle/velocity/acceleration shaping helper but does not read or
// write AC_AttitudeControl yaw target state.
class AC_Geometric_YawShaper {
public:
    AC_Geometric_YawShaper() = default;

    void set_limits(const AC_Geometric_Yaw_Shaper_Limits& limits) { _limits = limits; }
    const AC_Geometric_Yaw_Shaper_Limits& get_limits() const { return _limits; }

    void reset();

    // Updates yaw fields in shaped_target. Trajectory yaw uses the already
    // shaped translational velocity and acceleration from the position shaper.
    bool update(const AC_Geometric_State& state,
                const AC_Geometric_Target& raw_target,
                const Vector3f& velocity_ref_ned_ms,
                const Vector3f& accel_ref_ned_mss,
                float dt,
                AC_Geometric_Target& shaped_target);

private:
    void init_from_state(const AC_Geometric_State& state);
    void pass_through(const AC_Geometric_Target& raw_target, AC_Geometric_Target& shaped_target);
    void shape_yaw(float yaw_goal_rad, float yaw_rate_goal_rads, float dt);
    void shape_yaw_from_trajectory(const Vector3f& velocity_ref_ned_ms,
                                   const Vector3f& accel_ref_ned_mss,
                                   float dt);

    AC_Geometric_Yaw_Shaper_Limits _limits;
    float _yaw_ref_rad = 0.0f;
    float _yaw_rate_ref_rads = 0.0f;
    float _yaw_accel_ref_radss = 0.0f;
    bool _initialized = false;
};
