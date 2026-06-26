#pragma once

#include "AC_Geometric_Types.h"

class AC_Geometric_SetpointShaper {
public:
    AC_Geometric_SetpointShaper() = default;

    void set_limits(const AC_Geometric_Setpoint_Shaper_Limits& limits) { _limits = limits; }
    const AC_Geometric_Setpoint_Shaper_Limits& get_limits() const { return _limits; }

    void reset();

    void update(const AC_Geometric_State& state,
                const AC_Geometric_Target& raw_target,
                float dt,
                AC_Geometric_Target& shaped_target);

private:
    void init_from_state(const AC_Geometric_State& state);
    void shape_xy(const Vector3f& goal_ned_m, float dt);
    void shape_z(float goal_z_ned_m, float dt);
    void shape_yaw(float yaw_goal_rad, float dt);

    AC_Geometric_Setpoint_Shaper_Limits _limits;
    Vector3f _pos_ref_ned_m;
    Vector3f _vel_ref_ned_ms;
    Vector3f _accel_ref_ned_mss;
    float _yaw_ref_rad = 0.0f;
    float _yaw_rate_ref_rads = 0.0f;
    float _yaw_accel_ref_radss = 0.0f;
    bool _initialized = false;
};
