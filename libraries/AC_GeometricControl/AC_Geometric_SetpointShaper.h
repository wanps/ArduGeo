#pragma once

#include "AC_Geometric_Types.h"

// Guided reference shaper. It keeps an internal x_d, v_d and a_d state, then
// advances it toward the raw Guided target with ArduPilot's jerk-limited
// square-root helpers before geometric position feedback runs.
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

    // Translational shaping is split into horizontal NED XY and vertical NED Z
    // so climb/descent limits can differ while preserving a single target API.
    void shape_xy(const AC_Geometric_Target& raw_target, float dt);
    void shape_z(const AC_Geometric_Target& raw_target, float dt);

    AC_Geometric_Setpoint_Shaper_Limits _limits;
    Vector3f _pos_ref_ned_m;
    Vector3f _vel_ref_ned_ms;
    Vector3f _accel_ref_ned_mss;
    bool _initialized = false;
};
