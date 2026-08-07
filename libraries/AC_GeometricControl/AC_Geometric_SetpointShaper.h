#pragma once

#include <AP_Param/AP_Param.h>

#include "AC_Geometric_Types.h"

// Persistent limits for the Guided translational reference shaper. SHAPE_EN
// is the shared gate for both translational and yaw shaping.
class AC_Geometric_SetpointShaper_Params {
public:
    AC_Geometric_SetpointShaper_Params();

    static const AP_Param::GroupInfo var_info[];

    bool enabled() const { return _enabled.get() != 0; }
    AC_Geometric_Setpoint_Shaper_Limits limits() const;
    void convert_legacy_params(uint16_t old_key);

private:
    // Frozen migration table. Do not change mapped member types or add future
    // parameters here.
    static const AP_Param::GroupInfo legacy_var_info[];

    AP_Int8 _enabled;
    AP_Float _vel_xy_max_ms;
    AP_Float _accel_xy_max_mss;
    AP_Float _vel_up_max_ms;
    AP_Float _vel_down_max_ms;
    AP_Float _accel_z_max_mss;
};

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
