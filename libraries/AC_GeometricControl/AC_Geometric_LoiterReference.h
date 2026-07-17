#pragma once

#include "AC_Geometric_Types.h"

#include <AP_Param/AP_Param.h>

// Mode-specific pilot-command profile. These values are intentionally
// separate from the Guided setpoint shaper and from the native Loiter
// position-correction controller.
struct AC_Geometric_LoiterReference_Profile {
    float speed_xy_max_ms = 0.0f;
    float accel_xy_max_mss = 0.0f;
    float jerk_xy_max_msss = 0.0f;
    float brake_delay_s = 0.0f;
    float brake_accel_max_mss = 0.0f;
    float brake_jerk_max_msss = 0.0f;
    float jerk_z_max_msss = 0.0f;
    float brake_accel_z_max_mss = 0.0f;
    float brake_jerk_z_max_msss = 0.0f;
    float yaw_accel_max_radss = 0.0f;
    float yaw_jerk_max_radsss = 0.0f;
    float yaw_brake_accel_max_radss = 0.0f;
    float yaw_brake_jerk_max_radsss = 0.0f;
};

class AC_Geometric_LoiterReference_Params {
public:
    AC_Geometric_LoiterReference_Params();

    static const AP_Param::GroupInfo var_info[];
    AC_Geometric_LoiterReference_Profile get() const;

private:
    AP_Float _speed_xy_max_ms;
    AP_Float _accel_xy_max_mss;
    AP_Float _jerk_xy_max_msss;
    AP_Float _brake_delay_s;
    AP_Float _brake_accel_max_mss;
    AP_Float _brake_jerk_max_msss;
    AP_Float _jerk_z_max_msss;
    AP_Float _brake_accel_z_max_mss;
    AP_Float _brake_jerk_z_max_msss;
    AP_Float _yaw_accel_max_radss;
    AP_Float _yaw_jerk_max_radsss;
    AP_Float _yaw_brake_accel_max_radss;
    AP_Float _yaw_brake_jerk_max_radsss;
};

// Controller-independent pilot Loiter reference limits. All limits are
// positive magnitudes; the generated translational reference uses NED.
struct AC_Geometric_LoiterReference_Limits {
    float speed_xy_max_ms = 0.0f;
    // Pilot/drag shaping limit and final physical horizontal limit are kept
    // separate so a slow pilot profile does not silently disable the Loiter
    // braking parameters.
    float accel_xy_max_mss = 0.0f;
    float accel_xy_total_max_mss = 0.0f;
    float jerk_xy_max_msss = 0.0f;

    float brake_delay_s = 0.0f;
    float brake_accel_max_mss = 0.0f;
    float brake_jerk_max_msss = 0.0f;

    float speed_up_max_ms = 0.0f;
    float speed_down_max_ms = 0.0f;
    float accel_z_max_mss = 0.0f;
    float jerk_z_max_msss = 0.0f;
    float brake_accel_z_max_mss = 0.0f;
    float brake_jerk_z_max_msss = 0.0f;

    float yaw_rate_max_rads = 0.0f;
    float yaw_accel_max_radss = 0.0f;
    float yaw_jerk_max_radsss = 0.0f;
    float yaw_brake_accel_max_radss = 0.0f;
    float yaw_brake_jerk_max_radsss = 0.0f;
};

struct AC_Geometric_LoiterReference_Input {
    // Pilot horizontal acceleration after RC deadzone, Simple-mode rotation
    // and conversion into the earth-frame NE axes.
    Vector2f pilot_accel_ne_mss;
    bool pilot_xy_active = false;
    bool coordinated_turn = false;

    // Public climb-rate convention is Up-positive. The generated NED Z
    // velocity is therefore negative while climbing.
    float climb_rate_up_ms = 0.0f;
    bool pilot_z_active = false;

    // Optional absolute NED Z target used by the Loiter takeoff phase.
    bool use_z_position_target = false;
    float position_z_target_ned_m = 0.0f;

    float yaw_rate_rads = 0.0f;
    // True only while the raw pilot yaw stick is outside its deadzone.  The
    // shaped residual yaw rate after release must not keep horizontal or yaw
    // braking disabled.
    bool pilot_yaw_active = false;
};

struct AC_Geometric_LoiterReference_Status {
    bool braking = false;
    bool speed_xy_limited = false;
    bool speed_z_limited = false;
    bool z_braking = false;
    bool z_settled = false;
    bool yaw_rate_limited = false;
    bool yaw_braking = false;
    bool yaw_settled = false;
    bool xy_settled = false;
    bool velocity_constraint_applied = false;
    bool vertical_position_anchored = false;
};

// Generates one internally consistent Loiter PVA/yaw reference from pilot
// motion requests, including dedicated neutral/reversal braking. It owns no
// feedback-controller state and has no dependency on AC_PosControl,
// AC_AttitudeControl or AP_Motors.
class AC_Geometric_LoiterReference {
public:
    AC_Geometric_LoiterReference() = default;

    void reset();
    bool reset(const AC_Geometric_State& state, float yaw_rad, float yaw_rate_rads = 0.0f);

    bool update(const AC_Geometric_LoiterReference_Input& input,
                const AC_Geometric_LoiterReference_Limits& limits,
                float dt_s,
                AC_Geometric_Target& target,
                AC_Geometric_LoiterReference_Status& status);

    // Applies an external safety velocity adjustment (for example fence or
    // proximity avoidance) to the most recent update before the target is
    // consumed.  Position and acceleration are recomputed from the pre-update
    // state so the final target remains discretely PVA-consistent.
    bool apply_velocity_constraint(const Vector3f& velocity_ned_ms,
                                   AC_Geometric_Target& target,
                                   AC_Geometric_LoiterReference_Status& status);

    // Re-anchors only the vertical position component after an update.  This
    // is used when the mode intentionally owns vertical velocity/acceleration
    // while position follows the measured NED origin.  The current velocity
    // and acceleration references are preserved and target is rewritten.
    bool anchor_position_z(postype_t position_z_ned_m,
                           AC_Geometric_Target& target,
                           AC_Geometric_LoiterReference_Status& status);

    // Applies EKF-frame reset deltas without resetting shaper state.  Both the
    // live reference and the pending velocity-constraint integration base are
    // translated so a post-update constraint cannot undo the frame shift.
    bool shift_reference(const Vector3f& position_delta_ned_m, float yaw_delta_rad);

    bool initialized() const { return _initialized; }
    const Vector3p& position_ref_ned_m() const { return _position_ref_ned_m; }
    const Vector3f& velocity_ref_ned_ms() const { return _velocity_ref_ned_ms; }
    const Vector3f& accel_ref_ned_mss() const { return _accel_ref_ned_mss; }
    float yaw_ref_rad() const { return _yaw_ref_rad; }
    float yaw_rate_ref_rads() const { return _yaw_rate_ref_rads; }

private:
    bool limits_valid(const AC_Geometric_LoiterReference_Limits& limits) const;
    void write_target(AC_Geometric_Target& target) const;

    Vector3p _position_ref_ned_m;
    Vector3f _velocity_ref_ned_ms;
    Vector3f _accel_ref_ned_mss;
    Vector3f _accel_shaper_ned_mss;

    float _yaw_ref_rad = 0.0f;
    float _yaw_rate_ref_rads = 0.0f;
    float _yaw_accel_ref_radss = 0.0f;
    float _yaw_accel_shaper_radss = 0.0f;

    float _neutral_xy_time_s = 0.0f;
    float _brake_accel_mss = 0.0f;
    bool _z_brake_settled = false;

    Vector3p _constraint_base_position_ned_m;
    Vector3f _constraint_base_velocity_ned_ms;
    AC_Geometric_LoiterReference_Limits _last_limits;
    float _last_update_dt_s = 0.0f;
    bool _velocity_constraint_pending = false;
    bool _initialized = false;
};
