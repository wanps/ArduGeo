#pragma once

#include <AP_Param/AP_Param.h>

#include "AC_Geometric_Types.h"

// Persistent configuration for the translational geometric PID channel.
// Parameter metadata and defaults live beside the implementation while the
// AC_GeometricControl facade supplies the public GEO_ namespace.
class AC_Geometric_Position_PID_Params {
public:
    AC_Geometric_Position_PID_Params();

    static const AP_Param::GroupInfo var_info[];

    AC_Geometric_Position_Gains gains() const;
    AC_Geometric_Position_Filter_Hz filter_hz() const;
    AC_Geometric_Position_Integral_Limits integral_limits() const;
    void convert_legacy_params(uint16_t old_key);

private:
    // Frozen migration table. Do not change mapped member types or add future
    // parameters here.
    static const AP_Param::GroupInfo legacy_var_info[];

    AP_Float _kx_xy;
    AP_Float _kx_z;
    AP_Float _ki_xy;
    AP_Float _ki_z;
    AP_Float _kv_xy;
    AP_Float _kv_z;
    AP_Float _imax_xy;
    AP_Float _imax_z;
    AP_Float _integral_error_p;
    AP_Float _position_error_filt_hz;
    AP_Float _velocity_error_filt_hz;
    AP_Float _omega_c_filt_hz;
    AP_Float _omega_dot_c_filt_hz;
};

// SE(3) translational channel. It converts translational errors and
// feed-forward acceleration into A and applied f. When
// build_attitude_from_position is true it constructs R_c from the feasible
// force direction; otherwise it passes through the direct attitude reference.
class AC_Geometric_Position_PID {
public:
    AC_Geometric_Position_PID() = default;

    void set_gains(const AC_Geometric_Position_Gains& gains) { _gains = gains; }
    const AC_Geometric_Position_Gains& get_gains() const { return _gains; }

    void set_filter_hz(const AC_Geometric_Position_Filter_Hz& filter_hz) { _filter_hz = filter_hz; }
    const AC_Geometric_Position_Filter_Hz& get_filter_hz() const { return _filter_hz; }

    void set_integral_limits(const AC_Geometric_Position_Integral_Limits& integral_limits) { _integral_limits = integral_limits; }
    const AC_Geometric_Position_Integral_Limits& get_integral_limits() const { return _integral_limits; }

    void reset();

    void update(const AC_Geometric_State& state,
                const AC_Geometric_Target& target,
                float dt,
                AC_Geometric_Position_Output& output);

private:
    AC_Geometric_Position_Gains _gains;
    AC_Geometric_Position_Filter_Hz _filter_hz;
    AC_Geometric_Position_Integral_Limits _integral_limits;

    // The position integral follows e_I^x = integral(e_v + C_x*e_x). It is
    // stored in SI units and bounded before the Ki term is applied.
    Vector3f _integral_error_m;
    Vector3f _position_error_filtered_m;
    Vector3f _velocity_error_filtered_ms;

    // R_c derivatives are estimated from consecutive commanded attitudes.
    // They are filtered because finite differences can spike when the thrust
    // direction changes quickly.
    Quaternion _last_attitude_target_body_to_ned;
    Vector3f _omega_c_filtered_rads;
    Vector3f _omega_dot_c_filtered_radss;
    bool _filter_reset = true;
    bool _attitude_target_reset = true;
    bool _regularization_derivative_reset = false;
    // Near-zero collective has no useful force direction. Hysteresis keeps the
    // level-yaw attitude regularisation from chattering around its threshold.
    bool _upright_thrust_regularized = false;
};
