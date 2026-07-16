#pragma once

#include "AC_Geometric_Types.h"

// Lee/Gao SE(3) position channel. It converts translational state errors and
// feed-forward acceleration into a commanded resultant force, then constructs
// the commanded attitude R_c that aligns body thrust with that force.
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

    // The position integral follows e_XI = integral(e_v + c_x e_x). It is
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
