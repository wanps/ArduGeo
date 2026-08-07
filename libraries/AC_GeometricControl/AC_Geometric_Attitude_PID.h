#pragma once

#include <AP_Param/AP_Param.h>

#include "AC_Geometric_Types.h"

// Persistent configuration for the Lee SO(3) PID attitude channel.
class AC_Geometric_Attitude_PID_Params {
public:
    AC_Geometric_Attitude_PID_Params();

    static const AP_Param::GroupInfo var_info[];

    AC_Geometric_Attitude_Gains gains() const;
    AC_Geometric_Attitude_Model model() const;
    AC_Geometric_Attitude_Filter_Hz filter_hz() const;
    AC_Geometric_Attitude_Integral_Limits integral_limits() const;
    void convert_legacy_params(uint16_t old_key);

private:
    // Frozen migration table. Do not change mapped member types or add future
    // parameters here.
    static const AP_Param::GroupInfo legacy_var_info[];

    AP_Float _kr_x;
    AP_Float _kr_y;
    AP_Float _kr_z;
    AP_Float _ko_x;
    AP_Float _ko_y;
    AP_Float _ko_z;
    AP_Float _ki_x;
    AP_Float _ki_y;
    AP_Float _ki_z;
    AP_Float _imax_x;
    AP_Float _imax_y;
    AP_Float _imax_z;
    AP_Float _integral_error_p;
    AP_Float _inertia_x;
    AP_Float _inertia_y;
    AP_Float _inertia_z;
    AP_Float _omega_error_filt_hz;
};

// Lee SO(3) attitude and angular-rate channel. It tracks either a direct
// attitude target R_d or the position-generated R_c and produces the
// geometric moment proxy M for the output mapper.
class AC_Geometric_Attitude_PID {
public:
    AC_Geometric_Attitude_PID() = default;

    void set_gains(const AC_Geometric_Attitude_Gains& gains) { _gains = gains; }
    const AC_Geometric_Attitude_Gains& get_gains() const { return _gains; }

    void set_model(const AC_Geometric_Attitude_Model& model) { _model = model; }
    const AC_Geometric_Attitude_Model& get_model() const { return _model; }

    void set_filter_hz(const AC_Geometric_Attitude_Filter_Hz& filter_hz) { _filter_hz = filter_hz; }
    const AC_Geometric_Attitude_Filter_Hz& get_filter_hz() const { return _filter_hz; }

    void set_integral_limits(const AC_Geometric_Attitude_Integral_Limits& integral_limits) { _integral_limits = integral_limits; }
    const AC_Geometric_Attitude_Integral_Limits& get_integral_limits() const { return _integral_limits; }

    void reset();

    void update(const AC_Geometric_State& state,
                const AC_Geometric_Target& target,
                float dt,
                AC_Geometric_Attitude_Output& output);

private:
    AC_Geometric_Attitude_Gains _gains;
    AC_Geometric_Attitude_Model _model;
    AC_Geometric_Attitude_Filter_Hz _filter_hz;
    AC_Geometric_Attitude_Integral_Limits _integral_limits;

    // Optional filtered angular-rate error and geometric integral state e_I^R.
    // Both are reset when the controller is disabled or reinitialised.
    Vector3f _omega_error_filtered_rads;
    Vector3f _integral_error;
    bool _filter_reset = true;
};
