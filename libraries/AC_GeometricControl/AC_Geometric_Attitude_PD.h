#pragma once

#include "AC_Geometric_Types.h"

class AC_Geometric_Attitude_PD {
public:
    AC_Geometric_Attitude_PD() = default;

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
    Vector3f _omega_error_filtered_rads;
    Vector3f _integral_error;
    bool _filter_reset = true;
};
