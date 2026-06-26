#pragma once

#include "AC_Geometric_Types.h"

class AC_Geometric_Position_PID {
public:
    AC_Geometric_Position_PID() = default;

    void set_gains(const AC_Geometric_Position_Gains& gains) { _gains = gains; }
    const AC_Geometric_Position_Gains& get_gains() const { return _gains; }

    void set_filter_hz(const AC_Geometric_Position_Filter_Hz& filter_hz) { _filter_hz = filter_hz; }
    const AC_Geometric_Position_Filter_Hz& get_filter_hz() const { return _filter_hz; }

    void reset();

    void update(const AC_Geometric_State& state,
                const AC_Geometric_Target& target,
                float dt,
                AC_Geometric_Position_Output& output);

private:
    AC_Geometric_Position_Gains _gains;
    AC_Geometric_Position_Filter_Hz _filter_hz;
    Vector3f _position_error_integral_m;
    Vector3f _position_error_filtered_m;
    Vector3f _velocity_error_filtered_ms;
    bool _filter_reset = true;
};
