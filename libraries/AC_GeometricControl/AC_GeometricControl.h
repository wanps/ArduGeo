#pragma once

#include <AP_Common/AP_Common.h>

#include "AC_Geometric_Attitude_PD.h"
#include "AC_Geometric_Position_PID.h"
#include "AC_Geometric_Types.h"

class AC_GeometricControl {
public:
    AC_GeometricControl() = default;
    CLASS_NO_COPY(AC_GeometricControl);

    // Clear controller integrators and cached outputs.
    void reset();

    // The geometric path is opt-in. Disabling clears output so callers do not
    // accidentally consume stale geometric commands.
    void set_enabled(bool enabled);
    bool enabled() const { return _enabled; }

    void set_position_gains(const AC_Geometric_Position_Gains& gains) { _position_pid.set_gains(gains); }
    void set_attitude_gains(const AC_Geometric_Attitude_Gains& gains) { _attitude_pd.set_gains(gains); }

    // Run the geometric position-to-attitude and attitude PD cascade.
    // This currently computes internal outputs only; it does not write motors.
    void update(const AC_Geometric_State& state,
                const AC_Geometric_Target& target,
                float dt);

    const AC_Geometric_Output& get_output() const { return _output; }

private:
    bool _enabled = false;
    AC_Geometric_Position_PID _position_pid;
    AC_Geometric_Attitude_PD _attitude_pd;
    AC_Geometric_Output _output;
};
