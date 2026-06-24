#include "AC_GeometricControl.h"

void AC_GeometricControl::reset()
{
    _position_pid.reset();
    _output = {};
}

void AC_GeometricControl::set_enabled(bool enabled)
{
    if (_enabled && !enabled) {
        reset();
    }
    _enabled = enabled;
}

void AC_GeometricControl::update(const AC_Geometric_State& state,
                                 const AC_Geometric_Target& target,
                                 float dt)
{
    if (!_enabled) {
        return;
    }

    // The position channel owns the SE(3) coupling: it will eventually build
    // thrust and desired attitude from position/velocity/heading targets.
    _position_pid.update(state, target, dt, _output.position);

    // Feed the position-generated desired attitude into the SO(3) attitude channel.
    AC_Geometric_Target attitude_target = target;
    attitude_target.attitude_body_to_ned = _output.position.attitude_body_to_ned;
    attitude_target.omega_body_rads = _output.position.omega_body_rads;
    attitude_target.omega_dot_body_radss = _output.position.omega_dot_body_radss;

    _attitude_pd.update(state, attitude_target, dt, _output.attitude);
}
