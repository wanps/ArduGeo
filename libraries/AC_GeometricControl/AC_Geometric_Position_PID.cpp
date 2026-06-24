#include "AC_Geometric_Position_PID.h"

void AC_Geometric_Position_PID::reset()
{
    _position_error_integral_m.zero();
}

void AC_Geometric_Position_PID::update(const AC_Geometric_State& state,
                                       const AC_Geometric_Target& target,
                                       float dt,
                                       AC_Geometric_Position_Output& output)
{
    output.position_error_m = state.position_ned_m - target.position_ned_m;
    output.velocity_error_ms = state.velocity_ned_ms - target.velocity_ned_ms;

    if (is_positive(dt)) {
        _position_error_integral_m += output.position_error_m * dt;
    }

    // Placeholder translational PID. This will later be replaced by the full
    // Lee SE(3) construction of thrust direction, collective thrust and R_d.
    Vector3f accel_cmd_ned = target.accel_ned_mss;
    accel_cmd_ned.x += -_gains.p.x * output.position_error_m.x - _gains.d.x * output.velocity_error_ms.x - _gains.i.x * _position_error_integral_m.x;
    accel_cmd_ned.y += -_gains.p.y * output.position_error_m.y - _gains.d.y * output.velocity_error_ms.y - _gains.i.y * _position_error_integral_m.y;
    accel_cmd_ned.z += -_gains.p.z * output.position_error_m.z - _gains.d.z * output.velocity_error_ms.z - _gains.i.z * _position_error_integral_m.z;

    // Until the SE(3) attitude construction is implemented, pass through the
    // supplied attitude target so the attitude channel can be tested independently.
    output.attitude_body_to_ned = target.attitude_body_to_ned;
    output.omega_body_rads = target.omega_body_rads;
    output.omega_dot_body_radss = target.omega_dot_body_radss;
    output.thrust = -accel_cmd_ned.z;
}
