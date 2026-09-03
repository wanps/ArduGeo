#include "Copter.h"

bool Copter::update_geometric_controller(const AC_Geometric_Target& target,
                                         bool enabled,
                                         AC_Geometric_State& state)
{
    // Sample geometric state X from EKF/AHRS, combine it with the mode-owned
    // reference X_d, and evaluate the shared geometric cascade. Motor
    // ownership is decided later in the rate-control phase.
    geometric_control.set_enabled(enabled);
    if (!enabled) {
        return false;
    }

    const Vector3p& pos_estimate_ned_m = pos_control->get_pos_estimate_NED_m();
    state.position_ned_m = Vector3f{float(pos_estimate_ned_m.x),
                                   float(pos_estimate_ned_m.y),
                                   float(pos_estimate_ned_m.z)};
    state.velocity_ned_ms = pos_control->get_vel_estimate_NED_ms();
    ahrs.get_quat_body_to_ned(state.attitude_body_to_ned);
    state.omega_body_rads = ahrs.get_gyro_latest();

    geometric_control.set_hover_throttle_reference(motors->get_throttle_hover());
    geometric_control.update(state, target, G_Dt);
    return true;
}
