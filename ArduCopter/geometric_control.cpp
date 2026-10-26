#include "Copter.h"

namespace {

constexpr uint32_t GEOMETRIC_REFERENCE_MAX_AGE_MS = 100;

}

bool Copter::update_geometric_controller(const AC_TrajectoryReference& trajectory_reference,
                                         const AC_AttitudeReference* attitude_reference,
                                         const AC_GeometricReferencePolicy& policy,
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

    const uint32_t now_ms = AP_HAL::millis();
    AC_Geometric_Target target {};
    if (!trajectory_reference.meta.is_fresh(now_ms, GEOMETRIC_REFERENCE_MAX_AGE_MS) ||
        (attitude_reference != nullptr &&
         !attitude_reference->meta.is_fresh(now_ms, GEOMETRIC_REFERENCE_MAX_AGE_MS)) ||
        !AC_GeometricControl::references_to_target(trajectory_reference,
                                                   attitude_reference,
                                                   policy,
                                                   target)) {
        geometric_control.set_enabled(false);
        return false;
    }

    const Vector3p& pos_estimate_ned_m = pos_control->get_pos_estimate_NED_m();
    state.position_ned_m = Vector3f{float(pos_estimate_ned_m.x),
                                   float(pos_estimate_ned_m.y),
                                   float(pos_estimate_ned_m.z)};
    state.velocity_ned_ms = pos_control->get_vel_estimate_NED_ms();
    ahrs.get_quat_body_to_ned(state.attitude_body_to_ned);
    state.omega_body_rads = ahrs.get_gyro_latest();
    state.land_complete_maybe = ap.land_complete_maybe;
    state.thrust_limited = motors->limit.throttle_upper || motors->limit.throttle_lower;
    state.applied_throttle_norm = motors->get_throttle_out();

    geometric_control.set_hover_throttle_reference(motors->get_throttle_hover());
    geometric_control.update(state, target, G_Dt);
    geometric_controller_update_count++;

    // Once per episode: L1 has been injecting its clamped value for a second,
    // so the vehicle is outside the range GEO_L1_ZMAX was calibrated for.
    // Informational; nothing is changed by it.
    const float l1_clamped_s = geometric_control.l1_augment().clamped_for_s();
    if (l1_clamped_s >= 1.0f && !geometric_l1_clamp_notified) {
        gcs().send_text(MAV_SEVERITY_INFO, "L1: vertical estimate at GEO_L1_ZMAX for 1 s");
        geometric_l1_clamp_notified = true;
    } else if (!is_positive(l1_clamped_s)) {
        geometric_l1_clamp_notified = false;
    }
    return true;
}
