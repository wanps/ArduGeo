#include "AC_Geometric_Position_PID.h"

namespace {

// Below five percent of the hover specific force the mapped collective is
// effectively near zero (five percent of the configured hover throttle), and
// using its direction to construct R_c is ill-conditioned: an arbitrarily
// small lateral residual can request a near-90-degree tilt.  Treat this as the
// zero-collective attitude domain for a conventional upright multirotor.
constexpr float UPRIGHT_MIN_UPWARD_SPECIFIC_FORCE_MSS = GRAVITY_MSS * 0.05f;
constexpr float UPRIGHT_RELEASE_UPWARD_SPECIFIC_FORCE_MSS = GRAVITY_MSS * 0.07f;

Vector3f apply_optional_lowpass(const Vector3f& input,
                                float cutoff_hz,
                                float dt,
                                Vector3f& filtered)
{
    // A zero cutoff is the documented bypass path for all optional filters.
    if (!is_positive(cutoff_hz) || !is_positive(dt)) {
        filtered = input;
        return input;
    }

    filtered += (input - filtered) * calc_lowpass_alpha_dt(dt, cutoff_hz);
    return filtered;
}

void constrain_integral(Vector3f& integral, const Vector3f& limits)
{
    const Vector3f safe_limits {
        MAX(fabsf(limits.x), 0.0f),
        MAX(fabsf(limits.y), 0.0f),
        MAX(fabsf(limits.z), 0.0f)
    };

    integral.x = constrain_float(integral.x, -safe_limits.x, safe_limits.x);
    integral.y = constrain_float(integral.y, -safe_limits.y, safe_limits.y);
    integral.z = constrain_float(integral.z, -safe_limits.z, safe_limits.z);
}

// Build a body-to-NED commanded attitude R_c from an ArduPilot-style thrust
// vector expressed in NED and a yaw angle about NED +Z. In NED, hover thrust
// points upward, so the level reference thrust direction is {0, 0, -1}.
Quaternion attitude_from_thrust_vector(Vector3f thrust_vector, float yaw_rad)
{
    const Vector3f thrust_vector_up{0.0f, 0.0f, -1.0f};

    if (is_zero(thrust_vector.length_squared())) {
        thrust_vector = thrust_vector_up;
    } else {
        thrust_vector.normalize();
    }

    Vector3f thrust_vec_cross = thrust_vector_up % thrust_vector;
    const float thrust_vector_angle = acosf(constrain_float(thrust_vector_up * thrust_vector, -1.0f, 1.0f));
    const float thrust_vector_length = thrust_vec_cross.length();
    if (is_zero(thrust_vector_length) || is_zero(thrust_vector_angle)) {
        thrust_vec_cross = thrust_vector_up;
    } else {
        thrust_vec_cross /= thrust_vector_length;
    }

    Quaternion thrust_vec_quat;
    thrust_vec_quat.from_axis_angle(thrust_vec_cross, thrust_vector_angle);

    Quaternion yaw_quat;
    yaw_quat.from_axis_angle(Vector3f{0.0f, 0.0f, 1.0f}, yaw_rad);
    return thrust_vec_quat * yaw_quat;
}

// A conventional multirotor can only produce non-negative collective along
// body -Z.  The position controller therefore must not turn a force with a
// non-upward NED component into an inverted attitude target.  A force on or
// below the NED horizontal plane is outside the upright flight domain.  The
// near-zero-collective region above that exact boundary is regularised too,
// because its force direction is not a useful attitude command.  Use the
// zero-force member of the feasible set and let attitude_from_thrust_vector()
// select the finite, level attitude at the requested yaw.
//
// Keep this projection separate from specific_force_ned_mss so the
// unconstrained geometric request remains available for logging. The applied
// scalar collective is independently forced to zero only at its physical lower
// bound; a small force which is still upward remains available to the mapper.
// Estimate desired body-frame angular velocity from the change in commanded
// body-to-NED attitude. For body-to-inertial R, R_prev^T * R_curr is the small
// body-frame rotation over dt.
Vector3f attitude_delta_to_body_rate(const Quaternion& last_attitude_body_to_ned,
                                     const Quaternion& attitude_body_to_ned,
                                     float dt)
{
    if (!is_positive(dt)) {
        return Vector3f{};
    }

    Matrix3f last_attitude;
    Matrix3f attitude;
    last_attitude_body_to_ned.rotation_matrix(last_attitude);
    attitude_body_to_ned.rotation_matrix(attitude);

    const Matrix3f attitude_delta = last_attitude.transposed() * attitude;
    const Vector3f delta_vee {
        attitude_delta.c.y - attitude_delta.b.z,
        attitude_delta.a.z - attitude_delta.c.x,
        attitude_delta.b.x - attitude_delta.a.y
    };
    return delta_vee * (0.5f / dt);
}

}

void AC_Geometric_Position_PID::reset()
{
    _integral_error_m.zero();
    _position_error_filtered_m.zero();
    _velocity_error_filtered_ms.zero();
    _omega_c_filtered_rads.zero();
    _omega_dot_c_filtered_radss.zero();
    _filter_reset = true;
    _attitude_target_reset = true;
    _regularization_derivative_reset = false;
    _upright_thrust_regularized = false;
}

void AC_Geometric_Position_PID::update(const AC_Geometric_State& state,
                                       const AC_Geometric_Target& target,
                                       float dt,
                                       AC_Geometric_Position_Output& output)
{
    // Lee/Gao use e_x = x - x_d and e_v = v - v_d in inertial coordinates.
    // ArduPilot's inertial frame here is NED, with positive Z pointing down.
    const Vector3f position_error_raw_m = state.position_ned_m - target.position_ned_m;
    const Vector3f velocity_error_raw_ms = state.velocity_ned_ms - target.velocity_ned_ms;

    if (_filter_reset) {
        _position_error_filtered_m = position_error_raw_m;
        _velocity_error_filtered_ms = velocity_error_raw_ms;
        _filter_reset = false;
    } else {
        _position_error_filtered_m = apply_optional_lowpass(position_error_raw_m,
                                                            _filter_hz.position_error,
                                                            dt,
                                                            _position_error_filtered_m);
        _velocity_error_filtered_ms = apply_optional_lowpass(velocity_error_raw_ms,
                                                             _filter_hz.velocity_error,
                                                             dt,
                                                             _velocity_error_filtered_ms);
    }

    output.position_error_m = _position_error_filtered_m;
    output.velocity_error_ms = _velocity_error_filtered_ms;

    // Geometric PID integral. The c_x weighting lets the integral term reject
    // constant disturbances without requiring raw position error integration.
    const Vector3f integral_input {
        output.velocity_error_ms.x + _gains.integral_error_p.x * output.position_error_m.x,
        output.velocity_error_ms.y + _gains.integral_error_p.y * output.position_error_m.y,
        output.velocity_error_ms.z + _gains.integral_error_p.z * output.position_error_m.z
    };
    // Apply a possibly reduced runtime limit before remembering the state used
    // by conditional integration below.
    constrain_integral(_integral_error_m, _integral_limits.integral_error_m);
    const Vector3f integral_error_before_update_m = _integral_error_m;
    if (is_positive(dt)) {
        _integral_error_m += integral_input * dt;
    }
    constrain_integral(_integral_error_m, _integral_limits.integral_error_m);

    // Lee/Gao geometric PID position channel. The integral state follows the
    // geometric PID form e_XI = integral(e_v + c_x * e_x).
    // Lee writes the resultant command as A; Gao uses F_d.
    Vector3f accel_cmd_ned = target.accel_ned_mss;
    accel_cmd_ned.x += -_gains.p.x * output.position_error_m.x - _gains.d.x * output.velocity_error_ms.x - _gains.i.x * _integral_error_m.x;
    accel_cmd_ned.y += -_gains.p.y * output.position_error_m.y - _gains.d.y * output.velocity_error_ms.y - _gains.i.y * _integral_error_m.y;
    accel_cmd_ned.z += -_gains.p.z * output.position_error_m.z - _gains.d.z * output.velocity_error_ms.z - _gains.i.z * _integral_error_m.z;

    // Conditional vertical anti-windup for the conventional multirotor
    // boundary. If this sample's integral increment would make the raw force
    // point downward, analytically project I_z onto the zero-collective force
    // boundary. This lets the mapper reach exactly zero instead of holding a
    // small positive collective at the attitude-regularisation threshold.
    // Integral motion which unwinds the saturation remains active. This is
    // deliberately local to build_attitude_from_position: observer targets do
    // not use this force to construct an attitude.
    const float specific_force_z_mss = accel_cmd_ned.z - GRAVITY_MSS;
    const float integral_force_delta_z_mss = -_gains.i.z *
                                               (_integral_error_m.z - integral_error_before_update_m.z);
    bool vertical_collective_lower_limited = false;
    if (target.build_attitude_from_position &&
        specific_force_z_mss >= 0.0f &&
        integral_force_delta_z_mss > 0.0f &&
        !is_zero(_gains.i.z)) {
        const float specific_force_without_integral_z_mss = specific_force_z_mss +
                                                             _gains.i.z * _integral_error_m.z;
        _integral_error_m.z = specific_force_without_integral_z_mss / _gains.i.z;
        // If the exact zero-force boundary is outside Imax, retain the closest
        // reachable integral. The raw force and mapper clamp still provide the
        // safe fallback for that case.
        constrain_integral(_integral_error_m, _integral_limits.integral_error_m);
        accel_cmd_ned.z = target.accel_ned_mss.z
                          - _gains.p.z * output.position_error_m.z
                          - _gains.d.z * output.velocity_error_ms.z
                          - _gains.i.z * _integral_error_m.z;
        // Remember the analytical lower-bound projection independently of
        // floating-point roundoff in the recomputed raw force.
        vertical_collective_lower_limited = fabsf(accel_cmd_ned.z - GRAVITY_MSS) <= 1.0e-4f;
    }
    output.integral_error_m = _integral_error_m;

    output.specific_force_ned_mss = accel_cmd_ned;
    output.specific_force_ned_mss.z -= GRAVITY_MSS;
    // Keep a feasible, well-conditioned direction in NED for constructing R_c.
    // Enter level-yaw regularisation at 5% of hover specific force and release
    // only above 7%; the small hysteresis prevents attitude/rate chatter.
    const bool upright_thrust_regularized_before_update = _upright_thrust_regularized;
    if (target.build_attitude_from_position) {
        if (_upright_thrust_regularized) {
            if (output.specific_force_ned_mss.z < -UPRIGHT_RELEASE_UPWARD_SPECIFIC_FORCE_MSS) {
                _upright_thrust_regularized = false;
            }
        } else if (output.specific_force_ned_mss.z >= -UPRIGHT_MIN_UPWARD_SPECIFIC_FORCE_MSS) {
            _upright_thrust_regularized = true;
        }
    } else {
        _upright_thrust_regularized = false;
    }
    const bool upright_thrust_regularization_transition =
        _upright_thrust_regularized != upright_thrust_regularized_before_update;
    output.thrust_vector_ned = _upright_thrust_regularized ? Vector3f{} : output.specific_force_ned_mss;

    if (target.build_attitude_from_position) {
        // Full SE(3) coupling path: convert the desired force direction plus
        // yaw reference into R_c, then estimate Omega_c and dot(Omega_c).
        output.attitude_body_to_ned = attitude_from_thrust_vector(output.thrust_vector_ned, target.yaw_rad);
        Vector3f fallback_omega_body_rads = target.omega_body_rads;
        fallback_omega_body_rads.z = target.yaw_rate_rads;
        if (_attitude_target_reset || upright_thrust_regularization_transition) {
            // On the first sample, avoid differentiating from an uninitialised
            // attitude. The same reset is required when near-zero collective
            // enters or exits level-yaw regularisation; finite-differencing that
            // deliberate attitude switch would create a spurious rate impulse.
            // Use the caller-provided angular target as a seed.
            _omega_c_filtered_rads = fallback_omega_body_rads;
            _omega_dot_c_filtered_radss.zero();
            output.omega_body_rads = _omega_c_filtered_rads;
            output.omega_dot_body_radss = _omega_dot_c_filtered_radss;
            _attitude_target_reset = false;
            _regularization_derivative_reset = upright_thrust_regularization_transition;
        } else {
            // The paper defines Omega_c analytically. This implementation
            // approximates it from the discrete R_c sequence and filters the
            // result before feeding the SO(3) attitude channel.
            const Vector3f omega_c_raw_rads = attitude_delta_to_body_rate(_last_attitude_target_body_to_ned,
                                                                          output.attitude_body_to_ned,
                                                                          dt);
            const Vector3f omega_c_previous_rads = _omega_c_filtered_rads;
            output.omega_body_rads = apply_optional_lowpass(omega_c_raw_rads,
                                                            _filter_hz.omega_c,
                                                            dt,
                                                            _omega_c_filtered_rads);
            if (_regularization_derivative_reset) {
                // The first finite-difference sample after the transition must
                // not differentiate the fallback seed used on the switch frame.
                _omega_dot_c_filtered_radss.zero();
                output.omega_dot_body_radss.zero();
                _regularization_derivative_reset = false;
            } else {
                const Vector3f omega_dot_c_raw_radss = is_positive(dt) ? (output.omega_body_rads - omega_c_previous_rads) / dt : Vector3f{};
                output.omega_dot_body_radss = apply_optional_lowpass(omega_dot_c_raw_radss,
                                                                     _filter_hz.omega_dot_c,
                                                                     dt,
                                                                     _omega_dot_c_filtered_radss);
            }
        }
        _last_attitude_target_body_to_ned = output.attitude_body_to_ned;
    } else {
        // Direct SO(3) observer mode: pass through the supplied attitude target
        // so Guided angle tests observe ArduPilot's shaped attitude target.
        output.attitude_body_to_ned = target.attitude_body_to_ned;
        output.omega_body_rads = target.omega_body_rads;
        output.omega_dot_body_radss = target.omega_dot_body_radss;
        _omega_c_filtered_rads.zero();
        _omega_dot_c_filtered_radss.zero();
        _attitude_target_reset = true;
        _regularization_derivative_reset = false;
    }

    Matrix3f attitude;
    state.attitude_body_to_ned.rotation_matrix(attitude);
    // Temporary scalar thrust placeholder for the paper quantity f_d/m.
    // attitude.colz() is the body +Z axis expressed in NED.
    // At the unidirectional lower bound, no horizontal residual may leak into
    // collective through a tilted current b3 axis. Keep the raw specific force
    // above for logging, but explicitly apply zero thrust. In the near-zero
    // level-yaw domain, project collective through the same regularised force:
    // retain the small upward Z component but discard the XY component which
    // was deliberately removed from R_c. Normal upward flight keeps the full
    // geometric force projection.
    const bool non_upward_force = output.specific_force_ned_mss.z >= 0.0f;
    if (vertical_collective_lower_limited || non_upward_force) {
        output.thrust = 0.0f;
    } else if (_upright_thrust_regularized) {
        output.thrust = -output.specific_force_ned_mss.z * attitude.colz().z;
    } else {
        output.thrust = -(output.specific_force_ned_mss * attitude.colz());
    }
}
