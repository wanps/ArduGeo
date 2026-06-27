#include "AC_Geometric_Position_PID.h"

namespace {

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
    if (is_positive(dt)) {
        _integral_error_m += integral_input * dt;
    }
    constrain_integral(_integral_error_m, _integral_limits.integral_error_m);
    output.integral_error_m = _integral_error_m;

    // Lee/Gao geometric PID position channel. The integral state follows the
    // geometric PID form e_XI = integral(e_v + c_x * e_x).
    // Lee writes the resultant command as A; Gao uses F_d.
    Vector3f accel_cmd_ned = target.accel_ned_mss;
    accel_cmd_ned.x += -_gains.p.x * output.position_error_m.x - _gains.d.x * output.velocity_error_ms.x - _gains.i.x * output.integral_error_m.x;
    accel_cmd_ned.y += -_gains.p.y * output.position_error_m.y - _gains.d.y * output.velocity_error_ms.y - _gains.i.y * output.integral_error_m.y;
    accel_cmd_ned.z += -_gains.p.z * output.position_error_m.z - _gains.d.z * output.velocity_error_ms.z - _gains.i.z * output.integral_error_m.z;

    output.specific_force_ned_mss = accel_cmd_ned;
    output.specific_force_ned_mss.z -= GRAVITY_MSS;
    // Keep the direction in NED for constructing R_c. Negative Z is upward.
    output.thrust_vector_ned = output.specific_force_ned_mss;

    if (target.build_attitude_from_position) {
        // Full SE(3) coupling path: convert the desired force direction plus
        // yaw reference into R_c, then estimate Omega_c and dot(Omega_c).
        output.attitude_body_to_ned = attitude_from_thrust_vector(output.thrust_vector_ned, target.yaw_rad);
        Vector3f fallback_omega_body_rads = target.omega_body_rads;
        fallback_omega_body_rads.z = target.yaw_rate_rads;
        if (_attitude_target_reset) {
            // On the first sample, avoid differentiating from an uninitialised
            // attitude. Use the caller-provided angular target as a seed.
            _omega_c_filtered_rads = fallback_omega_body_rads;
            _omega_dot_c_filtered_radss.zero();
            output.omega_body_rads = _omega_c_filtered_rads;
            output.omega_dot_body_radss = _omega_dot_c_filtered_radss;
            _attitude_target_reset = false;
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
            const Vector3f omega_dot_c_raw_radss = is_positive(dt) ? (output.omega_body_rads - omega_c_previous_rads) / dt : Vector3f{};
            output.omega_dot_body_radss = apply_optional_lowpass(omega_dot_c_raw_radss,
                                                                 _filter_hz.omega_dot_c,
                                                                 dt,
                                                                 _omega_dot_c_filtered_radss);
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
    }

    Matrix3f attitude;
    state.attitude_body_to_ned.rotation_matrix(attitude);
    // Temporary scalar thrust placeholder for the paper quantity f_d/m.
    // attitude.colz() is the body +Z axis expressed in NED.
    output.thrust = -(output.specific_force_ned_mss * attitude.colz());
}
