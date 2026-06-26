#include "AC_Geometric_Position_PID.h"

namespace {

Vector3f apply_optional_lowpass(const Vector3f& input,
                                float cutoff_hz,
                                float dt,
                                Vector3f& filtered)
{
    if (!is_positive(cutoff_hz) || !is_positive(dt)) {
        filtered = input;
        return input;
    }

    filtered += (input - filtered) * calc_lowpass_alpha_dt(dt, cutoff_hz);
    return filtered;
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

}

void AC_Geometric_Position_PID::reset()
{
    _position_error_integral_m.zero();
    _position_error_filtered_m.zero();
    _velocity_error_filtered_ms.zero();
    _filter_reset = true;
}

void AC_Geometric_Position_PID::update(const AC_Geometric_State& state,
                                       const AC_Geometric_Target& target,
                                       float dt,
                                       AC_Geometric_Position_Output& output)
{
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

    if (is_positive(dt)) {
        _position_error_integral_m += output.position_error_m * dt;
    }

    // Placeholder translational PID. This will later be replaced by the full
    // SE(3) construction of resultant force direction, collective thrust and R_c.
    // Lee writes the resultant command as A; Gao uses F_d.
    Vector3f accel_cmd_ned = target.accel_ned_mss;
    accel_cmd_ned.x += -_gains.p.x * output.position_error_m.x - _gains.d.x * output.velocity_error_ms.x - _gains.i.x * _position_error_integral_m.x;
    accel_cmd_ned.y += -_gains.p.y * output.position_error_m.y - _gains.d.y * output.velocity_error_ms.y - _gains.i.y * _position_error_integral_m.y;
    accel_cmd_ned.z += -_gains.p.z * output.position_error_m.z - _gains.d.z * output.velocity_error_ms.z - _gains.i.z * _position_error_integral_m.z;

    output.specific_force_ned_mss = accel_cmd_ned;
    output.specific_force_ned_mss.z -= GRAVITY_MSS;
    // Keep the direction in NED for constructing R_c. Negative Z is upward.
    output.thrust_vector_ned = output.specific_force_ned_mss;

    if (target.build_attitude_from_position) {
        output.attitude_body_to_ned = attitude_from_thrust_vector(output.thrust_vector_ned, target.yaw_rad);
        output.omega_body_rads = target.omega_body_rads;
        output.omega_body_rads.z = target.yaw_rate_rads;
        output.omega_dot_body_radss.zero();
    } else {
        // Direct SO(3) observer mode: pass through the supplied attitude target
        // so Guided angle tests observe ArduPilot's shaped attitude target.
        output.attitude_body_to_ned = target.attitude_body_to_ned;
        output.omega_body_rads = target.omega_body_rads;
        output.omega_dot_body_radss = target.omega_dot_body_radss;
    }

    Matrix3f attitude;
    state.attitude_body_to_ned.rotation_matrix(attitude);
    // Temporary scalar thrust placeholder for the paper quantity f_d/m.
    // attitude.colz() is the body +Z axis expressed in NED.
    output.thrust = -(output.specific_force_ned_mss * attitude.colz());
}
