#include "AC_Geometric_Attitude_PD.h"

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

// Lee/Gao attitude error e_R = vee(0.5 * (Rd^T * R - R^T * Rd)).
// Both R and Rd are body-to-NED attitudes; their matrix columns are body
// basis vectors expressed in NED. The vee extraction follows Matrix3 row
// storage.
Vector3f attitude_error_lee(const Quaternion& attitude_body_to_ned,
                            const Quaternion& attitude_target_to_ned)
{
    Matrix3f attitude;
    Matrix3f attitude_target;

    attitude_body_to_ned.rotation_matrix(attitude);
    attitude_target_to_ned.rotation_matrix(attitude_target);

    const Matrix3f attitude_error_matrix = attitude_target.transposed() * attitude -
                                           attitude.transposed() * attitude_target;

    return Vector3f{attitude_error_matrix.c.y,
                    attitude_error_matrix.a.z,
                    attitude_error_matrix.b.x} * 0.5f;
}

Vector3f rotate_target_body_to_current_body(const Quaternion& attitude_body_to_ned,
                                            const Quaternion& attitude_target_to_ned,
                                            const Vector3f& vector_target_body)
{
    Matrix3f attitude;
    Matrix3f attitude_target;

    attitude_body_to_ned.rotation_matrix(attitude);
    attitude_target_to_ned.rotation_matrix(attitude_target);

    // Lee uses R^T * R_c to express desired angular terms in the current body frame.
    return attitude.mul_transpose(attitude_target * vector_target_body);
}

Vector3f safe_inertia(const Vector3f& inertia)
{
    return Vector3f{MAX(fabsf(inertia.x), 1.0e-6f),
                    MAX(fabsf(inertia.y), 1.0e-6f),
                    MAX(fabsf(inertia.z), 1.0e-6f)};
}

Vector3f scale_by_axis(const Vector3f& vector, const Vector3f& scale)
{
    return Vector3f{vector.x * scale.x,
                    vector.y * scale.y,
                    vector.z * scale.z};
}

float update_integral_axis(float integrator,
                           float input,
                           float gain,
                           float limit,
                           float dt)
{
    const float abs_limit = MAX(fabsf(limit), 0.0f);
    if (!is_positive(gain) || !is_positive(abs_limit)) {
        return 0.0f;
    }
    if (is_positive(dt)) {
        integrator += input * dt;
    }
    return constrain_float(integrator, -abs_limit, abs_limit);
}

}

void AC_Geometric_Attitude_PD::reset()
{
    _omega_error_filtered_rads.zero();
    _integral_error.zero();
    _filter_reset = true;
}

void AC_Geometric_Attitude_PD::update(const AC_Geometric_State& state,
                                      const AC_Geometric_Target& target,
                                      float dt,
                                      AC_Geometric_Attitude_Output& output)
{
    output.attitude_error = attitude_error_lee(state.attitude_body_to_ned, target.attitude_body_to_ned);
    const Vector3f omega_target_current_body_rads =
        rotate_target_body_to_current_body(state.attitude_body_to_ned,
                                           target.attitude_body_to_ned,
                                           target.omega_body_rads);
    const Vector3f omega_dot_target_current_body_radss =
        rotate_target_body_to_current_body(state.attitude_body_to_ned,
                                           target.attitude_body_to_ned,
                                           target.omega_dot_body_radss);
    const Vector3f omega_error_raw_rads = state.omega_body_rads - omega_target_current_body_rads;
    if (_filter_reset) {
        _omega_error_filtered_rads = omega_error_raw_rads;
        _filter_reset = false;
    } else {
        _omega_error_filtered_rads = apply_optional_lowpass(omega_error_raw_rads,
                                                            _filter_hz.omega_error,
                                                            dt,
                                                            _omega_error_filtered_rads);
    }
    output.omega_error_rads = _omega_error_filtered_rads;

    const Vector3f integral_input {
        output.omega_error_rads.x + _gains.integral_error_p.x * output.attitude_error.x,
        output.omega_error_rads.y + _gains.integral_error_p.y * output.attitude_error.y,
        output.omega_error_rads.z + _gains.integral_error_p.z * output.attitude_error.z
    };
    _integral_error.x = update_integral_axis(_integral_error.x,
                                             integral_input.x,
                                             _gains.attitude_i.x,
                                             _integral_limits.integral_error.x,
                                             dt);
    _integral_error.y = update_integral_axis(_integral_error.y,
                                             integral_input.y,
                                             _gains.attitude_i.y,
                                             _integral_limits.integral_error.y,
                                             dt);
    _integral_error.z = update_integral_axis(_integral_error.z,
                                             integral_input.z,
                                             _gains.attitude_i.z,
                                             _integral_limits.integral_error.z,
                                             dt);
    output.integral_error = _integral_error;

    // Lee SO(3) attitude control structure:
    // M = -k_R e_R - k_Omega e_Omega
    //     + Omega x J*Omega
    //     - J*(Omega x (R^T R_c Omega_c) - R^T R_c dot(Omega_c)).
    // J is currently represented as a diagonal model because the mapper consumes
    // normalized moment proxies, not physical motor torques yet.
    const Vector3f inertia = safe_inertia(_model.inertia);
    const Vector3f gyro = state.omega_body_rads % scale_by_axis(state.omega_body_rads, inertia);
    const Vector3f desired_transport = state.omega_body_rads % omega_target_current_body_rads;
    const Vector3f desired_dynamics = scale_by_axis(desired_transport - omega_dot_target_current_body_radss,
                                                    inertia);
    const Vector3f feedforward = gyro - desired_dynamics;
    output.moment.x = -_gains.attitude_p.x * output.attitude_error.x -
                      _gains.omega_p.x * output.omega_error_rads.x -
                      _gains.attitude_i.x * output.integral_error.x +
                      feedforward.x;
    output.moment.y = -_gains.attitude_p.y * output.attitude_error.y -
                      _gains.omega_p.y * output.omega_error_rads.y -
                      _gains.attitude_i.y * output.integral_error.y +
                      feedforward.y;
    output.moment.z = -_gains.attitude_p.z * output.attitude_error.z -
                      _gains.omega_p.z * output.omega_error_rads.z -
                      _gains.attitude_i.z * output.integral_error.z +
                      feedforward.z;

    // Temporary compatibility output for early Guided/attitude experiments.
    output.rate_target_body_rads = omega_target_current_body_rads + output.moment;
}
