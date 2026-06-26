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

}

void AC_Geometric_Attitude_PD::reset()
{
    _omega_error_filtered_rads.zero();
    _filter_reset = true;
}

void AC_Geometric_Attitude_PD::update(const AC_Geometric_State& state,
                                      const AC_Geometric_Target& target,
                                      float dt,
                                      AC_Geometric_Attitude_Output& output)
{
    output.attitude_error = attitude_error_lee(state.attitude_body_to_ned, target.attitude_body_to_ned);
    const Vector3f omega_error_raw_rads = state.omega_body_rads - target.omega_body_rads;
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

    // Lee SO(3) PD core without inertia/feedforward/Gao compensation terms yet.
    // In Gao notation this is the baseline part of M_d, retained here as a
    // moment proxy until the ArduPilot output mapping is designed.
    output.moment.x = -_gains.attitude_p.x * output.attitude_error.x - _gains.omega_p.x * output.omega_error_rads.x;
    output.moment.y = -_gains.attitude_p.y * output.attitude_error.y - _gains.omega_p.y * output.omega_error_rads.y;
    output.moment.z = -_gains.attitude_p.z * output.attitude_error.z - _gains.omega_p.z * output.omega_error_rads.z;

    // Temporary compatibility output for early Guided/attitude experiments.
    output.rate_target_body_rads = target.omega_body_rads + output.moment;
}
