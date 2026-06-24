#include "AC_Geometric_Attitude_PD.h"

namespace {

// vee(0.5 * (Rd^T * R - R^T * Rd)) using Matrix3 row-normal storage.
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

void AC_Geometric_Attitude_PD::update(const AC_Geometric_State& state,
                                      const AC_Geometric_Target& target,
                                      float dt,
                                      AC_Geometric_Attitude_Output& output) const
{
    (void)dt;

    output.attitude_error = attitude_error_lee(state.attitude_body_to_ned, target.attitude_body_to_ned);
    output.omega_error_rads = state.omega_body_rads - target.omega_body_rads;

    // Lee SO(3) PD core without inertia/feedforward terms yet. The result is
    // retained as a moment proxy until the ArduPilot output mapping is designed.
    output.moment.x = -_gains.attitude_p.x * output.attitude_error.x - _gains.omega_p.x * output.omega_error_rads.x;
    output.moment.y = -_gains.attitude_p.y * output.attitude_error.y - _gains.omega_p.y * output.omega_error_rads.y;
    output.moment.z = -_gains.attitude_p.z * output.attitude_error.z - _gains.omega_p.z * output.omega_error_rads.z;

    // Temporary compatibility output for early Guided/attitude experiments.
    output.rate_target_body_rads = target.omega_body_rads + output.moment;
}
