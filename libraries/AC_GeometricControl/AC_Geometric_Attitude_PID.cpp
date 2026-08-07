#include "AC_Geometric_Attitude_PID.h"

#define AC_GEOMETRIC_ATT_KR_X_DEFAULT 4.0f
#define AC_GEOMETRIC_ATT_KR_Y_DEFAULT 4.0f
#define AC_GEOMETRIC_ATT_KR_Z_DEFAULT 2.0f
#define AC_GEOMETRIC_ATT_KO_X_DEFAULT 0.2f
#define AC_GEOMETRIC_ATT_KO_Y_DEFAULT 0.2f
#define AC_GEOMETRIC_ATT_KO_Z_DEFAULT 0.4f
#define AC_GEOMETRIC_ATT_KI_X_DEFAULT 0.0f
#define AC_GEOMETRIC_ATT_KI_Y_DEFAULT 0.0f
#define AC_GEOMETRIC_ATT_KI_Z_DEFAULT 0.1f
#define AC_GEOMETRIC_ATT_IMAX_X_DEFAULT 0.0f
#define AC_GEOMETRIC_ATT_IMAX_Y_DEFAULT 0.0f
#define AC_GEOMETRIC_ATT_IMAX_Z_DEFAULT 1.0f
#define AC_GEOMETRIC_ATT_INT_C_DEFAULT 0.5f
#define AC_GEOMETRIC_ATT_J_X_DEFAULT 0.010f
#define AC_GEOMETRIC_ATT_J_Y_DEFAULT 0.020f
#define AC_GEOMETRIC_ATT_J_Z_DEFAULT 0.020f
#define AC_GEOMETRIC_FILTER_DISABLED 0.0f

const AP_Param::GroupInfo AC_Geometric_Attitude_PID_Params::var_info[] = {
    // Local indices mirror the pre-module flat layout for reviewability. The
    // nested storage identity is new; legacy_var_info performs the upgrade copy.

    // @Param: ATT_KR_X
    // @DisplayName: Geometric attitude roll KR
    // @Description: Lee SO(3) attitude error gain K_R for the body X axis. This affects the geometric observer moment proxy and, when geometric motor output is enabled, the active geometric roll output.
    // @Range: 0 20
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("ATT_KR_X", 7, AC_Geometric_Attitude_PID_Params, _kr_x, AC_GEOMETRIC_ATT_KR_X_DEFAULT),

    // @Param: ATT_KR_Y
    // @DisplayName: Geometric attitude pitch KR
    // @Description: Lee SO(3) attitude error gain K_R for the body Y axis. This affects the geometric observer moment proxy and, when geometric motor output is enabled, the active geometric pitch output.
    // @Range: 0 20
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("ATT_KR_Y", 8, AC_Geometric_Attitude_PID_Params, _kr_y, AC_GEOMETRIC_ATT_KR_Y_DEFAULT),

    // @Param: ATT_KR_Z
    // @DisplayName: Geometric attitude yaw KR
    // @Description: Lee SO(3) attitude error gain K_R for the body Z axis. This affects the geometric observer moment proxy and, when geometric motor output is enabled, the active geometric yaw output.
    // @Range: 0 20
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("ATT_KR_Z", 9, AC_Geometric_Attitude_PID_Params, _kr_z, AC_GEOMETRIC_ATT_KR_Z_DEFAULT),

    // @Param: ATT_KO_X
    // @DisplayName: Geometric angular velocity roll KOmega
    // @Description: Lee SO(3) angular velocity error gain K_Omega for the body X axis. This affects the geometric observer moment proxy and, when geometric motor output is enabled, the active geometric roll output.
    // @Range: 0 5
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("ATT_KO_X", 10, AC_Geometric_Attitude_PID_Params, _ko_x, AC_GEOMETRIC_ATT_KO_X_DEFAULT),

    // @Param: ATT_KO_Y
    // @DisplayName: Geometric angular velocity pitch KOmega
    // @Description: Lee SO(3) angular velocity error gain K_Omega for the body Y axis. This affects the geometric observer moment proxy and, when geometric motor output is enabled, the active geometric pitch output.
    // @Range: 0 5
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("ATT_KO_Y", 11, AC_Geometric_Attitude_PID_Params, _ko_y, AC_GEOMETRIC_ATT_KO_Y_DEFAULT),

    // @Param: ATT_KO_Z
    // @DisplayName: Geometric angular velocity yaw KOmega
    // @Description: Lee SO(3) angular velocity error gain K_Omega for the body Z axis. This affects the geometric observer moment proxy and, when geometric motor output is enabled, the active geometric yaw output.
    // @Range: 0 5
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("ATT_KO_Z", 12, AC_Geometric_Attitude_PID_Params, _ko_z, AC_GEOMETRIC_ATT_KO_Z_DEFAULT),

    // @Param: OMG_FLTE
    // @DisplayName: Geometric angular velocity error filter
    // @Description: Optional first-order low-pass cutoff applied to Lee angular velocity error before the SO(3) attitude PID channel. A value of zero disables this filter and preserves the raw angular-rate error path.
    // @Range: 0 100
    // @Units: Hz
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("OMG_FLTE", 20, AC_Geometric_Attitude_PID_Params, _omega_error_filt_hz, AC_GEOMETRIC_FILTER_DISABLED),

    // @Param: ATT_KI_Z
    // @DisplayName: Geometric yaw integral gain
    // @Description: Yaw geometric integral gain applied to the z-axis of attitude integral state e_I^R. This term is intended to reject slow yaw bias and drift.
    // @Range: 0 5
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("ATT_KI_Z", 23, AC_Geometric_Attitude_PID_Params, _ki_z, AC_GEOMETRIC_ATT_KI_Z_DEFAULT),

    // @Param: ATT_IMAX_Z
    // @DisplayName: Geometric yaw integrator limit
    // @Description: Limit applied to the yaw attitude integral state before the ATT_KI_Z term is applied. A value of zero disables yaw integral accumulation.
    // @Range: 0 20
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("ATT_IMAX_Z", 24, AC_Geometric_Attitude_PID_Params, _imax_z, AC_GEOMETRIC_ATT_IMAX_Z_DEFAULT),

    // @Param: ATT_INT_C
    // @DisplayName: Geometric attitude integral error weight
    // @Description: Attitude-error weight C_R used in e_I^R = integral(e_Omega + C_R*e_R). This has no effect on axes with zero attitude integral gain.
    // @Range: 0 10
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("ATT_INT_C", 25, AC_Geometric_Attitude_PID_Params, _integral_error_p, AC_GEOMETRIC_ATT_INT_C_DEFAULT),

    // @Param: ATT_KI_X
    // @DisplayName: Geometric roll integral gain
    // @Description: Roll geometric integral gain applied to the x-axis of attitude integral state e_I^R. This defaults to zero so roll remains PD unless explicitly enabled.
    // @Range: 0 5
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("ATT_KI_X", 26, AC_Geometric_Attitude_PID_Params, _ki_x, AC_GEOMETRIC_ATT_KI_X_DEFAULT),

    // @Param: ATT_KI_Y
    // @DisplayName: Geometric pitch integral gain
    // @Description: Pitch geometric integral gain applied to the y-axis of attitude integral state e_I^R. This defaults to zero so pitch remains PD unless explicitly enabled.
    // @Range: 0 5
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("ATT_KI_Y", 27, AC_Geometric_Attitude_PID_Params, _ki_y, AC_GEOMETRIC_ATT_KI_Y_DEFAULT),

    // @Param: ATT_IMAX_X
    // @DisplayName: Geometric roll integrator limit
    // @Description: Limit applied to the roll attitude integral state before the ATT_KI_X term is applied. This defaults to zero so roll integral accumulation is disabled unless explicitly enabled.
    // @Range: 0 20
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("ATT_IMAX_X", 28, AC_Geometric_Attitude_PID_Params, _imax_x, AC_GEOMETRIC_ATT_IMAX_X_DEFAULT),

    // @Param: ATT_IMAX_Y
    // @DisplayName: Geometric pitch integrator limit
    // @Description: Limit applied to the pitch attitude integral state before the ATT_KI_Y term is applied. This defaults to zero so pitch integral accumulation is disabled unless explicitly enabled.
    // @Range: 0 20
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("ATT_IMAX_Y", 29, AC_Geometric_Attitude_PID_Params, _imax_y, AC_GEOMETRIC_ATT_IMAX_Y_DEFAULT),

    // @Param: ATT_J_X
    // @DisplayName: Geometric roll inertia
    // @Description: Diagonal body-X inertia term Jx used by the Lee SO(3) attitude moment formula. The default reference model is J = diag(0.010,0.020,0.020) kg*m*m. This is a model parameter for the geometric moment proxy, not an ArduPilot motor normalization scale.
    // @Range: 0.001 1
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("ATT_J_X", 39, AC_Geometric_Attitude_PID_Params, _inertia_x, AC_GEOMETRIC_ATT_J_X_DEFAULT),

    // @Param: ATT_J_Y
    // @DisplayName: Geometric pitch inertia
    // @Description: Diagonal body-Y inertia term Jy used by the Lee SO(3) attitude moment formula. The default reference model is J = diag(0.010,0.020,0.020) kg*m*m. This is a model parameter for the geometric moment proxy, not an ArduPilot motor normalization scale.
    // @Range: 0.001 1
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("ATT_J_Y", 40, AC_Geometric_Attitude_PID_Params, _inertia_y, AC_GEOMETRIC_ATT_J_Y_DEFAULT),

    // @Param: ATT_J_Z
    // @DisplayName: Geometric yaw inertia
    // @Description: Diagonal body-Z inertia term Jz used by the Lee SO(3) attitude moment formula. The default reference model is J = diag(0.010,0.020,0.020) kg*m*m. This is a model parameter for the geometric moment proxy, not an ArduPilot motor normalization scale.
    // @Range: 0.001 1
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("ATT_J_Z", 41, AC_Geometric_Attitude_PID_Params, _inertia_z, AC_GEOMETRIC_ATT_J_Z_DEFAULT),

    AP_GROUPEND
};

const AP_Param::GroupInfo AC_Geometric_Attitude_PID_Params::legacy_var_info[] = {
    AP_GROUPINFO("", 7, AC_Geometric_Attitude_PID_Params, _kr_x, 0.0f),
    AP_GROUPINFO("", 8, AC_Geometric_Attitude_PID_Params, _kr_y, 0.0f),
    AP_GROUPINFO("", 9, AC_Geometric_Attitude_PID_Params, _kr_z, 0.0f),
    AP_GROUPINFO("", 10, AC_Geometric_Attitude_PID_Params, _ko_x, 0.0f),
    AP_GROUPINFO("", 11, AC_Geometric_Attitude_PID_Params, _ko_y, 0.0f),
    AP_GROUPINFO("", 12, AC_Geometric_Attitude_PID_Params, _ko_z, 0.0f),
    AP_GROUPINFO("", 20, AC_Geometric_Attitude_PID_Params, _omega_error_filt_hz, 0.0f),
    AP_GROUPINFO("", 23, AC_Geometric_Attitude_PID_Params, _ki_z, 0.0f),
    AP_GROUPINFO("", 24, AC_Geometric_Attitude_PID_Params, _imax_z, 0.0f),
    AP_GROUPINFO("", 25, AC_Geometric_Attitude_PID_Params, _integral_error_p, 0.0f),
    AP_GROUPINFO("", 26, AC_Geometric_Attitude_PID_Params, _ki_x, 0.0f),
    AP_GROUPINFO("", 27, AC_Geometric_Attitude_PID_Params, _ki_y, 0.0f),
    AP_GROUPINFO("", 28, AC_Geometric_Attitude_PID_Params, _imax_x, 0.0f),
    AP_GROUPINFO("", 29, AC_Geometric_Attitude_PID_Params, _imax_y, 0.0f),
    AP_GROUPINFO("", 39, AC_Geometric_Attitude_PID_Params, _inertia_x, 0.0f),
    AP_GROUPINFO("", 40, AC_Geometric_Attitude_PID_Params, _inertia_y, 0.0f),
    AP_GROUPINFO("", 41, AC_Geometric_Attitude_PID_Params, _inertia_z, 0.0f),
    AP_GROUPEND
};

AC_Geometric_Attitude_PID_Params::AC_Geometric_Attitude_PID_Params()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AC_Geometric_Attitude_PID_Params::convert_legacy_params(uint16_t old_key)
{
    AP_Param::convert_class(old_key, this, legacy_var_info, 0, true);
}

AC_Geometric_Attitude_Gains AC_Geometric_Attitude_PID_Params::gains() const
{
    AC_Geometric_Attitude_Gains gains {};
    gains.attitude_p = Vector3f{_kr_x.get(), _kr_y.get(), _kr_z.get()};
    gains.omega_p = Vector3f{_ko_x.get(), _ko_y.get(), _ko_z.get()};
    gains.attitude_i = Vector3f{_ki_x.get(), _ki_y.get(), _ki_z.get()};
    gains.integral_error_p = Vector3f{_integral_error_p.get(), _integral_error_p.get(), _integral_error_p.get()};
    return gains;
}

AC_Geometric_Attitude_Model AC_Geometric_Attitude_PID_Params::model() const
{
    AC_Geometric_Attitude_Model model {};
    model.inertia = Vector3f{_inertia_x.get(), _inertia_y.get(), _inertia_z.get()};
    return model;
}

AC_Geometric_Attitude_Filter_Hz AC_Geometric_Attitude_PID_Params::filter_hz() const
{
    AC_Geometric_Attitude_Filter_Hz filter_hz {};
    filter_hz.omega_error = _omega_error_filt_hz.get();
    return filter_hz;
}

AC_Geometric_Attitude_Integral_Limits AC_Geometric_Attitude_PID_Params::integral_limits() const
{
    AC_Geometric_Attitude_Integral_Limits limits {};
    limits.integral_error = Vector3f{_imax_x.get(), _imax_y.get(), _imax_z.get()};
    return limits;
}

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

// Let R_ref denote position-derived R_c or a direct target R_d. The attitude
// error is e_R = vee(0.5 * (R_ref^T*R - R^T*R_ref)). Both matrices are
// body-to-NED attitudes; their columns are body basis vectors expressed in
// NED. The vee extraction follows Matrix3 row storage.
struct LeeAttitudeError {
    Vector3f vector;
    float configuration_error = 0.0f;
    float angle_rad = 0.0f;
};

LeeAttitudeError attitude_error_lee(const Quaternion& attitude_body_to_ned,
                                    const Quaternion& attitude_target_to_ned)
{
    Matrix3f attitude;
    Matrix3f attitude_target;

    attitude_body_to_ned.rotation_matrix(attitude);
    attitude_target_to_ned.rotation_matrix(attitude_target);

    const Matrix3f attitude_relative = attitude_target.transposed() * attitude;
    const Matrix3f attitude_error_matrix = attitude_relative - attitude_relative.transposed();
    const float relative_trace = attitude_relative.a.x +
                                 attitude_relative.b.y +
                                 attitude_relative.c.z;

    LeeAttitudeError output {};
    output.vector = Vector3f{attitude_error_matrix.c.y,
                             attitude_error_matrix.a.z,
                             attitude_error_matrix.b.x} * 0.5f;
    output.configuration_error = constrain_float(0.5f * (3.0f - relative_trace),
                                                  0.0f,
                                                  2.0f);
    output.angle_rad = acosf(constrain_float(1.0f - output.configuration_error,
                                             -1.0f,
                                             1.0f));
    return output;
}

Vector3f rotate_target_body_to_current_body(const Quaternion& attitude_body_to_ned,
                                            const Quaternion& attitude_target_to_ned,
                                            const Vector3f& vector_target_body)
{
    Matrix3f attitude;
    Matrix3f attitude_target;

    attitude_body_to_ned.rotation_matrix(attitude);
    attitude_target_to_ned.rotation_matrix(attitude_target);

    // R^T*R_ref expresses reference-body angular terms in the current body frame.
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
        // Disable and clear the axis if either Ki or IMAX is zero.
        return 0.0f;
    }
    if (is_positive(dt)) {
        integrator += input * dt;
    }
    return constrain_float(integrator, -abs_limit, abs_limit);
}

}

void AC_Geometric_Attitude_PID::reset()
{
    _omega_error_filtered_rads.zero();
    _integral_error.zero();
    _filter_reset = true;
}

void AC_Geometric_Attitude_PID::update(const AC_Geometric_State& state,
                                       const AC_Geometric_Target& target,
                                       float dt,
                                       AC_Geometric_Attitude_Output& output)
{
    const LeeAttitudeError attitude_diagnostics =
        attitude_error_lee(state.attitude_body_to_ned, target.attitude_body_to_ned);
    output.attitude_error = attitude_diagnostics.vector;
    output.attitude_configuration_error = attitude_diagnostics.configuration_error;
    output.attitude_error_angle_rad = attitude_diagnostics.angle_rad;

    // e_Omega = Omega - R^T*R_ref*Omega_ref. Desired angular velocity and
    // acceleration are defined in the reference body frame, so R^T*R_ref
    // transports them into the current body frame.
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

    // Geometric PID attitude integral e_I^R = integral(e_Omega + C_R*e_R).
    // Each axis can be independently disabled with zero K_I or zero IMAX.
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

    // SO(3) PID moment equation:
    // M = -K_R*e_R - K_Omega*e_Omega - K_I*e_I^R
    //     + Omega x J*Omega
    //     - J[Omega x (R^T R_ref Omega_ref)
    //         - R^T R_ref dot(Omega_ref)].
    // J is the configured diagonal rigid-body model. M remains a moment proxy
    // because the downstream mapper normalizes it before AP_Motors.
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

    // Legacy diagnostic rate-target proxy retained for geometric logging. It
    // adds the moment proxy M to transported Omega_ref, so it is not a physical
    // angular-rate reference and is not fed into the native rate PID.
    output.rate_target_body_rads = omega_target_current_body_rads + output.moment;
}
