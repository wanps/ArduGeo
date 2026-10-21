#include "AC_Geometric_L1_Observer.h"

#include <AP_Math/AP_Math.h>

const AP_Param::GroupInfo AC_Geometric_L1_Observer_Params::var_info[] = {

    // @Param: L1_OBS_EN
    // @DisplayName: L1 translational observer enable
    // @Description: Runs the L1 adaptive estimator for the translational channel in observer-only mode. The estimate is computed and logged; it never contributes to any force, moment or motor command. Enabling it cannot change how the aircraft flies.
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO("L1_OBS_EN", 1, AC_Geometric_L1_Observer_Params, _enable, 0),

    // @Param: L1_OBS_AS
    // @DisplayName: L1 observer predictor gain
    // @Description: Convergence rate a_s of the state predictor, which runs with A_s = -a_s*I. Larger values pull the predicted velocity onto the measurement faster, moving unmodelled acceleration out of the prediction error and into the estimate sooner, at the cost of admitting more measurement noise.
    // @Range: 0.1 50
    // @Units: 1/s
    // @User: Advanced
    AP_GROUPINFO("L1_OBS_AS", 2, AC_Geometric_L1_Observer_Params, _predictor_gain, 5.0f),

    // @Param: L1_OBS_FLT
    // @DisplayName: L1 observer estimate filter
    // @Description: Cut-off frequency of the low-pass applied to the estimate. Zero bypasses the filter. This is the bandwidth limit that an L1 control law would impose before using the estimate, applied here so the logged signal matches what such a law would see.
    // @Range: 0 20
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("L1_OBS_FLT", 3, AC_Geometric_L1_Observer_Params, _filter_hz, 2.0f),

    // @Param: L1_OBS_SMAX
    // @DisplayName: L1 observer raw estimate backstop
    // @Description: Clamp on the raw, unfiltered estimate. It is a guard against numerical blow-up and should never fire in flight: the raw law amplifies velocity noise by about 1/dt, so its magnitude routinely reaches tens of m/s/s while the filtered estimate stays under 5. The overrun criterion is L1_OBS_SFMAX, not this. Zero disables the clamp.
    // @Range: 0 100
    // @Units: m/s/s
    // @User: Advanced
    AP_GROUPINFO("L1_OBS_SMAX", 4, AC_Geometric_L1_Observer_Params, _sigma_max, 60.0f),

    // @Param: L1_OBS_SFMAX
    // @DisplayName: L1 observer overrun limit
    // @Description: Magnitude of the filtered estimate above which it is flagged as out of range, and below which L1 may be admitted. 5.0 is 1.03 times the largest filtered value seen in a nominal-load flight; it is a starting point and must be recalibrated before any offset-load flight. Zero disables the flag and makes admission impossible.
    // @Range: 0 50
    // @Units: m/s/s
    // @User: Advanced
    AP_GROUPINFO("L1_OBS_SFMAX", 5, AC_Geometric_L1_Observer_Params, _sigma_filtered_max, 5.0f),

    // @Param: L1_OBS_ADMV
    // @DisplayName: L1 admission: continuous validity
    // @Description: How long the estimate must have been valid without a break before L1 may be admitted. Excludes the first moments after power-up or a re-seed. An order-of-magnitude choice, not a measured one.
    // @Range: 0 30
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("L1_OBS_ADMV", 6, AC_Geometric_L1_Observer_Params, _admit_valid_s, 2.0f),

    // @Param: L1_OBS_ADMQ
    // @DisplayName: L1 admission: time since last re-seed
    // @Description: How long since the predictor was last re-seeded before L1 may be admitted. Excludes an estimator that keeps resetting. An order-of-magnitude choice, not a measured one.
    // @Range: 0 60
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("L1_OBS_ADMQ", 7, AC_Geometric_L1_Observer_Params, _admit_quiet_s, 5.0f),

    AP_GROUPEND
};

AC_Geometric_L1_Observer_Params::AC_Geometric_L1_Observer_Params()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool AC_Geometric_L1_Observer_Params::enabled() const
{
    return _enable > 0;
}

AC_Geometric_L1_Observer_Config AC_Geometric_L1_Observer_Params::config() const
{
    AC_Geometric_L1_Observer_Config config {};
    config.predictor_gain = _predictor_gain;
    config.filter_hz = _filter_hz;
    config.sigma_max = _sigma_max;
    config.sigma_filtered_max = _sigma_filtered_max;
    config.admit_valid_s = _admit_valid_s;
    config.admit_quiet_s = _admit_quiet_s;
    return config;
}

namespace {

// Matches the bypass convention used by the other optional filters in this
// library: a non-positive cut-off or timestep passes the input through.
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

bool is_finite(const Vector3f& v)
{
    return !v.is_nan() && !v.is_inf();
}

} // namespace

AC_Geometric_L1_Observer::AC_Geometric_L1_Observer() :
    _valid_time_s(0.0f),
    _quiet_time_s(0.0f),
    _resets_seen(0)
{
    reset();
}

void AC_Geometric_L1_Observer::set_config(const AC_Geometric_L1_Observer_Config& config)
{
    _config = config;
}

void AC_Geometric_L1_Observer::reset()
{
    _velocity_predicted_ned_ms.zero();
    _sigma_hat_ned_mss.zero();
    _sigma_filtered_ned_mss.zero();
    _initialised = false;
    // A re-seed breaks continuity, so the validity timer restarts. The quiet
    // timer does not: it answers "has the estimator been resetting itself",
    // which is what the reset count records, and a caller re-seeding on a
    // mode change or a fallback is not that. Admission still waits out the
    // validity timer after any re-seed.
    _valid_time_s = 0.0f;

    const uint32_t resets = _output.resets;
    _output = AC_Geometric_L1_Observer_Output {};
    _output.resets = resets;
}

void AC_Geometric_L1_Observer::update(const AC_Geometric_State& state,
                                      const AC_Geometric_Position_Output& position,
                                      float dt)
{
    _output.valid = false;
    _output.saturated = false;
    _output.filtered_over_limit = false;
    _output.admissible = false;

    const float a_s = _config.predictor_gain;
    if (!is_positive(a_s) || !is_positive(dt) || !isfinite(dt)) {
        reset();
        _output.resets++;
        return;
    }
    if (!is_finite(state.velocity_ned_ms) || !isfinite(position.thrust)) {
        reset();
        _output.resets++;
        return;
    }

    // Nominal translational dynamics.  The applied collective is used rather
    // than the commanded resultant, so that attitude tracking error stays in
    // the plant where it belongs instead of being absorbed into the estimate.
    //
    // position.thrust is f = -A^T R e_D, which is +g at hover, so the applied
    // specific force in NED is -f * (R e_D) and gravity is +g on the down axis.
    Matrix3f rotation;
    state.attitude_body_to_ned.rotation_matrix(rotation);
    const Vector3f body_down_in_ned = rotation * Vector3f{0.0f, 0.0f, 1.0f};
    const Vector3f nominal_accel_ned_mss =
        body_down_in_ned * (-position.thrust) + Vector3f{0.0f, 0.0f, GRAVITY_MSS};
    if (!is_finite(nominal_accel_ned_mss)) {
        reset();
        _output.resets++;
        return;
    }
    _output.nominal_accel_ned_mss = nominal_accel_ned_mss;

    // Seeding the predictor from the measurement makes the first prediction
    // error exactly zero, so a resumed observer cannot report a step that the
    // aircraft never experienced.
    if (!_initialised) {
        _velocity_predicted_ned_ms = state.velocity_ned_ms;
        _sigma_hat_ned_mss.zero();
        _sigma_filtered_ned_mss.zero();
        _initialised = true;
        _output.velocity_predicted_ned_ms = _velocity_predicted_ned_ms;
        _output.velocity_error_ned_ms.zero();
        _output.sigma_hat_ned_mss.zero();
        _output.sigma_filtered_ned_mss.zero();
        return;
    }

    const Vector3f velocity_error_ned_ms =
        _velocity_predicted_ned_ms - state.velocity_ned_ms;

    // Piecewise-constant adaptive law.  With A_s = -a_s*I every axis is
    // independent, so the matrix form
    //     sigma = -Phi^-1 * exp(A_s dt) * v_tilde
    // reduces to a scalar factor, where Phi = A_s^-1 (exp(A_s dt) - I).
    // Phi tends to dt as dt tends to zero, so Phi^-1 is already the 1/dt
    // term; multiplying by 1/dt again overshoots the gain by that factor and
    // drives the prediction error below what float32 can hold against the
    // velocities it is differencing.
    const float decay = expf(-a_s * dt);
    const float phi = (1.0f - decay) / a_s;
    if (!is_positive(phi)) {
        // Only reachable if a_s*dt underflows the exponential; holding the
        // previous estimate is safer than dividing by it.
        _output.velocity_error_ned_ms = velocity_error_ned_ms;
        _valid_time_s = 0.0f;
        return;
    }
    Vector3f sigma_hat_ned_mss = velocity_error_ned_ms * (-decay / phi);

    if (is_positive(_config.sigma_max)) {
        const float magnitude = sigma_hat_ned_mss.length();
        if (magnitude > _config.sigma_max) {
            sigma_hat_ned_mss *= _config.sigma_max / magnitude;
            _output.saturated = true;
        }
    }
    if (!is_finite(sigma_hat_ned_mss)) {
        reset();
        _output.resets++;
        return;
    }
    _sigma_hat_ned_mss = sigma_hat_ned_mss;

    apply_optional_lowpass(_sigma_hat_ned_mss, _config.filter_hz, dt,
                           _sigma_filtered_ned_mss);

    // Propagate the predictor with the estimate that was just produced, which
    // is what makes the law piecewise constant over the step.
    _velocity_predicted_ned_ms +=
        (nominal_accel_ned_mss + _sigma_hat_ned_mss - velocity_error_ned_ms * a_s) * dt;
    if (!is_finite(_velocity_predicted_ned_ms)) {
        reset();
        _output.resets++;
        return;
    }

    _output.velocity_predicted_ned_ms = _velocity_predicted_ned_ms;
    _output.velocity_error_ned_ms = velocity_error_ned_ms;
    _output.sigma_hat_ned_mss = _sigma_hat_ned_mss;
    _output.sigma_filtered_ned_mss = _sigma_filtered_ned_mss;
    _output.valid = true;

    // Overrun criterion, on the filtered estimate (8.4 decision one).
    const float filtered_mag = _sigma_filtered_ned_mss.length();
    const bool limit_set = is_positive(_config.sigma_filtered_max);
    _output.filtered_over_limit = limit_set && filtered_mag > _config.sigma_filtered_max;

    // Admission (8.4 decision three). A reset counted since the last valid
    // step restarts the quiet timer even if reset() was not what counted it.
    if (_output.resets != _resets_seen) {
        _resets_seen = _output.resets;
        _quiet_time_s = 0.0f;
    }
    _valid_time_s += dt;
    _quiet_time_s += dt;
    // Fails closed: without a configured limit the third condition cannot be
    // evaluated, so nothing is admitted.
    _output.admissible = limit_set &&
                         !_output.filtered_over_limit &&
                         _valid_time_s >= _config.admit_valid_s &&
                         _quiet_time_s >= _config.admit_quiet_s;
}
