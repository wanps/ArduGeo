#include "AC_Geometric_L1_Augment.h"

const AP_Param::GroupInfo AC_Geometric_L1_Augment_Params::var_info[] = {

    // @Param: L1_EN
    // @DisplayName: L1 force augmentation enable
    // @Description: Lets the L1 estimate enter the position loop, vertical axis only. Requires GEO_L1_OBS_EN; with the observer off nothing is ever admitted. Engagement is checked once, when the observer reports it admissible (GEO_L1_OBS_SFMAX, ADMV, ADMQ). An observer fault after that latches L1 off until the vehicle is disarmed; the geometric controller carries on without it.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("L1_EN", 1, AC_Geometric_L1_Augment_Params, _enable, 0),

    // @Param: L1_RIN
    // @DisplayName: L1 engage ramp
    // @Description: Time over which the estimate is brought in after admission, so engaging does not step the collective.
    // @Range: 0.1 5
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("L1_RIN", 2, AC_Geometric_L1_Augment_Params, _ramp_in_s, 1.0f),

    // @Param: L1_ROUT
    // @DisplayName: L1 clean-exit ramp
    // @Description: Time over which the estimate is removed on a clean exit or a pause. On a clean exit what is removed each frame is handed to the position integrator, so the resultant force stays continuous.
    // @Range: 0.1 5
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("L1_ROUT", 3, AC_Geometric_L1_Augment_Params, _ramp_out_s, 0.5f),

    // @Param: L1_PTHR
    // @DisplayName: L1 pause collective
    // @Description: Collective below which L1 pauses rather than faults. At low collective the nominal thrust model overstates how little thrust there is, and the estimate grows: a hard throttle cut on x400b took it to 6-10 m/s/s. Pausing is not a fault and does not latch; L1 re-enters through admission once collective recovers.
    // @Range: 0 0.5
    // @User: Advanced
    AP_GROUPINFO("L1_PTHR", 4, AC_Geometric_L1_Augment_Params, _pause_throttle, 0.12f),

    AP_GROUPEND
};

AC_Geometric_L1_Augment_Params::AC_Geometric_L1_Augment_Params()
{
    AP_Param::setup_object_defaults(this, var_info);
}

AC_Geometric_L1_Augment_Config AC_Geometric_L1_Augment_Params::config() const
{
    // Non-finite or non-positive ramps would divide by zero or never finish;
    // fall back to the defaults rather than to an instant step.
    auto ramp = [](float v, float dflt) {
        return (isfinite(v) && is_positive(v)) ? constrain_float(v, 0.1f, 5.0f) : dflt;
    };
    AC_Geometric_L1_Augment_Config c {};
    c.enabled = _enable.get() != 0;
    c.ramp_in_s = ramp(_ramp_in_s.get(), 1.0f);
    c.ramp_out_s = ramp(_ramp_out_s.get(), 0.5f);
    const float thr = _pause_throttle.get();
    c.pause_throttle = isfinite(thr) ? constrain_float(thr, 0.0f, 0.5f) : 0.12f;
    return c;
}

void AC_Geometric_L1_Augment::drop()
{
    _lambda = 0.0f;
    _handover_ned_mss.zero();
    if (_state != AC_Geometric_L1_State::LATCHED) {
        _state = AC_Geometric_L1_State::OFF;
        _reason = AC_Geometric_L1_Reason::OWNERSHIP;
    }
}

void AC_Geometric_L1_Augment::clear_latch()
{
    if (_state == AC_Geometric_L1_State::LATCHED) {
        _state = AC_Geometric_L1_State::OFF;
        _reason = AC_Geometric_L1_Reason::NONE;
        _lambda = 0.0f;
        _held_ned_mss.zero();
    }
}

void AC_Geometric_L1_Augment::ramp_down(float dt, float ramp_s, bool hand_over)
{
    const float before = _lambda;
    _lambda = MAX(0.0f, _lambda - dt / ramp_s);
    if (hand_over) {
        // Whatever leaves the feed-forward this frame enters the integrator,
        // so -lambda*sigma - K_I*e_I is unchanged across the frame.
        _handover_ned_mss = _held_ned_mss * (before - _lambda);
    }
}

void AC_Geometric_L1_Augment::update(const AC_Geometric_L1_Augment_Inputs& in, float dt)
{
    _handover_ned_mss.zero();
    if (!is_positive(dt) || !isfinite(dt)) {
        return;
    }
    // S3a: vertical only, by construction rather than by parameter.
    const Vector3f sigma_v { 0.0f, 0.0f, in.sigma_filtered_ned_mss.z };
    const bool finite = isfinite(in.sigma_filtered_ned_mss.z);

    const bool pause_landed = in.land_complete_maybe;
    const bool pause_throttle = in.throttle < _config.pause_throttle;

    switch (_state) {
    case AC_Geometric_L1_State::OFF:
        _lambda = 0.0f;
        _resets_seen = in.observer_resets;
        if (_config.enabled && in.owns_output && !pause_landed && !pause_throttle &&
            in.observer_admissible && finite) {
            _held_ned_mss = sigma_v;
            _state = AC_Geometric_L1_State::RAMP_IN;
            _reason = AC_Geometric_L1_Reason::NONE;
        }
        break;

    case AC_Geometric_L1_State::RAMP_IN:
    case AC_Geometric_L1_State::ENGAGED: {
        AC_Geometric_L1_Reason fault = AC_Geometric_L1_Reason::NONE;
        if (!finite) {
            fault = AC_Geometric_L1_Reason::FAULT_NONFINITE;
        } else if (!in.observer_valid) {
            fault = AC_Geometric_L1_Reason::FAULT_INVALID;
        } else if (in.observer_resets != _resets_seen) {
            fault = AC_Geometric_L1_Reason::FAULT_RESET;
        } else if (in.observer_over_limit && !pause_throttle && !pause_landed) {
            fault = AC_Geometric_L1_Reason::FAULT_OVERRUN;
        }
        if (fault != AC_Geometric_L1_Reason::NONE) {
            // _held_ned_mss keeps the last healthy value.
            _state = AC_Geometric_L1_State::LATCHED;
            _reason = fault;
            ramp_down(dt, FAULT_RAMP_S, false);
            break;
        }
        if (!in.owns_output) {
            drop();
            break;
        }
        if (pause_landed || pause_throttle) {
            _state = AC_Geometric_L1_State::PAUSED;
            _reason = pause_landed ? AC_Geometric_L1_Reason::PAUSE_LANDED
                                   : AC_Geometric_L1_Reason::PAUSE_THROTTLE;
            ramp_down(dt, _config.ramp_out_s, false);
            break;
        }
        if (!_config.enabled) {
            _state = AC_Geometric_L1_State::RAMP_OUT;
            _reason = AC_Geometric_L1_Reason::DISABLED;
            ramp_down(dt, _config.ramp_out_s, true);
            break;
        }
        _held_ned_mss = sigma_v;
        _lambda = MIN(1.0f, _lambda + dt / _config.ramp_in_s);
        _state = (_lambda >= 1.0f) ? AC_Geometric_L1_State::ENGAGED : AC_Geometric_L1_State::RAMP_IN;
        break;
    }

    case AC_Geometric_L1_State::RAMP_OUT:
        if (!in.owns_output) {
            drop();
            break;
        }
        ramp_down(dt, _config.ramp_out_s, true);
        if (!is_positive(_lambda)) {
            _state = AC_Geometric_L1_State::OFF;
        }
        break;

    case AC_Geometric_L1_State::PAUSED:
        ramp_down(dt, _config.ramp_out_s, false);
        if (!is_positive(_lambda)) {
            _state = AC_Geometric_L1_State::OFF;
        }
        break;

    case AC_Geometric_L1_State::LATCHED:
        ramp_down(dt, FAULT_RAMP_S, false);
        break;
    }
}
