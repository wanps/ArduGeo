#include "AC_Geometric_L1_Augment.h"

const AP_Param::GroupInfo AC_Geometric_L1_Augment_Params::var_info[] = {

    // @Param: L1_EN
    // @DisplayName: L1 force augmentation enable
    // @Description: Lets the L1 estimate enter the position loop, vertical axis only. Requires GEO_L1_OBS_EN; with the observer off nothing is ever admitted. Engagement is checked once, when the observer reports it admissible (GEO_L1_OBS_SFMAX, GEO_L1_OBS_ADMV, GEO_L1_OBS_ADMQ). Switching GEO_L1_OBS_EN off while engaged is treated as a clean exit, not a fault. An observer fault after that latches L1 off until the vehicle is disarmed; the geometric controller carries on without it.
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
    // @Description: Time over which the estimate is removed on a clean exit or a pause. On a clean exit what is removed each frame is handed to the position integrator, so the resultant force stays continuous. The handover needs a positive GEO_POS_KI_Z and headroom under GEO_POS_IMAX_Z; without either, the exit is a plain ramp.
    // @Range: 0.1 5
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("L1_ROUT", 3, AC_Geometric_L1_Augment_Params, _ramp_out_s, 0.5f),

    // @Param: L1_PTHR
    // @DisplayName: L1 pause collective
    // @Description: Collective below which L1 pauses rather than faults. At low collective the nominal thrust model overstates how little thrust there is, and the estimate grows: a hard throttle cut on x400b took it to 6-10 m/s/s. Pausing is not a fault and does not latch; L1 re-enters through admission once collective recovers. Absolute, not relative to hover: ground contact is only certain to pause before it overruns while this is at least about half the hover throttle (0.49 x hover at the default overrun limit), which holds on x400b (hover ~0.19) but not on an airframe hovering above ~0.25.
    // @Range: 0 0.5
    // @Increment: 0.01
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

// Out-of-class definitions: under -std=gnu++11 a static constexpr member that
// is bound to a reference (MIN/MAX take their arguments by reference) needs
// one, and an unoptimised build otherwise fails to link.
constexpr float AC_Geometric_L1_Augment::FAULT_RAMP_S;
constexpr float AC_Geometric_L1_Augment::HOLD_SETTLE_S;
constexpr float AC_Geometric_L1_Augment::REJOIN_RATE_MSS_PER_S;
constexpr float AC_Geometric_L1_Augment::HOLD_MAX_S;
constexpr float AC_Geometric_L1_Augment::HOLD_RESET_QUIET_S;

void AC_Geometric_L1_Augment::drop(AC_Geometric_L1_Reason reason)
{
    // Only a departure from an active state is worth a reason; Loiter's
    // ground-safe path resets every frame while landed and would otherwise
    // overwrite the pause that put L1 there.
    const bool active = _state != AC_Geometric_L1_State::OFF &&
                        _state != AC_Geometric_L1_State::LATCHED;
    _lambda = 0.0f;
    _handover_ned_mss.zero();
    if (active) {
        _state = AC_Geometric_L1_State::OFF;
        _reason = reason;
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

void AC_Geometric_L1_Augment::move_lambda(float target, float rate_per_s, float dt, bool hand_over)
{
    const float before = _lambda;
    if (target > _lambda) {
        _lambda = MIN(target, _lambda + rate_per_s * dt);
    } else {
        _lambda = MAX(target, _lambda - rate_per_s * dt);
    }
    if (hand_over) {
        // Whatever the feed-forward gains or loses this frame, the integrator
        // loses or gains, so -lambda*sigma - K_I*e_I is unchanged across it.
        // On the way in this matters as much as on the way out: the
        // integrator already holds whatever deficit sigma reports, and
        // without the transfer L1 would command it a second time.
        _handover_ned_mss = _held_ned_mss * (before - _lambda);
    }
}

uint8_t AC_Geometric_L1_Augment::hold_status() const
{
    const bool active = _state == AC_Geometric_L1_State::RAMP_IN ||
                        _state == AC_Geometric_L1_State::ENGAGED;
    if (active && is_positive(_hold_s)) {
        return 1;
    }
    if (_state == AC_Geometric_L1_State::OFF && _config.enabled &&
        (is_positive(_hold_s) || _held_for_s >= HOLD_MAX_S)) {
        // Off, and a hold is what is refusing admission. Gated on enabled
        // because on the ground AP_Motors keeps the limit flags set.
        return 1;
    }
    return (active && _rejoining) ? 2 : 0;
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
    // A large estimate is not evidence that the estimate is broken: a real
    // step in load overshoots in it, and on an airframe with thrust to spare
    // a real disturbance can stay above any fixed limit for seconds with
    // collective free. So the estimate is clamped and injected, as a
    // projection onto a bounded set, rather than dropped. The bound is the
    // same value that used to be the fault threshold, so L1's authority is
    // what it was. Integrity faults (invalid, reset, non-finite) still latch.
    // Only the vertical axis counts, since only the vertical axis is
    // injected; horizontal drag at speed is no reason to drop it.
    const float limit = _config.sigma_limit;
    const bool over = finite && is_positive(limit) && fabsf(sigma_v.z) > limit;
    Vector3f target = sigma_v;
    if (over) {
        target.z = constrain_float(sigma_v.z, -limit, limit);
    }
    _clamped = false;

    if (in.thrust_limited) {
        _hold_s = HOLD_SETTLE_S;
    } else {
        _hold_s = MAX(0.0f, _hold_s - dt);
    }
    const bool holding = is_positive(_hold_s);
    if (holding) {
        _quiet_s = 0.0f;
        _held_for_s = MIN(_held_for_s + dt, HOLD_MAX_S);
    } else {
        _quiet_s += dt;
        if (_quiet_s >= HOLD_RESET_QUIET_S) {
            _held_for_s = 0.0f;
        }
    }

    const bool pause_landed = in.land_complete_maybe;
    const bool pause_throttle = in.throttle < _config.pause_throttle;
    const float rate_in = 1.0f / _config.ramp_in_s;
    const float rate_out = 1.0f / _config.ramp_out_s;

    switch (_state) {
    case AC_Geometric_L1_State::OFF:
        _lambda = 0.0f;
        _resets_seen = in.observer_resets;
        // After a timeout the count stays at the ceiling until a quiet
        // second clears it; admitting before that would time out again on
        // the next limited frame, once per flicker.
        if (_config.enabled && in.owns_output && !pause_landed && !pause_throttle &&
            !holding && _held_for_s < HOLD_MAX_S && in.observer_admissible && finite) {
            _held_ned_mss = target;
            _rejoining = false;
            _state = AC_Geometric_L1_State::RAMP_IN;
            _reason = AC_Geometric_L1_Reason::NONE;
        }
        break;

    case AC_Geometric_L1_State::RAMP_IN:
    case AC_Geometric_L1_State::ENGAGED: {
        // A deliberate disable is a clean exit even if the observer has just
        // been switched off with it: the held value is the last healthy one.
        if (!_config.enabled) {
            _state = AC_Geometric_L1_State::RAMP_OUT;
            _reason = AC_Geometric_L1_Reason::DISABLED;
            move_lambda(0.0f, rate_out, dt, true);
            break;
        }
        AC_Geometric_L1_Reason fault = AC_Geometric_L1_Reason::NONE;
        if (!finite) {
            fault = AC_Geometric_L1_Reason::FAULT_NONFINITE;
        } else if (!in.observer_valid) {
            fault = AC_Geometric_L1_Reason::FAULT_INVALID;
        } else if (in.observer_resets != _resets_seen) {
            fault = AC_Geometric_L1_Reason::FAULT_RESET;
        }
        if (fault != AC_Geometric_L1_Reason::NONE) {
            // _held_ned_mss keeps the last healthy value.
            _state = AC_Geometric_L1_State::LATCHED;
            _reason = fault;
            move_lambda(0.0f, 1.0f / FAULT_RAMP_S, dt, false);
            break;
        }
        if (!in.owns_output) {
            drop(AC_Geometric_L1_Reason::OWNERSHIP);
            break;
        }
        if (pause_landed || pause_throttle) {
            _state = AC_Geometric_L1_State::PAUSED;
            _reason = pause_landed ? AC_Geometric_L1_Reason::PAUSE_LANDED
                                   : AC_Geometric_L1_Reason::PAUSE_THROTTLE;
            // The held value is the last one with collective above the
            // pause threshold, as healthy as a clean exit's, and the
            // integrator carries it from here as it would with L1 off.
            move_lambda(0.0f, rate_out, dt, true);
            break;
        }
        if (holding) {
            if (_held_for_s >= HOLD_MAX_S) {
                // The held value was healthy when the hold began, so it is
                // handed over like any clean exit; re-entry waits for the
                // limit to go quiet and for admission.
                _state = AC_Geometric_L1_State::RAMP_OUT;
                _reason = AC_Geometric_L1_Reason::HOLD_TIMEOUT;
                move_lambda(0.0f, rate_out, dt, true);
                break;
            }
            // Saturated, or just out of it: what was commanded is not what
            // was applied, and the estimate has absorbed some of the
            // shortfall. Hold the last value and lambda where they are;
            // neither a fault nor a reason to leave.
            _rejoining = true;
            break;
        }
        _clamped = over;
        if (_rejoining) {
            const float step = REJOIN_RATE_MSS_PER_S * dt;
            const float gap = target.z - _held_ned_mss.z;
            _held_ned_mss.z += constrain_float(gap, -step, step);
            _rejoining = fabsf(gap) > step;
        } else {
            _held_ned_mss = target;
        }
        move_lambda(1.0f, rate_in, dt, true);
        _state = (_lambda >= 1.0f) ? AC_Geometric_L1_State::ENGAGED : AC_Geometric_L1_State::RAMP_IN;
        break;
    }

    case AC_Geometric_L1_State::RAMP_OUT:
        if (!in.owns_output) {
            drop(AC_Geometric_L1_Reason::OWNERSHIP);
            break;
        }
        move_lambda(0.0f, rate_out, dt, true);
        if (!is_positive(_lambda)) {
            _state = AC_Geometric_L1_State::OFF;
        }
        break;

    case AC_Geometric_L1_State::PAUSED:
        if (!in.owns_output) {
            drop(AC_Geometric_L1_Reason::OWNERSHIP);
            break;
        }
        move_lambda(0.0f, rate_out, dt, true);
        if (!is_positive(_lambda)) {
            _state = AC_Geometric_L1_State::OFF;
        }
        break;

    case AC_Geometric_L1_State::LATCHED:
        move_lambda(0.0f, 1.0f / FAULT_RAMP_S, dt, false);
        break;
    }
}
