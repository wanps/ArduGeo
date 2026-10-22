#include "AC_Geometric_HandoverCarry.h"

const AP_Param::GroupInfo AC_Geometric_HandoverCarry_Params::var_info[] = {

    // @Param: HAND_TC
    // @DisplayName: Geometric output handover washout time constant
    // @Description: Washout time constant for smoothing the command discontinuity on the frame the geometric controller takes the outputs. The carry makes the first geometric command reproduce the last native one, then decays that correction with this time constant. Measured jumps on the two logged engagements were 0.035 and 0.076 in roll, 0.033 and 0.049 in pitch, and 0.022 and 0.208 in yaw, so yaw is the axis this is for. Keep it short: the measured transient decays with a 50 to 100 ms envelope, and a longer constant holds a correction the aircraft no longer needs. Start at 0.1 s. Values above 0.5 are clamped in code. Zero disables the carry and reproduces the behaviour measured on the aircraft.
    // @Range: 0 0.5
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("HAND_TC", 1, AC_Geometric_HandoverCarry_Params, _time_constant_s, 0.0f),

    // @Param: HAND_PRE
    // @DisplayName: Preload the native rate loop when geometric output ends
    // @Description: When the geometric controller gives the outputs back, the native rate loop is relaxed, which resets its integrators and leaves it commanding zero. Both logged handoffs show the native output resuming at exactly 0.0000 against a geometric command of 0.041/0.052/0.003 and 0.013/0.017/0.017, so the step is the whole command. With this enabled the rate integrators are seeded with that command instead, which makes the handoff continuous because the relax leaves the proportional, derivative and feedforward terms at zero on that frame. This is the fail-closed path and no flight has used it; default is off.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("HAND_PRE", 2, AC_Geometric_HandoverCarry_Params, _preload_on_fallback, 0),

    // @Param: HAND_PFLT
    // @DisplayName: Fallback preload trim filter
    // @Description: Cut-off of the low-pass applied to the geometric command while it owns the outputs, whose value GEO_HAND_PRE loads into the native rate integrators on fallback. On x400b the geometric roll command carried high-frequency noise of 0.06 against a trim of 0.04, so loading the last frame as it stood loaded a noise sample: preload errors of 0.019, 0.083 and 0.063 on three real handoffs, the largest followed by a 7 degree roll excursion where no preload gave 2. The rate integrator is meant to hold the trim, not the noise. Zero loads the last frame unfiltered, which reproduces the earlier behaviour.
    // @Range: 0 20
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("HAND_PFLT", 3, AC_Geometric_HandoverCarry_Params, _preload_filter_hz, 1.0f),

    AP_GROUPEND
};

// AP_Motors documents set_roll/pitch/yaw as -1..+1 but does not enforce it.
static Vector3f realizable(const Vector3f& command)
{
    return Vector3f {
        constrain_float(command.x, -1.0f, 1.0f),
        constrain_float(command.y, -1.0f, 1.0f),
        constrain_float(command.z, -1.0f, 1.0f)
    };
}

AC_Geometric_HandoverCarry::Applied
AC_Geometric_HandoverCarry::apply(const Vector3f& rpy_norm_raw, const Vector3f& offset,
                                  bool mapper_limited)
{
    const Vector3f carried = rpy_norm_raw + offset;
    const Vector3f limited = realizable(carried);
    const bool clipped = !is_equal(limited.x, carried.x) ||
                         !is_equal(limited.y, carried.y) ||
                         !is_equal(limited.z, carried.z);
    return Applied { limited, mapper_limited || clipped };
}

const Vector3f& AC_Geometric_HandoverCarry::seed(const Vector3f& native_rpy_norm,
                                                 const Vector3f& geometric_rpy_norm)
{
    _offset.zero();
    if (!is_positive(_config.time_constant_s)) {
        return _offset;
    }
    if (native_rpy_norm.is_nan() || native_rpy_norm.is_inf() ||
        geometric_rpy_norm.is_nan() || geometric_rpy_norm.is_inf()) {
        return _offset;
    }

    // A saturated law is refused rather than bounded. Seeding against it would
    // hold the command near what native wanted while the law is asking for
    // more than full authority.
    if (geometric_rpy_norm != realizable(geometric_rpy_norm)) {
        return _offset;
    }

    // The geometric side is deliberately not clamped -- it is already in range
    // by the check above -- so adding this to the law's raw command lands on
    // the native command exactly.
    const Vector3f difference = realizable(native_rpy_norm) - geometric_rpy_norm;

    // Refused, not truncated, and the trade is not one-sided. Refusing leaves
    // an engage step MAX_OFFSET larger than truncating would -- worst just
    // above the bound, where truncation would have smoothed nearly all of it.
    // What it buys is that nothing the law never asked for is held afterwards:
    // truncation holds MAX_OFFSET of that through the decay, hundreds of
    // frames, below the rail where no judgement reports it, while the excess
    // step survives anyway. One frame of a larger step against hundreds of a
    // command nobody chose, and refusing falls back to the behaviour both
    // logged flights flew.
    if (fabsf(difference.x) > MAX_OFFSET ||
        fabsf(difference.y) > MAX_OFFSET ||
        fabsf(difference.z) > MAX_OFFSET) {
        return _offset;
    }
    if (difference.length() < NEGLIGIBLE) {
        return _offset;
    }
    _offset = difference;
    return _offset;
}

const Vector3f& AC_Geometric_HandoverCarry::update(float dt)
{
    if (_offset.is_zero()) {
        return _offset;
    }
    if (!is_positive(_config.time_constant_s) || !is_positive(dt) || !isfinite(dt)) {
        _offset.zero();
        return _offset;
    }

    _offset *= expf(-dt / _config.time_constant_s);
    if (_offset.length() < NEGLIGIBLE) {
        _offset.zero();
    }
    return _offset;
}

void AC_Geometric_HandoverTrim::seed(const Vector3f& command)
{
    if (command.is_nan() || command.is_inf()) {
        return;
    }
    _value = command;
    _valid = true;
}

void AC_Geometric_HandoverTrim::update(const Vector3f& command, float dt, float cutoff_hz)
{
    // A non-finite command is corruption, and the motors were not given it
    // either: AP_Motors would have been fed the same frame, and the vehicle's
    // validity gate rejects such output before it gets there.
    if (command.is_nan() || command.is_inf()) {
        return;
    }
    if (!_valid) {
        seed(command);
        return;
    }
    if (!is_positive(cutoff_hz) || !isfinite(cutoff_hz)) {
        // Off: hold the last frame as written, which is what the preload
        // loaded before this filter existed.
        _value = command;
        return;
    }
    if (!is_positive(dt) || !isfinite(dt)) {
        return;
    }
    _value += (command - _value) * calc_lowpass_alpha_dt(dt, cutoff_hz);
}

void AC_Geometric_HandoverTrim::reset()
{
    _value.zero();
    _valid = false;
}
