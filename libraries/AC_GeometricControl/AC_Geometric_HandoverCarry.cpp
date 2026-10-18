#include "AC_Geometric_HandoverCarry.h"

const AP_Param::GroupInfo AC_Geometric_HandoverCarry_Params::var_info[] = {

    // @Param: HAND_TC
    // @DisplayName: Geometric output handover washout time constant
    // @Description: Washout time constant for smoothing the command discontinuity on the frame the geometric controller takes the outputs. The carry makes the first geometric command reproduce the last native one, then decays that correction with this time constant. Measured jumps on the two logged engagements were 0.035 and 0.076 in roll, 0.033 and 0.049 in pitch, and 0.022 and 0.208 in yaw, so yaw is the axis this is for. Keep it short: the measured transient decays with a 50 to 100 ms envelope, and a longer constant holds a correction the aircraft no longer needs. Start at 0.1 s. Values above 2 are clamped in code. Zero disables the carry and reproduces the behaviour measured on the aircraft.
    // @Range: 0 2
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("HAND_TC", 1, AC_Geometric_HandoverCarry_Params, _time_constant_s, 0.0f),

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

    // The geometric side is deliberately not clamped: adding this to the law's
    // raw command has to land on the native command exactly, and clamping it
    // would land somewhere between the two instead.
    const Vector3f difference = realizable(native_rpy_norm) - geometric_rpy_norm;
    const Vector3f bounded {
        constrain_float(difference.x, -MAX_OFFSET, MAX_OFFSET),
        constrain_float(difference.y, -MAX_OFFSET, MAX_OFFSET),
        constrain_float(difference.z, -MAX_OFFSET, MAX_OFFSET)
    };
    if (bounded.length() < NEGLIGIBLE) {
        return _offset;
    }
    _offset = bounded;
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
