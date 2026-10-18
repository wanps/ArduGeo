#include "AC_Geometric_HandoverCarry.h"

const AP_Param::GroupInfo AC_Geometric_HandoverCarry_Params::var_info[] = {

    // @Param: HAND_TC
    // @DisplayName: Geometric output handover washout time constant
    // @Description: Washout time constant for smoothing the command discontinuity on the frame the geometric controller takes the outputs. The two logged handovers jumped 0.050 and 0.059 of full scale; the carry cancels that jump on the first frame and decays the correction with this time constant. Keep it short. The jump is fast content, not a trim the geometric law lacks: the difference in mean command over a second either side of those same handovers was 0.002 to 0.022, so what the washout decays toward is already near zero and a long constant only holds stale content open loop. Start around 0.1 to 0.2 s. Zero disables the carry and reproduces the behaviour measured on the aircraft.
    // @Range: 0 2
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("HAND_TC", 1, AC_Geometric_HandoverCarry_Params, _time_constant_s, 0.0f),

    AP_GROUPEND
};

// AP_Motors documents set_roll/pitch/yaw as -1..+1 but does not enforce it, and
// the geometric side of the subtraction is the command before the mapper's own
// limiting. Clamping both to the realizable range is what bounds the offset.
static Vector3f realizable(const Vector3f& command)
{
    return Vector3f {
        constrain_float(command.x, -1.0f, 1.0f),
        constrain_float(command.y, -1.0f, 1.0f),
        constrain_float(command.z, -1.0f, 1.0f)
    };
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

    const Vector3f difference = realizable(native_rpy_norm) - realizable(geometric_rpy_norm);
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
