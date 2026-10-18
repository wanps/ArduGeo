#include "AC_Geometric_HandoverCarry.h"

const AP_Param::GroupInfo AC_Geometric_HandoverCarry_Params::var_info[] = {

    // @Param: HAND_TC
    // @DisplayName: Geometric output handover washout time constant
    // @Description: Time constant for carrying the native controller's trim across the frame the geometric controller takes the outputs. The native rate loop holds a steady command against wind in its integrator; the geometric roll and pitch channels have no integrator, so without this the command steps by that trim and the aircraft receives a kick. The offset is seeded at the handover and decays with this time constant, so the geometric controller takes the trim over through its own attitude error rather than all at once. Zero disables the carry and reproduces the behaviour measured on the aircraft.
    // @Range: 0 5
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("HAND_TC", 1, AC_Geometric_HandoverCarry_Params, _time_constant_s, 0.0f),

    AP_GROUPEND
};

// Bounds one axis by the native integrator's own limit. A limit that is zero,
// negative or not finite leaves the axis at zero rather than unbounded: an
// integrator that cannot wind up is holding no trim worth carrying, and an
// unusable limit is not grounds for trusting the difference it failed to bound.
static float bounded_trim(float difference, float limit)
{
    if (!isfinite(limit) || !is_positive(limit)) {
        return 0.0f;
    }
    return constrain_float(difference, -limit, limit);
}

const Vector3f& AC_Geometric_HandoverCarry::seed(const Vector3f& native_trim_norm,
                                                 const Vector3f& geometric_trim_norm,
                                                 const Vector3f& limit_norm)
{
    _offset.zero();
    if (!is_positive(_config.time_constant_s)) {
        return _offset;
    }
    if (native_trim_norm.is_nan() || native_trim_norm.is_inf() ||
        geometric_trim_norm.is_nan() || geometric_trim_norm.is_inf() ||
        limit_norm.is_nan() || limit_norm.is_inf()) {
        return _offset;
    }

    const Vector3f difference = native_trim_norm - geometric_trim_norm;
    const Vector3f bounded {
        bounded_trim(difference.x, limit_norm.x),
        bounded_trim(difference.y, limit_norm.y),
        bounded_trim(difference.z, limit_norm.z)
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
