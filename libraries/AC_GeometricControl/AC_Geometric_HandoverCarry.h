#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>

// Carries the native controller's trim across the frame the geometric
// controller takes the outputs.
//
// The native rate loop holds a steady command against wind in its integrator.
// The geometric roll and pitch channels default to no integrator, so at the
// handover that trim disappears from the command and the aircraft receives a
// step: the aircraft showed 0.060 and 0.068 of full scale on two flights, with
// the gyro peaking at 15.9 and 14.4 deg/s against a 5 to 7 deg/s background.
//
// What transfers is the two loops' trims, not their commands. The rest of the
// native output is P and D answering instantaneous error, and the geometric law
// is computing its own answer to that same error in the same frame, from the
// same gyro. Carrying it would transfer a transient open loop for the length of
// the washout and count the error twice. The yaw axis has a geometric
// integrator by default, so the seed is a difference rather than the native
// trim alone.
//
// It is a transient rather than a divergence. The geometric controller re-earns
// the trim through attitude error, and its mean command one second after the
// handover matched the native mean one second before. This seeds the difference
// as an offset and washes it out, so the trim transfers gradually instead of
// being dropped and rebuilt.
//
// A non-positive time constant disables the carry entirely, which reproduces
// the behaviour those two flights measured.
struct AC_Geometric_HandoverCarry_Config {
    float time_constant_s;
};

class AC_Geometric_HandoverCarry {
public:
    AC_Geometric_HandoverCarry() = default;

    CLASS_NO_COPY(AC_Geometric_HandoverCarry);

    void set_config(const AC_Geometric_HandoverCarry_Config& config) { _config = config; }

    // Seeds the carry from the trim each loop is holding, in normalized
    // actuator units. Returns the offset to add to the geometric command, which
    // is zero when the carry is disabled or any input is not finite.
    //
    // limit_norm bounds the result per axis, and the bound that means something
    // is the native integrator's own imax: nothing larger can be trim, so
    // anything larger is a transient or a saturated geometric command and
    // holding it for the washout would make the handover worse than the step it
    // replaces. A zero or non-finite limit disables that axis, which is right --
    // an integrator that cannot wind up holds no trim to carry.
    const Vector3f& seed(const Vector3f& native_trim_norm,
                         const Vector3f& geometric_trim_norm,
                         const Vector3f& limit_norm);

    // Decays the offset by one step and returns it. Zero once the carry has
    // decayed below the command resolution that reaches the motors, so a dead
    // term is not kept alive.
    const Vector3f& update(float dt);

    void reset() { _offset.zero(); }

    const Vector3f& offset() const { return _offset; }
    bool active() const { return !_offset.is_zero(); }

private:
    // Below this the offset is smaller than the resolution that survives to the
    // motors, so holding it serves no purpose.
    static constexpr float NEGLIGIBLE = 1.0e-4f;

    AC_Geometric_HandoverCarry_Config _config {};
    Vector3f _offset;
};

class AC_Geometric_HandoverCarry_Params {
public:
    AC_Geometric_HandoverCarry_Params() { AP_Param::setup_object_defaults(this, var_info); }

    CLASS_NO_COPY(AC_Geometric_HandoverCarry_Params);

    static const struct AP_Param::GroupInfo var_info[];

    AC_Geometric_HandoverCarry_Config config() const {
        return AC_Geometric_HandoverCarry_Config { _time_constant_s.get() };
    }

private:
    AP_Float _time_constant_s;
};
