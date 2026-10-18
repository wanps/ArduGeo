#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>

// Smooths the command discontinuity on the frame the geometric controller
// takes the outputs.
//
// Measured at the AP_Motors interface on the two logged engagements -- RATE
// keeps reporting after the handover because the geometric path writes through
// the same setters, and its timestamp freezing marks the boundary exactly:
//
//            roll     pitch    yaw
//   log_15  +0.0352  +0.0325  +0.0223
//   log_19  -0.0762  -0.0493  +0.2078
//
// Yaw on the second flight is the event worth smoothing. Roll and pitch are
// inside the geometric law's own per-frame command activity, which runs a
// median single-frame step of 0.015 to 0.030 once it owns the outputs.
//
// The seed is the whole difference, which makes the first geometric command
// reproduce the last native one exactly. An earlier revision carried only the
// native rate integrator, on the reasoning that P and D answer instantaneous
// error the geometric law answers for itself; against these jumps that left
// 0.009 to 0.215, so the whole difference is simply the better seed and the
// reasoning is beside the point.
//
// What this is not: a transfer of trim the geometric law lacks. The difference
// in mean command over a second either side is 0.002 to 0.022, well below the
// jump, because two controllers holding the same aircraft have to supply
// similar mean moments. That is a statement about the second after the
// handover, not about steady state. The law has no roll or pitch integrator
// and builds its share out of attitude error over tens of seconds: GEOA.ERx
// averages +0.001 and +0.003 in the second before the engage, and -0.119 and
// +0.025 thirty seconds after.
//
// The washout should therefore be short. A long one holds stale content open
// loop and delays that error from forming, parking the aircraft at the wrong
// attitude.
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

    // The command the motors should receive once an offset is applied, and
    // whether reaching it required limiting.
    struct Applied {
        Vector3f rpy_norm;
        bool limited;
    };

    // Seeds the carry from the two commands either side of the handover, in
    // normalized actuator units. The geometric side is the law's raw command,
    // so that adding the result reproduces the native command exactly. Zero
    // when the carry is disabled or either input is not finite.
    //
    // Returns zero in two cases, both falling back to the default-off
    // behaviour both logged flights flew.
    //
    // When this law's own command is outside the realizable range:
    // reproducing the last native command then means withholding authority
    // the law is asking for, which is the wrong way to be wrong. Refusing on
    // any axis refuses all three. The cost is asymmetric in the right
    // direction -- a saturated axis clamps to its rail with or without a
    // carry, so what is withheld is smoothing on the other two, and those are
    // roll and pitch, whose jumps sit inside the law's own per-frame activity.
    //
    // And when the two commands differ by more than MAX_OFFSET on any axis.
    // Neither flight came within a factor of two of that, so this is a guard
    // rather than a working path.
    //
    // The native side is clamped, because AP_Motors documents its -1..+1 range
    // without enforcing it.
    const Vector3f& seed(const Vector3f& native_rpy_norm,
                         const Vector3f& geometric_rpy_norm);

    // Applies an offset to the law's raw command without writing to it. The
    // raw command is the law's intent, read by the vehicle's finite check and
    // logged as GEOM; an earlier revision added the offset there, which made a
    // saturated law look unsaturated to the output-rail gate.
    //
    // mapper_limited is the flag the mapper already set. It is carried through
    // rather than recomputed, because either cause of reaching the rail is
    // worth reporting: the law asking for more than the mapper could give, or
    // the carry pushing what it could give past the limit.
    static Applied apply(const Vector3f& rpy_norm_raw, const Vector3f& offset,
                         bool mapper_limited);

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

    // The largest disagreement this will smooth, per axis: 2.4 times the
    // largest jump measured on either flight (0.208, yaw, log_19). Beyond it
    // the seed is refused, which costs one frame of a step this size and
    // saves holding that much unrequested command for the whole decay.
    static constexpr float MAX_OFFSET = 0.5f;

    AC_Geometric_HandoverCarry_Config _config {};
    Vector3f _offset;
};

class AC_Geometric_HandoverCarry_Params {
public:
    AC_Geometric_HandoverCarry_Params() { AP_Param::setup_object_defaults(this, var_info); }

    CLASS_NO_COPY(AC_Geometric_HandoverCarry_Params);

    static const struct AP_Param::GroupInfo var_info[];

    // The parameter's declared range is metadata the ground station honours and
    // nothing else does. Five times the suggested starting value of 0.1 s,
    // which is already twice the measured transient envelope.
    static constexpr float MAX_TIME_CONSTANT_S = 0.5f;

    static float clamped_time_constant(float requested) {
        // A non-finite value is corruption, not a request, and is treated as
        // off. A finite one, however large, is treated as a request and
        // clamped. constrain_float would otherwise return the midpoint of its
        // range for NaN, turning a corrupt parameter into an enabled carry
        // where is_positive had rejected it before the clamp existed.
        if (!isfinite(requested)) {
            return 0.0f;
        }
        return constrain_float(requested, 0.0f, MAX_TIME_CONSTANT_S);
    }

    AC_Geometric_HandoverCarry_Config config() const {
        return AC_Geometric_HandoverCarry_Config {
            clamped_time_constant(_time_constant_s.get()) };
    }

private:
    AP_Float _time_constant_s;
};
