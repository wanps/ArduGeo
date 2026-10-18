#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>

// Smooths the command discontinuity on the frame the geometric controller
// takes the outputs.
//
// The two logged handovers jumped 0.050 and 0.059 of full scale between the
// last native command and the first geometric one, with the gyro peaking at
// 15.9 and 14.4 deg/s against a 5 to 7 deg/s background.
//
// What it is not: a transfer of trim the geometric law lacks. Decomposing
// those same handovers, the difference in mean command over a second either
// side is 0.002 to 0.022 -- an order of magnitude below the jump -- and the
// native integrator is larger than it on four of the six axes, with
// inconsistent sign. In steady flight any controller holding the aircraft must
// supply the same mean moment or the aircraft drifts, so the persistent
// difference between two stable controllers is near zero by construction. The
// geometric law gets that moment from attitude error, because the aircraft is
// already leaning into the wind.
//
// So the discontinuity is fast content, and the seed is the whole command
// difference, which cancels it exactly on the first frame. The washout should
// be short for the same reason: what it decays toward is a persistent
// difference already near zero, and a long constant holds stale fast content
// open loop instead.
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

    // Seeds the carry from the two commands either side of the handover, in
    // normalized actuator units. Returns the offset to add to the geometric
    // command, which is zero when the carry is disabled or either input is not
    // finite.
    //
    // Both inputs are clamped to the command range first, which is what bounds
    // the result. AP_Motors documents that range but does not enforce it, and
    // the geometric side is the pre-limit command, so without this a law
    // demanding three times full scale would seed an offset of -2.6 and the
    // washout would hold the aircraft's command down for seconds while the
    // controller was saturated. Clamping makes the offset the difference of two
    // realizable commands, so the carry can neither hide a saturation nor
    // undo one.
    const Vector3f& seed(const Vector3f& native_rpy_norm,
                         const Vector3f& geometric_rpy_norm);

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
