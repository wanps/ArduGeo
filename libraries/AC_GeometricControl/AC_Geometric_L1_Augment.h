#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

// L1 force-channel augmentation, vertical axis only (architecture 8.4, S3a).
//
// The observer estimates sigma, the acceleration the nominal model does not
// account for. This class decides when that estimate may enter the position
// loop and how it leaves, and produces the term the controller adds to its
// feed-forward acceleration:
//
//     A = a_d - K_x*e_x - K_v*e_v - K_I*e_I - lambda*sigma_z*e_D - g*e_D
//
// Only the vertical component is used. The horizontal one would change the
// commanded attitude R_c, whose rate target already carries high-frequency
// noise on the aircraft; it is held back until that is resolved (S3b).
//
// The contract it implements:
//  - admission is checked once, at engage, using the observer's own verdict;
//  - an observer fault latches L1 off for the rest of the flight, while the
//    geometric controller carries on, and the estimate is discarded;
//  - engaging and a clean exit both move lambda*sigma between the
//    feed-forward and the position integrator frame by frame, so the
//    resultant force does not step: the integrator already holds whatever
//    deficit sigma reports, and engaging without the transfer would command
//    it twice;
//  - while collective is at a limit the estimate is held, because what was
//    commanded is not what was applied;
//  - low collective or a possible landing pauses L1 without counting as a
//    fault, because both are ordinary pilot actions under which the nominal
//    thrust model is known to be wrong; it re-enters through admission.

enum class AC_Geometric_L1_State : uint8_t {
    OFF = 0,
    RAMP_IN = 1,
    ENGAGED = 2,
    RAMP_OUT = 3,   // clean exit, handing over to the integrator
    PAUSED = 4,     // low collective or possible landing; no handover
    LATCHED = 5,    // observer fault; off for the rest of the flight
};

enum class AC_Geometric_L1_Reason : uint8_t {
    NONE = 0,
    DISABLED = 1,
    OWNERSHIP = 2,
    PAUSE_THROTTLE = 3,
    PAUSE_LANDED = 4,
    FAULT_OVERRUN = 5,
    FAULT_INVALID = 6,
    FAULT_RESET = 7,
    FAULT_NONFINITE = 8,
    RESET = 9,          // controller reset (integrator and observer cleared)
};

struct AC_Geometric_L1_Augment_Config {
    bool enabled = false;
    float ramp_in_s = 1.0f;
    float ramp_out_s = 0.5f;
    float pause_throttle = 0.12f;
};

// What the observer and the vehicle report on one frame.
struct AC_Geometric_L1_Augment_Inputs {
    bool observer_valid = false;
    bool observer_over_limit = false;
    bool observer_admissible = false;
    uint32_t observer_resets = 0;
    Vector3f sigma_filtered_ned_mss;
    bool owns_output = false;
    bool land_complete_maybe = false;
    float throttle = 0.0f;
    // Collective is at a limit, in the mapper or in AP_Motors. What was
    // commanded is then not what was applied, and the estimate cannot be
    // trusted to separate the two.
    bool thrust_limited = false;
};

class AC_Geometric_L1_Augment {
public:
    // Removes the estimate faster than a clean exit: the value being removed
    // is the last healthy one, held, but there is no reason to keep it.
    static constexpr float FAULT_RAMP_S = 0.2f;

    void set_config(const AC_Geometric_L1_Augment_Config& config) { _config = config; }

    void update(const AC_Geometric_L1_Augment_Inputs& in, float dt);

    // Added to the position channel's feed-forward acceleration, NED.
    Vector3f injection_ned_mss() const { return _held_ned_mss * -_lambda; }

    // Acceleration to hand to the position integrator on this frame
    // (e_I += handover / K_I). Negative while ramping in (taken out of the
    // integrator), positive while ramping out cleanly, zero otherwise.
    const Vector3f& handover_ned_mss() const { return _handover_ned_mss; }

    // Ownership lost or controller reset: geometry is no longer flying, so
    // there is nothing to hand over and nothing to ramp. The latch survives.
    void drop(AC_Geometric_L1_Reason reason = AC_Geometric_L1_Reason::OWNERSHIP);

    // Only the vehicle clears a latch, on disarm.
    void clear_latch();

    AC_Geometric_L1_State state() const { return _state; }
    AC_Geometric_L1_Reason reason() const { return _reason; }
    float lambda() const { return _lambda; }

private:
    void move_lambda(float target, float rate_per_s, float dt, bool hand_over);

    AC_Geometric_L1_Augment_Config _config {};
    AC_Geometric_L1_State _state = AC_Geometric_L1_State::OFF;
    AC_Geometric_L1_Reason _reason = AC_Geometric_L1_Reason::NONE;
    float _lambda = 0.0f;
    // The vertical estimate in use; frozen once the estimate stops being
    // trusted, so a ramp-out never carries a corrupt value.
    Vector3f _held_ned_mss;
    Vector3f _handover_ned_mss;
    uint32_t _resets_seen = 0;
};

class AC_Geometric_L1_Augment_Params {
public:
    AC_Geometric_L1_Augment_Params();
    CLASS_NO_COPY(AC_Geometric_L1_Augment_Params);
    static const struct AP_Param::GroupInfo var_info[];

    AC_Geometric_L1_Augment_Config config() const;

private:
    AP_Int8 _enable;
    AP_Float _ramp_in_s;
    AP_Float _ramp_out_s;
    AP_Float _pause_throttle;
};
