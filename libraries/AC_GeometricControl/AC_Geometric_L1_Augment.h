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
//  - an observer integrity fault (invalid, re-seeded, non-finite) latches L1
//    off for the rest of the flight, while the geometric controller carries
//    on, and the estimate is discarded;
//  - a large estimate is not a fault: the vertical estimate is clamped to
//    sigma_limit (GEO_L1_ZMAX) before injection and L1 stays engaged, the
//    remainder left to the position integrator; an invalid limit is a clean
//    exit;
//  - engaging and a clean exit both move lambda*sigma between the
//    feed-forward and the position integrator frame by frame, so the
//    resultant force does not step: the integrator already holds whatever
//    deficit sigma reports, and engaging without the transfer would command
//    it twice;
//  - while collective is at a limit, and for HOLD_SETTLE_S after, the
//    estimate is held, because what was commanded
//    is not what was applied; the held value then walks back, never steps;
//  - a hold that runs past HOLD_MAX_S, as a limit flag that keeps re-arming
//    would make it, ends in a clean exit: past that point the frozen value
//    is a guess, and L1 off is the baseline the integrator already handles;
//  - a pause (low collective, possible landing) hands over like a clean
//    exit, so the integrator carries the deficit through it, as it would
//    with L1 off, and the force-neutral re-entry has something to take back;
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
    FAULT_OVERRUN = 5,  // no longer raised; kept so older logs decode
    FAULT_INVALID = 6,
    FAULT_RESET = 7,
    FAULT_NONFINITE = 8,
    RESET = 9,          // controller reset (integrator and observer cleared)
    HOLD_TIMEOUT = 10,  // held through saturation for longer than HOLD_MAX_S
};

struct AC_Geometric_L1_Augment_Config {
    bool enabled = false;
    float ramp_in_s = 1.0f;
    float ramp_out_s = 0.5f;
    float pause_throttle = 0.12f;
    // Largest vertical estimate L1 will inject, m/s^2 (GEO_L1_ZMAX). Not
    // positive or not finite means no valid bound, and L1 does not run.
    float sigma_limit = 5.0f;
};

// What the observer and the vehicle report on one frame.
struct AC_Geometric_L1_Augment_Inputs {
    bool observer_valid = false;
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
    // After collective leaves its limit, the estimate is still settling out
    // of what the saturation put into it (predictor plus the 2 Hz filter), so
    // the hold outlasts the flag.
    static constexpr float HOLD_SETTLE_S = 0.5f;
    // Then the held value walks back to the estimate at this rate rather than
    // stepping: 0.01 m/s^2 per frame at 400 Hz.
    static constexpr float REJOIN_RATE_MSS_PER_S = 4.0f;
    // Longest continuous hold, settle window included, so a limit that
    // lasts HOLD_MAX_S - HOLD_SETTLE_S on its own already reaches it. The
    // window re-arms on every limited frame, so without a ceiling a flag
    // that keeps flickering would hold for the rest of the flight while
    // reporting engaged.
    static constexpr float HOLD_MAX_S = 2.5f;
    // The hold time only starts again from zero after this long without a
    // hold. Clearing it on the first quiet frame let a flag that recurs just
    // slower than the settle window (every 0.52 s) slip under the ceiling.
    static constexpr float HOLD_RESET_QUIET_S = 1.0f;

    void set_config(const AC_Geometric_L1_Augment_Config& config) { _config = config; }

    void update(const AC_Geometric_L1_Augment_Inputs& in, float dt);

    // Added to the position channel's feed-forward acceleration, NED.
    Vector3f injection_ned_mss() const { return _held_ned_mss * -_lambda; }

    // Acceleration to hand to the position integrator on this frame
    // (e_I += handover / K_I). Negative while ramping in (taken out of the
    // integrator), positive while ramping out cleanly or pausing, zero
    // otherwise.
    const Vector3f& handover_ned_mss() const { return _handover_ned_mss; }

    // Ownership lost or controller reset: geometry is no longer flying, so
    // there is nothing to hand over and nothing to ramp. The latch survives.
    void drop(AC_Geometric_L1_Reason reason = AC_Geometric_L1_Reason::OWNERSHIP);

    // Only the vehicle clears a latch, on disarm.
    void clear_latch();

    AC_Geometric_L1_State state() const { return _state; }
    AC_Geometric_L1_Reason reason() const { return _reason; }
    float lambda() const { return _lambda; }
    // For the log: 0 tracking, 1 holding through a limit (in OFF, with L1
    // enabled: admission refused because of a hold, or because a timeout's
    // count has not yet been cleared), 2 walking back. 0 while ramping out,
    // paused or latched.
    uint8_t hold_status() const;
    // True on frames where what L1 injects sits at sigma_limit.
    bool clamped() const { return _clamped; }
    // How long it has sat there without a break, for the operator notice.
    float clamped_for_s() const { return _clamped_for_s; }

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
    float _hold_s = 0.0f;
    float _held_for_s = 0.0f;
    float _quiet_s = 0.0f;
    bool _rejoining = false;
    bool _clamped = false;
    float _clamped_for_s = 0.0f;
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
    AP_Float _sigma_limit;
};
