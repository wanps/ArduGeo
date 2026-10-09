#pragma once

#include <AP_Param/AP_Param.h>

#include "AC_Geometric_Types.h"

// Persistent configuration for the L1 translational observer.
class AC_Geometric_L1_Observer_Params {
public:
    AC_Geometric_L1_Observer_Params();

    static const AP_Param::GroupInfo var_info[];

    bool enabled() const;
    AC_Geometric_L1_Observer_Config config() const;

private:
    AP_Int8 _enable;
    AP_Float _predictor_gain;
    AP_Float _filter_hz;
    AP_Float _sigma_max;
};

// Observer-only L1 adaptive estimator for the translational channel.
//
// It estimates the matched uncertainty sigma that the nominal geometric model
// does not account for, and it does nothing else: the class exposes no command
// and the controller never reads its output into a force, moment or motor
// value.  R-23 requires an independent observer stage before any new active
// path, and this is that stage for the L1 work.
//
// Structure (Hovakimyan/Cao piecewise-constant adaptive law, velocity channel):
//
//   nominal      v_dot = -f * (R e_D) + g_ned
//   predictor    v_hat_dot = v_dot + sigma_hat - a_s * (v_hat - v)
//   adaptive     sigma_hat = -Phi^-1 * exp(A_s dt) * (v_hat - v)
//
// with A_s = -a_s * I, so Phi = (1 - exp(-a_s dt)) / a_s and every axis is
// independent.  The estimate is then low-passed; the filter exists because the
// control law that will eventually consume sigma needs it bandwidth-limited,
// and running it here keeps the observer's output identical to what that law
// would see.
//
// The projection is deliberately the full NED vector rather than body z alone.
// L1Quad projects its thrust-channel estimate onto body z because its own
// control law only has authority there; this stage is diagnostic, so narrowing
// it would discard the components that decide whether a lateral disturbance is
// observable at all.
class AC_Geometric_L1_Observer {
public:
    AC_Geometric_L1_Observer();
    CLASS_NO_COPY(AC_Geometric_L1_Observer);

    void set_config(const AC_Geometric_L1_Observer_Config& config);

    // Re-seed the predictor from the measured state so that resuming after a
    // gap, a mode change or a fault cannot inject a step into sigma.
    void reset();

    // Advance one step.  Never writes anything the controller consumes.
    void update(const AC_Geometric_State& state,
                const AC_Geometric_Position_Output& position,
                float dt);

    const AC_Geometric_L1_Observer_Output& get_output() const { return _output; }

private:
    AC_Geometric_L1_Observer_Config _config;
    AC_Geometric_L1_Observer_Output _output;

    Vector3f _velocity_predicted_ned_ms;
    Vector3f _sigma_hat_ned_mss;
    Vector3f _sigma_filtered_ned_mss;
    bool _initialised;
};
