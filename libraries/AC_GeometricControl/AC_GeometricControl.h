#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AC_AttitudeControl/AC_ControlReference.h>

#include "AC_Geometric_Attitude_PID.h"
#include "AC_Geometric_LoiterReference.h"
#include "AC_Geometric_OutputMapper.h"
#include "AC_Geometric_Position_PID.h"
#include "AC_Geometric_SetpointShaper.h"
#include "AC_Geometric_Types.h"
#include "AC_Geometric_YawShaper.h"

// Top-level coordinator for the experimental SE(3) geometric control path.
// Vehicle code owns mode gating, safety checks and motor writes; this class
// only shapes references, runs the geometric control cascade and exposes the
// latest mapped command for the caller to consume.
//
// Shared geometric pipeline:
// (X, X_d) -> (A, R_ref, Omega_ref, dot(Omega_ref), f)
//          -> (e_R, e_Omega, e_I^R, M) -> u_geo.
// R_ref is position-derived R_c in the coupled path or direct R_d in the
// SO(3)-only path.
// Guided and Loiter provide different references but share this cascade.
class AC_GeometricControl {
public:
    AC_GeometricControl();
    CLASS_NO_COPY(AC_GeometricControl);

    static const AP_Param::GroupInfo var_info[];

    // Copy saved values from the pre-module flat GEO_ parameter layout. The
    // conversion is idempotent and never overwrites configured new storage.
    void convert_params(uint16_t old_key);

    // Clear controller integrators and cached outputs.
    void reset();

    // The geometric path is opt-in. Disabling clears output so callers do not
    // accidentally consume stale geometric commands.
    void set_enabled(bool enabled);
    bool enabled() const { return _enabled; }
    bool output_enabled() const { return _output_enabled; }
    uint32_t output_age_ms(uint32_t now_ms) const;
    bool output_is_fresh(uint32_t now_ms, uint32_t max_age_ms) const;
    void set_hover_throttle_reference(float hover_throttle_norm);

    // Run the geometric position-to-attitude and attitude PID cascade.
    // This library only computes outputs; vehicle code decides whether to write motors.
    void update(const AC_Geometric_State& state,
                const AC_Geometric_Target& target,
                float dt);

    static bool reference_to_target(const AC_TrajectoryReference& reference,
                                    AC_Geometric_Target& target);
    static bool reference_to_target(const AC_AttitudeReference& reference,
                                    AC_Geometric_Target& target);
    static bool references_to_target(const AC_TrajectoryReference& trajectory_reference,
                                     const AC_AttitudeReference* attitude_reference,
                                     const AC_GeometricReferencePolicy& policy,
                                     AC_Geometric_Target& target);

    const AC_Geometric_Output& get_output() const { return _output; }
    const AC_Geometric_Target& get_raw_target() const { return _raw_target; }
    const AC_Geometric_Target& get_shaped_target() const { return _shaped_target; }
    bool shaper_active() const { return _shaper_active; }
    AC_Geometric_LoiterReference_Profile get_loiter_reference_profile() const { return _loiter_reference_params.get(); }

private:
    void update_gains_from_params();
    float hover_throttle_norm() const;

    // Controls whether the cascade computes. GEO_OUT_EN is a separate output
    // permission, and vehicle-level mode/safety checks still decide ownership.
    bool _enabled = false;

    // The cascade is kept as separate components so future controllers can
    // replace one layer, for example an alternative attitude block, without
    // changing the data contract between layers.
    AC_Geometric_Position_PID_Params _position_params;
    AC_Geometric_Attitude_PID_Params _attitude_params;
    AC_Geometric_OutputMapper_Params _output_mapper_params;
    AC_Geometric_SetpointShaper_Params _setpoint_shaper_params;
    AC_Geometric_YawShaper_Params _yaw_shaper_params;
    AC_Geometric_Position_PID _position_pid;
    AC_Geometric_Attitude_PID _attitude_pid;
    AC_Geometric_OutputMapper _output_mapper;
    AC_Geometric_LoiterReference_Params _loiter_reference_params;
    AC_Geometric_SetpointShaper _setpoint_shaper;
    AC_Geometric_YawShaper _yaw_shaper;
    AC_Geometric_Output _output;
    AC_Geometric_Target _raw_target;
    AC_Geometric_Target _shaped_target;
    // Time of the latest computed mapped intent. Freshness alone does not
    // prove that the vehicle-level arbiter selected or wrote it on this frame.
    uint32_t _last_update_ms = 0;
    bool _shaper_active = false;

    float _hover_throttle_reference_norm = 0.5f;
    AP_Int8 _output_enabled;
};
