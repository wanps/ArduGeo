#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

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

    AP_Float _pos_kx_xy;
    AP_Float _pos_kx_z;
    AP_Float _pos_ki_xy;
    AP_Float _pos_ki_z;
    AP_Float _pos_kv_xy;
    AP_Float _pos_kv_z;
    AP_Float _pos_imax_xy;
    AP_Float _pos_imax_z;
    AP_Float _pos_int_c;
    AP_Float _pos_error_filt_hz;
    AP_Float _vel_error_filt_hz;
    AP_Float _omega_c_filt_hz;
    AP_Float _omega_dot_c_filt_hz;

    AP_Float _att_kr_x;
    AP_Float _att_kr_y;
    AP_Float _att_kr_z;
    AP_Float _att_ko_x;
    AP_Float _att_ko_y;
    AP_Float _att_ko_z;
    AP_Float _att_ki_x;
    AP_Float _att_ki_y;
    AP_Float _att_ki_z;
    AP_Float _att_imax_x;
    AP_Float _att_imax_y;
    AP_Float _att_imax_z;
    AP_Float _att_int_c;
    AP_Float _att_j_x;
    AP_Float _att_j_y;
    AP_Float _att_j_z;
    AP_Float _omega_error_filt_hz;

    AP_Float _hover_throttle_norm;
    float _hover_throttle_reference_norm = 0.5f;
    AP_Float _mom_norm_x;
    AP_Float _mom_norm_y;
    AP_Float _mom_norm_z;
    AP_Int8 _output_enabled;

    AP_Int8 _shape_enabled;
    AP_Float _shape_vel_xy_max_ms;
    AP_Float _shape_accel_xy_max_mss;
    AP_Float _shape_vel_up_max_ms;
    AP_Float _shape_vel_down_max_ms;
    AP_Float _shape_accel_z_max_mss;
    AP_Int8 _shape_yaw_enabled;
    AP_Float _shape_yaw_rate_max_rads;
    AP_Float _shape_yaw_accel_max_radss;
};
