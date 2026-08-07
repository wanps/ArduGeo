#include "AC_GeometricControl.h"

#include <AP_HAL/AP_HAL.h>

#define AC_GEOMETRIC_OUTPUT_ENABLED_DEFAULT 1

const AP_Param::GroupInfo AC_GeometricControl::var_info[] = {
    // The empty subgroup prefixes preserve every public GEO_* parameter name
    // while each module owns its metadata, defaults, and AP_Param storage.

    // @Group:
    // @Path: AC_Geometric_Position_PID.cpp
    AP_SUBGROUPINFO(_position_params, "", 1, AC_GeometricControl, AC_Geometric_Position_PID_Params),

    // @Group:
    // @Path: AC_Geometric_Attitude_PID.cpp
    AP_SUBGROUPINFO(_attitude_params, "", 7, AC_GeometricControl, AC_Geometric_Attitude_PID_Params),

    // @Group:
    // @Path: AC_Geometric_OutputMapper.cpp
    AP_SUBGROUPINFO(_output_mapper_params, "", 13, AC_GeometricControl, AC_Geometric_OutputMapper_Params),

    // @Param: OUT_EN
    // @DisplayName: Geometric motor output enable
    // @Description: Enables the geometric controller to write normalized roll, pitch, yaw and throttle outputs to AP_Motors when the vehicle-specific mode also allows it. For Copter, Guided requires GUID_OPTIONS bit 8 and Loiter requires LOIT_OPTIONS bits 1 and 2. Leave disabled until the geometric controller has been validated in simulation for the vehicle and parameter set.
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO("OUT_EN", 17, AC_GeometricControl, _output_enabled, AC_GEOMETRIC_OUTPUT_ENABLED_DEFAULT),

    // @Group:
    // @Path: AC_Geometric_SetpointShaper.cpp
    AP_SUBGROUPINFO(_setpoint_shaper_params, "", 30, AC_GeometricControl, AC_Geometric_SetpointShaper_Params),

    // @Group:
    // @Path: AC_Geometric_YawShaper.cpp
    AP_SUBGROUPINFO(_yaw_shaper_params, "", 36, AC_GeometricControl, AC_Geometric_YawShaper_Params),

    // Keep 45 through 61 unused so the PID-only branch neither interprets
    // SANM storage nor changes LREF_ persistent identity across compatible
    // geometric-controller branch variants.

    // @Group: LREF_
    // @Path: AC_Geometric_LoiterReference.cpp
    AP_SUBGROUPINFO(_loiter_reference_params, "LREF_", 62, AC_GeometricControl, AC_Geometric_LoiterReference_Params),

    AP_GROUPEND
};

AC_GeometricControl::AC_GeometricControl()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AC_GeometricControl::convert_params(uint16_t old_key)
{
    // These module-local tables are frozen to the former flat GEO_ layout.
    // Future module parameters must not be added to the legacy tables.
    _position_params.convert_legacy_params(old_key);
    _attitude_params.convert_legacy_params(old_key);
    _output_mapper_params.convert_legacy_params(old_key);
    _setpoint_shaper_params.convert_legacy_params(old_key);
    _yaw_shaper_params.convert_legacy_params(old_key);
}

void AC_GeometricControl::reset()
{
    _position_pid.reset();
    _attitude_pid.reset();
    _setpoint_shaper.reset();
    _yaw_shaper.reset();
    _output = {};
    _raw_target = {};
    _shaped_target = {};
    _last_update_ms = 0;
    _shaper_active = false;
}

void AC_GeometricControl::set_enabled(bool enabled)
{
    if (_enabled && !enabled) {
        reset();
    }
    _enabled = enabled;
}

void AC_GeometricControl::set_hover_throttle_reference(float hover_throttle_norm)
{
    _hover_throttle_reference_norm = constrain_float(hover_throttle_norm, 0.05f, 0.95f);
}

uint32_t AC_GeometricControl::output_age_ms(uint32_t now_ms) const
{
    if (_last_update_ms == 0) {
        return UINT32_MAX;
    }
    return now_ms - _last_update_ms;
}

bool AC_GeometricControl::output_is_fresh(uint32_t now_ms, uint32_t max_age_ms) const
{
    return output_age_ms(now_ms) <= max_age_ms;
}

void AC_GeometricControl::update_gains_from_params()
{
    // Refreshing AP_Param-backed configuration here keeps QGC/MAVProxy
    // changes live during SITL tuning without requiring a controller restart.
    _position_pid.set_gains(_position_params.gains());
    _position_pid.set_integral_limits(_position_params.integral_limits());
    _position_pid.set_filter_hz(_position_params.filter_hz());

    _attitude_pid.set_gains(_attitude_params.gains());
    _attitude_pid.set_model(_attitude_params.model());
    _attitude_pid.set_integral_limits(_attitude_params.integral_limits());
    _attitude_pid.set_filter_hz(_attitude_params.filter_hz());
}

float AC_GeometricControl::hover_throttle_norm() const
{
    const float hover_throttle_override = _output_mapper_params.hover_throttle_override();
    if (is_positive(hover_throttle_override)) {
        return constrain_float(hover_throttle_override, 0.05f, 0.95f);
    }
    return _hover_throttle_reference_norm;
}

void AC_GeometricControl::update(const AC_Geometric_State& state,
                                 const AC_Geometric_Target& target,
                                 float dt)
{
    if (!_enabled) {
        return;
    }

    update_gains_from_params();

    AC_Geometric_Target position_target = target;
    _raw_target = target;
    _shaper_active = false;
    const bool shaper_enabled = _setpoint_shaper_params.enabled();
    const AC_Geometric_Setpoint_Shaper_Limits shaper_limits = _setpoint_shaper_params.limits();
    if (shaper_enabled && target.build_attitude_from_position && target.shape_position_target) {
        // Guided position targets are often step-like. The geometric shaper
        // converts them into jerk-limited x_d, v_d and a_d references before
        // the Lee/Gao position channel sees them.
        _setpoint_shaper.set_limits(shaper_limits);
        _setpoint_shaper.update(state, target, dt, position_target);
        _shaper_active = true;
    } else {
        // Reset when bypassed so a later shaped segment starts from the
        // current vehicle state instead of an old cached reference.
        _setpoint_shaper.reset();
    }

    if (shaper_enabled && target.build_attitude_from_position && target.shape_yaw_target) {
        const float trajectory_min_speed_ms = MAX(shaper_limits.vel_xy_max_ms * 0.05f, 0.05f);
        _yaw_shaper.set_limits(_yaw_shaper_params.limits(trajectory_min_speed_ms));
        _shaper_active = _yaw_shaper.update(state,
                                            target,
                                            position_target.velocity_ned_ms,
                                            position_target.accel_ned_mss,
                                            dt,
                                            position_target) || _shaper_active;
    } else {
        _yaw_shaper.reset();
    }
    _shaped_target = position_target;

    // The shared translational stage always computes (A,f). With
    // build_attitude_from_position it also derives (R_c,Omega_c,dot(Omega_c));
    // otherwise it passes through the supplied direct attitude reference.
    _position_pid.update(state, position_target, dt, _output.position);

    // The derived or pass-through attitude state becomes R_ref, Omega_ref and
    // dot(Omega_ref) for the SO(3) attitude layer.
    AC_Geometric_Target attitude_target = position_target;
    attitude_target.attitude_body_to_ned = _output.position.attitude_body_to_ned;
    attitude_target.omega_body_rads = _output.position.omega_body_rads;
    attitude_target.omega_dot_body_radss = _output.position.omega_dot_body_radss;

    _attitude_pid.update(state, attitude_target, dt, _output.attitude);

    // The mapper is deliberately last: it translates the geometric force and
    // moment proxies into normalized ArduPilot-facing commands while keeping
    // the physical controller math independent from AP_Motors scaling.
    _output_mapper.update(_output.position,
                          _output.attitude,
                          hover_throttle_norm(),
                          _output_mapper_params.moment_norm(),
                          _output.mapped);
    // This timestamp marks computation freshness only. Vehicle code performs
    // the separate geometric/native ownership decision and motor write.
    _last_update_ms = AP_HAL::millis();
}
