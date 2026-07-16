#include "AC_Geometric_LoiterReference.h"

#include <AP_Math/control.h>

namespace {

constexpr float reference_epsilon = 1.0e-4f;
constexpr float reference_max_dt_s = 0.1f;

constexpr float reference_speed_xy_default_ms = 2.0f;
constexpr float reference_accel_xy_default_mss = 1.0f;
constexpr float reference_jerk_xy_default_msss = 2.0f;
constexpr float reference_brake_delay_default_s = 0.2f;
constexpr float reference_brake_accel_default_mss = 2.5f;
constexpr float reference_brake_jerk_default_msss = 5.0f;
constexpr float reference_jerk_z_default_msss = 5.0f;
constexpr float reference_brake_accel_z_default_mss = 2.5f;
constexpr float reference_brake_jerk_z_default_msss = 5.0f;
constexpr float reference_yaw_accel_default_radss = 1.0f;
constexpr float reference_yaw_jerk_default_radsss = 2.0f;
constexpr float reference_yaw_brake_accel_default_radss = 2.0f;
constexpr float reference_yaw_brake_jerk_default_radsss = 6.0f;

bool vector_is_finite(const Vector2f& value)
{
    return !value.is_nan() && !value.is_inf();
}

bool vector_is_finite(const Vector3f& value)
{
    return !value.is_nan() && !value.is_inf();
}

// Velocity removed after the current frame if a negative acceleration along
// the direction of travel is ramped toward zero by +jerk on every subsequent
// semi-implicit integration step.  This is the discrete counterpart of
// a^2/(2*j) and lets the vertical brake choose a jerk-bounded acceleration
// that reaches velocity=0 and acceleration=0 on the same frame.
float vertical_brake_release_velocity_ms(float accel_along_velocity_mss,
                                         float jerk_max_msss,
                                         float dt_s)
{
    if (!is_negative(accel_along_velocity_mss)) {
        return 0.0f;
    }
    const float accel_step_mss = jerk_max_msss * dt_s;
    const float accel_magnitude_mss = -accel_along_velocity_mss;
    const float negative_step_count =
        MAX(ceilf(accel_magnitude_mss / accel_step_mss) - 1.0f, 0.0f);
    return dt_s *
           (negative_step_count * accel_magnitude_mss -
            0.5f * accel_step_mss * negative_step_count *
                (negative_step_count + 1.0f));
}

}

const AP_Param::GroupInfo AC_Geometric_LoiterReference_Params::var_info[] = {
    // @Param: VXY
    // @DisplayName: Geometric Loiter reference horizontal speed
    // @Description: Maximum horizontal speed of the controller-independent geometric Loiter reference. The lowest of this value, LOIT_SPEED_MS and the active EKF navigation speed limit is used.
    // @Units: m/s
    // @Range: 0.2 20
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("VXY", 1, AC_Geometric_LoiterReference_Params, _speed_xy_max_ms, reference_speed_xy_default_ms),

    // @Param: AXY
    // @DisplayName: Geometric Loiter reference pilot acceleration
    // @Description: Maximum pilot and drag acceleration used by the geometric Loiter reference. Braking uses the separate GEO_LREF_BACC limit and remains bounded by the physical Loiter lean limit.
    // @Units: m/s/s
    // @Range: 0.1 9.81
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("AXY", 2, AC_Geometric_LoiterReference_Params, _accel_xy_max_mss, reference_accel_xy_default_mss),

    // @Param: JXY
    // @DisplayName: Geometric Loiter reference pilot jerk
    // @Description: Horizontal jerk limit used while shaping pilot acceleration. Braking uses the separate GEO_LREF_BJRK limit.
    // @Units: m/s/s/s
    // @Range: 0.1 50
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("JXY", 3, AC_Geometric_LoiterReference_Params, _jerk_xy_max_msss, reference_jerk_xy_default_msss),

    // @Param: JZ
    // @DisplayName: Geometric Loiter reference vertical jerk
    // @Description: Vertical jerk limit for pilot climb, descent and takeoff reference generation.
    // @Units: m/s/s/s
    // @Range: 0.1 50
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("JZ", 4, AC_Geometric_LoiterReference_Params, _jerk_z_max_msss, reference_jerk_z_default_msss),

    // @Param: YACC
    // @DisplayName: Geometric Loiter reference yaw acceleration
    // @Description: Maximum yaw acceleration used by the controller-independent Loiter yaw reference.
    // @Units: rad/s/s
    // @Range: 0.1 20
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("YACC", 5, AC_Geometric_LoiterReference_Params, _yaw_accel_max_radss, reference_yaw_accel_default_radss),

    // @Param: YJRK
    // @DisplayName: Geometric Loiter reference yaw jerk
    // @Description: Maximum yaw jerk used by the controller-independent Loiter yaw reference.
    // @Units: rad/s/s/s
    // @Range: 0.1 50
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("YJRK", 6, AC_Geometric_LoiterReference_Params, _yaw_jerk_max_radsss, reference_yaw_jerk_default_radsss),

    // @Param: BDLY
    // @DisplayName: Geometric Loiter horizontal brake delay
    // @Description: Delay after the pilot releases the horizontal stick before dedicated geometric Loiter braking starts.
    // @Units: s
    // @Range: 0 2
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("BDLY", 7, AC_Geometric_LoiterReference_Params, _brake_delay_s, reference_brake_delay_default_s),

    // @Param: BACC
    // @DisplayName: Geometric Loiter horizontal brake acceleration
    // @Description: Maximum extra horizontal brake-acceleration component after stick release. This component is added to the speed-dependent GEO_LREF_AXY drag and the total remains bounded by the physical Loiter lean limit.
    // @Units: m/s/s
    // @Range: 0.1 9.81
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("BACC", 8, AC_Geometric_LoiterReference_Params, _brake_accel_max_mss, reference_brake_accel_default_mss),

    // @Param: BJRK
    // @DisplayName: Geometric Loiter horizontal brake jerk
    // @Description: Maximum jerk used by the dedicated geometric Loiter horizontal brake.
    // @Units: m/s/s/s
    // @Range: 0.1 50
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("BJRK", 9, AC_Geometric_LoiterReference_Params, _brake_jerk_max_msss, reference_brake_jerk_default_msss),

    // @Param: YBACC
    // @DisplayName: Geometric Loiter yaw brake acceleration
    // @Description: Maximum yaw deceleration used after the pilot releases the yaw stick.
    // @Units: rad/s/s
    // @Range: 0.1 20
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("YBACC", 10, AC_Geometric_LoiterReference_Params, _yaw_brake_accel_max_radss, reference_yaw_brake_accel_default_radss),

    // @Param: YBJRK
    // @DisplayName: Geometric Loiter yaw brake jerk
    // @Description: Maximum jerk used by the dedicated geometric Loiter yaw brake after stick release.
    // @Units: rad/s/s/s
    // @Range: 0.1 50
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("YBJRK", 11, AC_Geometric_LoiterReference_Params, _yaw_brake_jerk_max_radsss, reference_yaw_brake_jerk_default_radsss),

    // @Param: ZBACC
    // @DisplayName: Geometric Loiter vertical brake acceleration
    // @Description: Maximum vertical deceleration used after the pilot releases or reverses the throttle stick. The effective value is also bounded by PILOT_ACC_Z.
    // @Units: m/s/s
    // @Range: 0.1 9.81
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("ZBACC", 12, AC_Geometric_LoiterReference_Params, _brake_accel_z_max_mss, reference_brake_accel_z_default_mss),

    // @Param: ZBJRK
    // @DisplayName: Geometric Loiter vertical brake jerk
    // @Description: Maximum jerk used by the dedicated geometric Loiter vertical brake after throttle-stick release or reversal.
    // @Units: m/s/s/s
    // @Range: 0.1 50
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("ZBJRK", 13, AC_Geometric_LoiterReference_Params, _brake_jerk_z_max_msss, reference_brake_jerk_z_default_msss),

    AP_GROUPEND
};

AC_Geometric_LoiterReference_Params::AC_Geometric_LoiterReference_Params()
{
    AP_Param::setup_object_defaults(this, var_info);
}

AC_Geometric_LoiterReference_Profile AC_Geometric_LoiterReference_Params::get() const
{
    AC_Geometric_LoiterReference_Profile profile;
    profile.speed_xy_max_ms = _speed_xy_max_ms.get();
    profile.accel_xy_max_mss = _accel_xy_max_mss.get();
    profile.jerk_xy_max_msss = _jerk_xy_max_msss.get();
    profile.brake_delay_s = _brake_delay_s.get();
    profile.brake_accel_max_mss = _brake_accel_max_mss.get();
    profile.brake_jerk_max_msss = _brake_jerk_max_msss.get();
    profile.jerk_z_max_msss = _jerk_z_max_msss.get();
    profile.brake_accel_z_max_mss = _brake_accel_z_max_mss.get();
    profile.brake_jerk_z_max_msss = _brake_jerk_z_max_msss.get();
    profile.yaw_accel_max_radss = _yaw_accel_max_radss.get();
    profile.yaw_jerk_max_radsss = _yaw_jerk_max_radsss.get();
    profile.yaw_brake_accel_max_radss = _yaw_brake_accel_max_radss.get();
    profile.yaw_brake_jerk_max_radsss = _yaw_brake_jerk_max_radsss.get();
    return profile;
}

void AC_Geometric_LoiterReference::reset()
{
    _position_ref_ned_m.zero();
    _velocity_ref_ned_ms.zero();
    _accel_ref_ned_mss.zero();
    _accel_shaper_ned_mss.zero();
    _yaw_ref_rad = 0.0f;
    _yaw_rate_ref_rads = 0.0f;
    _yaw_accel_ref_radss = 0.0f;
    _yaw_accel_shaper_radss = 0.0f;
    _neutral_xy_time_s = 0.0f;
    _brake_accel_mss = 0.0f;
    _z_brake_settled = false;
    _constraint_base_position_ned_m.zero();
    _constraint_base_velocity_ned_ms.zero();
    _last_limits = {};
    _last_update_dt_s = 0.0f;
    _velocity_constraint_pending = false;
    _initialized = false;
}

bool AC_Geometric_LoiterReference::reset(const AC_Geometric_State& state,
                                         float yaw_rad,
                                         float yaw_rate_rads)
{
    reset();
    if (!vector_is_finite(state.position_ned_m) ||
        !vector_is_finite(state.velocity_ned_ms) ||
        !isfinite(yaw_rad) ||
        !isfinite(yaw_rate_rads)) {
        return false;
    }

    _position_ref_ned_m = state.position_ned_m.topostype();
    _velocity_ref_ned_ms = state.velocity_ned_ms;
    _yaw_ref_rad = wrap_PI(yaw_rad);
    _yaw_rate_ref_rads = yaw_rate_rads;
    _initialized = true;
    return true;
}

bool AC_Geometric_LoiterReference::limits_valid(const AC_Geometric_LoiterReference_Limits& limits) const
{
    const float values[] {
        limits.speed_xy_max_ms,
        limits.accel_xy_max_mss,
        limits.accel_xy_total_max_mss,
        limits.jerk_xy_max_msss,
        limits.brake_delay_s,
        limits.brake_accel_max_mss,
        limits.brake_jerk_max_msss,
        limits.speed_up_max_ms,
        limits.speed_down_max_ms,
        limits.accel_z_max_mss,
        limits.jerk_z_max_msss,
        limits.brake_accel_z_max_mss,
        limits.brake_jerk_z_max_msss,
        limits.yaw_rate_max_rads,
        limits.yaw_accel_max_radss,
        limits.yaw_jerk_max_radsss,
        limits.yaw_brake_accel_max_radss,
        limits.yaw_brake_jerk_max_radsss,
    };
    for (const float value : values) {
        if (!isfinite(value)) {
            return false;
        }
    }

    return is_positive(limits.speed_xy_max_ms) &&
           is_positive(limits.accel_xy_max_mss) &&
           limits.accel_xy_total_max_mss >= limits.accel_xy_max_mss &&
           is_positive(limits.jerk_xy_max_msss) &&
           !is_negative(limits.brake_delay_s) &&
           is_positive(limits.brake_accel_max_mss) &&
           is_positive(limits.brake_jerk_max_msss) &&
           is_positive(limits.speed_up_max_ms) &&
           is_positive(limits.speed_down_max_ms) &&
           is_positive(limits.accel_z_max_mss) &&
           is_positive(limits.jerk_z_max_msss) &&
           is_positive(limits.brake_accel_z_max_mss) &&
           is_positive(limits.brake_jerk_z_max_msss) &&
           is_positive(limits.yaw_rate_max_rads) &&
           is_positive(limits.yaw_accel_max_radss) &&
           is_positive(limits.yaw_jerk_max_radsss) &&
           is_positive(limits.yaw_brake_accel_max_radss) &&
           is_positive(limits.yaw_brake_jerk_max_radsss);
}

void AC_Geometric_LoiterReference::write_target(AC_Geometric_Target& target) const
{
    target = {};
    target.position_ned_m = _position_ref_ned_m.tofloat();
    target.velocity_ned_ms = _velocity_ref_ned_ms;
    target.accel_ned_mss = _accel_ref_ned_mss;
    target.build_attitude_from_position = true;
    target.shape_position_target = false;
    target.shape_yaw_target = false;
    target.yaw_from_trajectory = false;
    target.yaw_rad = _yaw_ref_rad;
    target.yaw_rate_rads = _yaw_rate_ref_rads;
    target.omega_body_rads.z = _yaw_rate_ref_rads;
    target.omega_dot_body_radss.z = _yaw_accel_ref_radss;
}

bool AC_Geometric_LoiterReference::update(const AC_Geometric_LoiterReference_Input& input,
                                          const AC_Geometric_LoiterReference_Limits& limits,
                                          float dt_s,
                                          AC_Geometric_Target& target,
                                          AC_Geometric_LoiterReference_Status& status)
{
    status = {};
    if (!_initialized ||
        !is_positive(dt_s) ||
        dt_s > reference_max_dt_s ||
        !limits_valid(limits) ||
        !vector_is_finite(input.pilot_accel_ne_mss) ||
        !isfinite(input.climb_rate_up_ms) ||
        !isfinite(input.position_z_target_ned_m) ||
        !isfinite(input.yaw_rate_rads)) {
        return false;
    }

    _constraint_base_position_ned_m = _position_ref_ned_m;
    _constraint_base_velocity_ned_ms = _velocity_ref_ned_ms;
    _last_limits = limits;
    _last_update_dt_s = dt_s;
    _velocity_constraint_pending = false;

    // Horizontal pilot trajectory. Drag makes full-stick acceleration settle
    // at the configured speed; delayed braking acts only after stick release.
    const Vector2f velocity_xy_old = _velocity_ref_ned_ms.xy();
    const float speed_xy_ms = velocity_xy_old.length();

    Vector2f pilot_accel_ne_mss = input.pilot_accel_ne_mss;
    bool horizontal_command_active = input.pilot_xy_active;
    if (input.coordinated_turn) {
        const Vector2f turn_accel_ne_mss{-velocity_xy_old.y * _yaw_rate_ref_rads,
                                         velocity_xy_old.x * _yaw_rate_ref_rads};
        pilot_accel_ne_mss += turn_accel_ne_mss;
        // A residual shaped yaw rate after raw-stick release may still add a
        // coordinated-turn feedforward, but it must not restart the neutral
        // timer.  Only an active raw yaw command can defer XY braking.
        horizontal_command_active |= input.pilot_yaw_active &&
                                     turn_accel_ne_mss.length_squared() > sq(reference_epsilon);
    }
    pilot_accel_ne_mss.limit_length(limits.accel_xy_max_mss);

    if (horizontal_command_active) {
        _neutral_xy_time_s = 0.0f;
    } else {
        _neutral_xy_time_s = MIN(_neutral_xy_time_s + dt_s, limits.brake_delay_s + 10.0f);
    }

    float brake_accel_target_mss = 0.0f;
    if (!horizontal_command_active &&
        _neutral_xy_time_s >= limits.brake_delay_s &&
        speed_xy_ms > reference_epsilon) {
        const float brake_gain = limits.brake_jerk_max_msss / limits.brake_accel_max_mss;
        brake_accel_target_mss = constrain_float(sqrt_controller(speed_xy_ms,
                                                                 brake_gain,
                                                                 limits.brake_jerk_max_msss,
                                                                 dt_s),
                                                  0.0f,
                                                  limits.brake_accel_max_mss);
    }
    shape_accel(brake_accel_target_mss, _brake_accel_mss, limits.brake_jerk_max_msss, dt_s);

    Vector2f accel_xy_target_mss = pilot_accel_ne_mss;
    const bool speed_xy_overspeed = speed_xy_ms > limits.speed_xy_max_ms + reference_epsilon;
    if (speed_xy_ms > reference_epsilon) {
        const Vector2f velocity_unit = velocity_xy_old / speed_xy_ms;
        const float drag_accel_mss = constrain_float(limits.accel_xy_max_mss * speed_xy_ms / limits.speed_xy_max_ms,
                                                     0.0f,
                                                     limits.accel_xy_max_mss);
        accel_xy_target_mss -= velocity_unit * (drag_accel_mss + _brake_accel_mss);

        // A reference may start above the configured speed after an airborne
        // entry or a runtime limit reduction.  At full stick in the direction
        // of travel, pilot acceleration and saturated drag otherwise cancel
        // exactly and leave the reference permanently overspeed.  Add a
        // jerk-shaped radial recovery term while preserving continuity from
        // the measured reset velocity.
        if (speed_xy_overspeed) {
            const float overspeed_error_ms = speed_xy_ms - limits.speed_xy_max_ms;
            const float overspeed_gain = limits.jerk_xy_max_msss / limits.accel_xy_max_mss;
            const float overspeed_accel_mss = constrain_float(sqrt_controller(overspeed_error_ms,
                                                                              overspeed_gain,
                                                                              limits.jerk_xy_max_msss,
                                                                              dt_s),
                                                               0.0f,
                                                               limits.accel_xy_max_mss);
            accel_xy_target_mss -= velocity_unit * overspeed_accel_mss;
        }
    }
    const bool brake_shaping_active = is_positive(brake_accel_target_mss) ||
                                      _brake_accel_mss > reference_epsilon;
    const float accel_xy_total_limit_mss = brake_shaping_active ?
                                           limits.accel_xy_total_max_mss :
                                           limits.accel_xy_max_mss;
    accel_xy_target_mss.limit_length(accel_xy_total_limit_mss);

    Vector2f accel_xy_shaped_mss = _accel_shaper_ned_mss.xy();
    const float jerk_xy_limit_msss = brake_shaping_active ?
                                      MAX(limits.jerk_xy_max_msss, limits.brake_jerk_max_msss) :
                                      limits.jerk_xy_max_msss;
    shape_accel_xy(accel_xy_target_mss, accel_xy_shaped_mss, jerk_xy_limit_msss, dt_s);
    accel_xy_shaped_mss.limit_length(limits.accel_xy_total_max_mss);

    Vector2f velocity_xy_next = velocity_xy_old + accel_xy_shaped_mss * dt_s;
    const float speed_allowed_ms = MAX(limits.speed_xy_max_ms, speed_xy_ms);
    if (velocity_xy_next.length() > speed_allowed_ms) {
        velocity_xy_next.limit_length(speed_allowed_ms);
        status.speed_xy_limited = true;
    }
    status.speed_xy_limited |= speed_xy_overspeed;
    bool overspeed_settled = false;
    if (speed_xy_overspeed && velocity_xy_next.length() < limits.speed_xy_max_ms) {
        velocity_xy_next = velocity_xy_old.normalized() * limits.speed_xy_max_ms;
        overspeed_settled = true;
    }
    bool xy_settled = false;
    if (!horizontal_command_active && speed_xy_ms <= reference_epsilon) {
        // A sub-epsilon residual would otherwise persist forever because the
        // drag and brake direction are intentionally undefined at zero speed.
        velocity_xy_next.zero();
        xy_settled = true;
    } else if (!horizontal_command_active &&
               velocity_xy_next.dot(velocity_xy_old) <= 0.0f) {
        velocity_xy_next.zero();
        xy_settled = true;
    }
    const Vector2f accel_xy_applied_mss = (velocity_xy_next - velocity_xy_old) / dt_s;

    Vector2p position_xy_next = _position_ref_ned_m.xy();
    Vector2f velocity_xy_integrated = velocity_xy_old;
    const Vector2f zero_xy;
    update_pos_vel_accel_xy(position_xy_next,
                            velocity_xy_integrated,
                            accel_xy_applied_mss,
                            dt_s,
                            zero_xy,
                            zero_xy,
                            zero_xy);
    _position_ref_ned_m.x = position_xy_next.x;
    _position_ref_ned_m.y = position_xy_next.y;
    _velocity_ref_ned_ms.x = velocity_xy_integrated.x;
    _velocity_ref_ned_ms.y = velocity_xy_integrated.y;
    _accel_ref_ned_mss.x = accel_xy_applied_mss.x;
    _accel_ref_ned_mss.y = accel_xy_applied_mss.y;
    if (xy_settled || overspeed_settled) {
        _accel_shaper_ned_mss.x = 0.0f;
        _accel_shaper_ned_mss.y = 0.0f;
        if (xy_settled) {
            _brake_accel_mss = 0.0f;
        }
    } else {
        _accel_shaper_ned_mss.x = accel_xy_shaped_mss.x;
        _accel_shaper_ned_mss.y = accel_xy_shaped_mss.y;
    }
    status.braking = _brake_accel_mss > reference_epsilon;
    status.xy_settled = xy_settled;

    // Vertical pilot trajectory. A takeoff target adds a position boundary;
    // normal flying follows the Up-positive climb-rate input directly. Stick
    // release and reversal first brake the existing NED velocity to zero with
    // a dedicated, zero-delay profile. Unlike yaw, residual vertical
    // acceleration is not projected away: preserving jerk continuity avoids
    // a collective-thrust step.
    const float velocity_z_command_ms = constrain_float(-input.climb_rate_up_ms,
                                                        -limits.speed_up_max_ms,
                                                        limits.speed_down_max_ms);
    const float velocity_z_old_ms = _velocity_ref_ned_ms.z;
    float accel_z_shaped_mss = _accel_shaper_ned_mss.z;
    bool vertical_brake_active = false;
    bool vertical_terminal_stop_bounded = false;
    if (input.use_z_position_target) {
        _z_brake_settled = false;
        // An absolute takeoff boundary must converge to zero velocity at the
        // boundary.  Feeding the pilot climb rate into shape_pos_vel_accel()
        // would make it a moving-reference feedforward and carry the target
        // past the requested altitude.
        shape_pos_vel_accel(postype_t(input.position_z_target_ned_m),
                            0.0f,
                            0.0f,
                            _position_ref_ned_m.z,
                            velocity_z_old_ms,
                            accel_z_shaped_mss,
                            -limits.speed_up_max_ms,
                            limits.speed_down_max_ms,
                            -limits.accel_z_max_mss,
                            limits.accel_z_max_mss,
                            limits.jerk_z_max_msss,
                            dt_s,
                            true);
    } else {
        const bool vertical_command_reversing =
            input.pilot_z_active &&
            fabsf(velocity_z_command_ms) > reference_epsilon &&
            velocity_z_command_ms * velocity_z_old_ms < 0.0f;
        const bool vertical_brake_requested = !input.pilot_z_active ||
                                              vertical_command_reversing;
        if (!vertical_brake_requested) {
            _z_brake_settled = false;
        }
        if (vertical_brake_requested) {
            vertical_brake_active = true;
            const float accel_step_mss = limits.brake_jerk_z_max_msss * dt_s;
            const float accel_stop_mss = -velocity_z_old_ms / dt_s;
            // A terminal zero-velocity clamp is allowed only if its applied
            // acceleration is reachable from the inherited shaper state in
            // one jerk step, can return to zero in the following jerk step,
            // and remains inside the configured brake-acceleration envelope.
            // This matters after an external velocity constraint rebases the
            // reference to a small velocity with a comparatively large
            // acceleration.
            vertical_terminal_stop_bounded =
                fabsf(accel_stop_mss - accel_z_shaped_mss) <=
                    accel_step_mss + reference_epsilon &&
                fabsf(accel_stop_mss) <= accel_step_mss + reference_epsilon &&
                fabsf(accel_stop_mss) <=
                    limits.brake_accel_z_max_mss + reference_epsilon;
            if (fabsf(velocity_z_old_ms) <= reference_epsilon &&
                vertical_terminal_stop_bounded) {
                // Close the finite-time brake explicitly at the numerical
                // zero-speed boundary.  The switching curve has already
                // reduced acceleration to within one jerk step here, so this
                // removes only the final sub-epsilon velocity without a
                // collective-thrust discontinuity.  For an active reversal,
                // this also provides an observable one-frame stop before the
                // opposite command starts on the following frame.
                accel_z_shaped_mss = accel_stop_mss;
            } else {
                // Finite-time discrete S-curve. Choose the next acceleration
                // so the remaining velocity equals exactly what a max-jerk
                // release of that acceleration will remove on later
                // integration frames. When that boundary is outside this
                // frame's jerk interval, use the closest interval endpoint.
                // This produces accel-limited trapezoidal braking at high
                // speed and a triangular profile near zero, without an
                // exponential tail or terminal collective step.
                float direction = 1.0f;
                if (!is_zero(velocity_z_old_ms)) {
                    direction = velocity_z_old_ms > 0.0f ? 1.0f : -1.0f;
                } else if (!is_zero(accel_z_shaped_mss)) {
                    // A zero-velocity state can retain acceleration after an
                    // external constraint. Unwind it in the direction in
                    // which it would physically start moving.
                    direction = accel_z_shaped_mss > 0.0f ? 1.0f : -1.0f;
                }
                const float velocity_along_direction_ms =
                    MAX(direction * velocity_z_old_ms, 0.0f);
                const float accel_along_direction_mss =
                    direction * accel_z_shaped_mss;
                // A newly selected brake limit may be below the acceleration
                // inherited from the pilot trajectory. Preserve jerk
                // continuity while returning into the configured envelope.
                const float dynamic_accel_limit_mss =
                    MAX(limits.brake_accel_z_max_mss,
                        fabsf(accel_along_direction_mss));
                float accel_next_low_mss =
                    MAX(-dynamic_accel_limit_mss,
                        accel_along_direction_mss - accel_step_mss);
                float accel_next_high_mss =
                    MIN(dynamic_accel_limit_mss,
                        accel_along_direction_mss + accel_step_mss);

                const auto stop_residual_ms = [&](float accel_next_mss) {
                    const float velocity_next_ms =
                        velocity_along_direction_ms + accel_next_mss * dt_s;
                    return velocity_next_ms -
                           vertical_brake_release_velocity_ms(
                               accel_next_mss,
                               limits.brake_jerk_z_max_msss,
                               dt_s);
                };

                if (stop_residual_ms(accel_next_low_mss) > 0.0f) {
                    accel_z_shaped_mss = direction * accel_next_low_mss;
                } else if (stop_residual_ms(accel_next_high_mss) < 0.0f) {
                    accel_z_shaped_mss = direction * accel_next_high_mss;
                } else {
                    // The exact switching acceleration is inside the
                    // permitted jerk interval. A short fixed-iteration
                    // bisection is deterministic and avoids a discontinuous
                    // closed-form branch at integer acceleration-step
                    // boundaries.
                    for (uint8_t i = 0; i < 18; i++) {
                        const float accel_next_mid_mss =
                            0.5f * (accel_next_low_mss +
                                    accel_next_high_mss);
                        if (stop_residual_ms(accel_next_mid_mss) > 0.0f) {
                            accel_next_high_mss = accel_next_mid_mss;
                        } else {
                            accel_next_low_mss = accel_next_mid_mss;
                        }
                    }
                    accel_z_shaped_mss = direction *
                        (0.5f * (accel_next_low_mss +
                                 accel_next_high_mss));
                }
            }
        } else {
            shape_vel_accel(velocity_z_command_ms,
                            0.0f,
                            velocity_z_old_ms,
                            accel_z_shaped_mss,
                            -limits.accel_z_max_mss,
                            limits.accel_z_max_mss,
                            limits.jerk_z_max_msss,
                            dt_s,
                            true);
        }
    }

    float velocity_z_next_ms = velocity_z_old_ms + accel_z_shaped_mss * dt_s;
    const float velocity_z_min_ms = MIN(-limits.speed_up_max_ms, velocity_z_old_ms);
    const float velocity_z_max_ms = MAX(limits.speed_down_max_ms, velocity_z_old_ms);
    const float velocity_z_limited_ms = constrain_float(velocity_z_next_ms,
                                                        velocity_z_min_ms,
                                                        velocity_z_max_ms);
    if (!is_equal(velocity_z_limited_ms, velocity_z_next_ms)) {
        velocity_z_next_ms = velocity_z_limited_ms;
        status.speed_z_limited = true;
    }
    bool z_hard_stopped = false;
    if (vertical_brake_active && vertical_terminal_stop_bounded) {
        // Clamp only when both the incoming and following acceleration changes
        // remain inside the configured jerk and acceleration bounds. If an
        // external constraint supplied an inconsistent near-zero PVA state,
        // preserve acceleration continuity and let the S-curve dissipate it
        // instead of synthesising a collective-thrust step.
        if (velocity_z_next_ms * velocity_z_old_ms <= 0.0f) {
            velocity_z_next_ms = 0.0f;
            z_hard_stopped = true;
            _z_brake_settled = true;
        }
    }
    const float accel_z_applied_mss = (velocity_z_next_ms - velocity_z_old_ms) / dt_s;

    postype_t position_z_next_m = _position_ref_ned_m.z;
    float velocity_z_integrated_ms = velocity_z_old_ms;
    update_pos_vel_accel(position_z_next_m,
                         velocity_z_integrated_ms,
                         accel_z_applied_mss,
                         dt_s,
                         0.0f,
                         0.0f,
                         0.0f);
    _position_ref_ned_m.z = position_z_next_m;
    _velocity_ref_ned_ms.z = velocity_z_integrated_ms;
    _accel_ref_ned_mss.z = accel_z_applied_mss;
    _accel_shaper_ned_mss.z = (status.speed_z_limited || z_hard_stopped) ?
                                0.0f : accel_z_shaped_mss;
    status.z_braking = vertical_brake_active && !z_hard_stopped;
    status.z_settled = _z_brake_settled;

    // Independent pilot-yaw trajectory.  Raw stick release switches to a
    // dedicated brake profile.  Any acceleration still increasing the
    // residual rate is projected to zero before braking, so release cannot
    // produce the former extra yaw-rate rise while the jerk shaper reverses.
    const float yaw_rate_command_rads = constrain_float(input.yaw_rate_rads,
                                                        -limits.yaw_rate_max_rads,
                                                        limits.yaw_rate_max_rads);
    const float yaw_rate_old_rads = _yaw_rate_ref_rads;
    if (!input.pilot_yaw_active &&
        _yaw_accel_shaper_radss * yaw_rate_old_rads > 0.0f) {
        _yaw_accel_shaper_radss = 0.0f;
    }
    const float yaw_rate_target_rads = input.pilot_yaw_active ? yaw_rate_command_rads : 0.0f;
    const float yaw_accel_limit_radss = input.pilot_yaw_active ?
                                        limits.yaw_accel_max_radss :
                                        limits.yaw_brake_accel_max_radss;
    const float yaw_jerk_limit_radsss = input.pilot_yaw_active ?
                                        limits.yaw_jerk_max_radsss :
                                        limits.yaw_brake_jerk_max_radsss;
    shape_vel_accel(yaw_rate_target_rads,
                    0.0f,
                    yaw_rate_old_rads,
                    _yaw_accel_shaper_radss,
                    -yaw_accel_limit_radss,
                    yaw_accel_limit_radss,
                    yaw_jerk_limit_radsss,
                    dt_s,
                    true);
    float yaw_rate_next_rads = yaw_rate_old_rads + _yaw_accel_shaper_radss * dt_s;
    bool yaw_settled = false;
    if (!input.pilot_yaw_active) {
        // Enforce a one-way brake boundary even around zero where the generic
        // velocity shaper may cross by one integration step.
        if (fabsf(yaw_rate_old_rads) <= reference_epsilon ||
            yaw_rate_next_rads * yaw_rate_old_rads <= 0.0f) {
            yaw_rate_next_rads = 0.0f;
            yaw_settled = true;
        } else if (yaw_rate_old_rads > 0.0f) {
            yaw_rate_next_rads = constrain_float(yaw_rate_next_rads, 0.0f, yaw_rate_old_rads);
        } else {
            yaw_rate_next_rads = constrain_float(yaw_rate_next_rads, yaw_rate_old_rads, 0.0f);
        }
    }
    // Preserve an initially out-of-range rate and shape it back continuously.
    // Clamping immediately to the configured limit would synthesize an
    // unbounded acceleration on the first frame.  The dynamic boundary only
    // prevents motion farther outside the envelope.
    const bool yaw_rate_overspeed = fabsf(yaw_rate_old_rads) > limits.yaw_rate_max_rads + reference_epsilon;
    const float yaw_rate_dynamic_limit_rads = MAX(limits.yaw_rate_max_rads, fabsf(yaw_rate_old_rads));
    const float yaw_rate_limited_rads = constrain_float(yaw_rate_next_rads,
                                                        -yaw_rate_dynamic_limit_rads,
                                                        yaw_rate_dynamic_limit_rads);
    bool yaw_rate_hard_limited = false;
    if (!is_equal(yaw_rate_limited_rads, yaw_rate_next_rads)) {
        yaw_rate_next_rads = yaw_rate_limited_rads;
        yaw_rate_hard_limited = true;
    }
    status.yaw_rate_limited = yaw_rate_overspeed ||
                              fabsf(yaw_rate_next_rads) > limits.yaw_rate_max_rads + reference_epsilon ||
                              yaw_rate_hard_limited;
    _yaw_accel_ref_radss = (yaw_rate_next_rads - yaw_rate_old_rads) / dt_s;
    if (yaw_rate_hard_limited || yaw_settled) {
        _yaw_accel_shaper_radss = 0.0f;
    }
    _yaw_ref_rad = wrap_PI(_yaw_ref_rad + yaw_rate_old_rads * dt_s +
                          0.5f * _yaw_accel_ref_radss * sq(dt_s));
    _yaw_rate_ref_rads = yaw_rate_next_rads;
    status.yaw_braking = !input.pilot_yaw_active &&
                         fabsf(yaw_rate_old_rads) > reference_epsilon &&
                         !yaw_settled;
    status.yaw_settled = yaw_settled;

    write_target(target);
    _velocity_constraint_pending = true;
    return true;
}

bool AC_Geometric_LoiterReference::apply_velocity_constraint(
    const Vector3f& velocity_ned_ms,
    AC_Geometric_Target& target,
    AC_Geometric_LoiterReference_Status& status)
{
    if (!_initialized ||
        !_velocity_constraint_pending ||
        !vector_is_finite(velocity_ned_ms) ||
        !is_positive(_last_update_dt_s) ||
        !limits_valid(_last_limits)) {
        return false;
    }
    _velocity_constraint_pending = false;

    Vector3f constrained_velocity_ned_ms = velocity_ned_ms;
    constrained_velocity_ned_ms.xy().limit_length(_last_limits.speed_xy_max_ms);
    constrained_velocity_ned_ms.z = constrain_float(constrained_velocity_ned_ms.z,
                                                    -_last_limits.speed_up_max_ms,
                                                    _last_limits.speed_down_max_ms);

    Vector3f velocity_delta_ned_ms = constrained_velocity_ned_ms - _constraint_base_velocity_ned_ms;
    Vector2f velocity_delta_xy_ms = velocity_delta_ned_ms.xy();
    velocity_delta_xy_ms.limit_length(_last_limits.accel_xy_total_max_mss * _last_update_dt_s);
    velocity_delta_ned_ms.x = velocity_delta_xy_ms.x;
    velocity_delta_ned_ms.y = velocity_delta_xy_ms.y;
    velocity_delta_ned_ms.z = constrain_float(velocity_delta_ned_ms.z,
                                              -_last_limits.accel_z_max_mss * _last_update_dt_s,
                                              _last_limits.accel_z_max_mss * _last_update_dt_s);

    constrained_velocity_ned_ms = _constraint_base_velocity_ned_ms + velocity_delta_ned_ms;
    const Vector3f accel_applied_ned_mss = velocity_delta_ned_ms / _last_update_dt_s;

    Vector3p constrained_position_ned_m = _constraint_base_position_ned_m;
    Vector3f integrated_velocity_ned_ms = _constraint_base_velocity_ned_ms;
    update_pos_vel_accel(constrained_position_ned_m.x,
                         integrated_velocity_ned_ms.x,
                         accel_applied_ned_mss.x,
                         _last_update_dt_s,
                         0.0f,
                         0.0f,
                         0.0f);
    update_pos_vel_accel(constrained_position_ned_m.y,
                         integrated_velocity_ned_ms.y,
                         accel_applied_ned_mss.y,
                         _last_update_dt_s,
                         0.0f,
                         0.0f,
                         0.0f);
    update_pos_vel_accel(constrained_position_ned_m.z,
                         integrated_velocity_ned_ms.z,
                         accel_applied_ned_mss.z,
                         _last_update_dt_s,
                         0.0f,
                         0.0f,
                         0.0f);

    const Vector3f constraint_velocity_adjustment_ned_ms =
        integrated_velocity_ned_ms - _velocity_ref_ned_ms;
    const bool velocity_constraint_xy_applied =
        constraint_velocity_adjustment_ned_ms.xy().length_squared() > sq(reference_epsilon);
    const bool velocity_constraint_z_applied =
        fabsf(constraint_velocity_adjustment_ned_ms.z) > reference_epsilon;
    const bool velocity_constraint_applied = velocity_constraint_xy_applied ||
                                             velocity_constraint_z_applied;
    status.velocity_constraint_applied = velocity_constraint_applied;
    _position_ref_ned_m = constrained_position_ned_m;
    _velocity_ref_ned_ms = integrated_velocity_ned_ms;
    _accel_ref_ned_mss = accel_applied_ned_mss;
    // Most mode frames call this hook even when avoidance leaves the target
    // unchanged.  In that case preserve the reference generator's internal
    // shaper state, especially a just-settled zero-acceleration boundary.
    // Only a real external velocity change may rebase the shaper.
    if (velocity_constraint_xy_applied) {
        _accel_shaper_ned_mss.x = accel_applied_ned_mss.x;
        _accel_shaper_ned_mss.y = accel_applied_ned_mss.y;
    }
    if (velocity_constraint_z_applied) {
        _accel_shaper_ned_mss.z = accel_applied_ned_mss.z;
        _z_brake_settled = false;
        status.z_settled = false;
    }
    write_target(target);
    return true;
}

bool AC_Geometric_LoiterReference::anchor_position_z(
    postype_t position_z_ned_m,
    AC_Geometric_Target& target,
    AC_Geometric_LoiterReference_Status& status)
{
    if (!_initialized || !isfinite(position_z_ned_m)) {
        return false;
    }

    const postype_t position_delta_z_ned_m = position_z_ned_m - _position_ref_ned_m.z;
    _position_ref_ned_m.z = position_z_ned_m;
    _constraint_base_position_ned_m.z += position_delta_z_ned_m;
    write_target(target);
    status.vertical_position_anchored = true;
    return true;
}

bool AC_Geometric_LoiterReference::shift_reference(
    const Vector3f& position_delta_ned_m,
    float yaw_delta_rad)
{
    if (!_initialized ||
        !vector_is_finite(position_delta_ned_m) ||
        !isfinite(yaw_delta_rad)) {
        return false;
    }

    _position_ref_ned_m.x += position_delta_ned_m.x;
    _position_ref_ned_m.y += position_delta_ned_m.y;
    _position_ref_ned_m.z += position_delta_ned_m.z;
    _constraint_base_position_ned_m.x += position_delta_ned_m.x;
    _constraint_base_position_ned_m.y += position_delta_ned_m.y;
    _constraint_base_position_ned_m.z += position_delta_ned_m.z;
    _yaw_ref_rad = wrap_PI(_yaw_ref_rad + yaw_delta_rad);
    return true;
}
