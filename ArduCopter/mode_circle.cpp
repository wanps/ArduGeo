#include "Copter.h"
#include <AP_Mount/AP_Mount.h>

#if MODE_CIRCLE_ENABLED

static constexpr uint32_t circle_geometric_output_recent_ms = 100;

/*
 * Init and run calls for circle flight mode
 */

// circle_init - initialise circle controller flight mode
bool ModeCircle::init(bool ignore_checks)
{
    speed_changing = false;
    _geometric_circle_authorization.reset();
#if HAL_LOGGING_ENABLED
    _geometric_circle_log_counter = 0;
    _geometric_circle_observer_frames = 0;
#endif
    stop_geometric_circle_observer();
    _geometric_circle_update_count = copter.geometric_controller_updates();

    // set speed and acceleration limits
    pos_control->NE_set_max_speed_accel_m(wp_nav->get_default_speed_NE_ms(), wp_nav->get_wp_acceleration_mss());
    pos_control->NE_set_correction_speed_accel_m(wp_nav->get_default_speed_NE_ms(), wp_nav->get_wp_acceleration_mss());
    pos_control->D_set_max_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());
    pos_control->D_set_correction_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());

    // initialise circle controller including setting the circle center based on vehicle speed
    copter.circle_nav->init();

#if HAL_MOUNT_ENABLED
    // Check if the CIRCLE_OPTIONS parameter have roi_at_center
    if (copter.circle_nav->roi_at_center()) {
        const Vector3p &pos_ned_m { copter.circle_nav->get_center_NED_m() };
        Location circle_center;
        if (!AP::ahrs().get_location_from_origin_offset_NED(circle_center, pos_ned_m)) {
            return false;
        }
        // point at the ground:
        circle_center.set_alt_m(0, Location::AltFrame::ABOVE_TERRAIN);
        AP_Mount *s = AP_Mount::get_singleton();
        s->set_roi_target(circle_center);
    }
#endif

    // set auto yaw circle mode
    auto_yaw.set_mode(AutoYaw::Mode::CIRCLE);

    return true;
}

void ModeCircle::exit()
{
    // A new mode is initialised before this exit hook runs.  Invalidate the
    // shared output only if it has not already been replaced by that mode.
    const bool output_replaced =
        copter.geometric_controller_updates() != _geometric_circle_update_count;
    _geometric_circle_reference_supported = false;
    _geometric_circle_authorization.stop();
    if (!output_replaced) {
        copter.geometric_control.set_enabled(false);
    }
    _geometric_circle_authorization.reset();
#if HAL_LOGGING_ENABLED
    // @LoggerMessage: GECE
    // @Description: Circle geometric observer mode-exit cleanup and ownership snapshot
    // @Field: TimeUS: Time since system startup
    // @Field: Run: True if the geometric controller remains enabled
    // @Field: Age: Geometric controller output age
    // @Field: MFrm: Cumulative main-loop rate-controller frames
    // @Field: GFrm: Cumulative geometric motor-output frames
    // @Field: NFrm: Cumulative native rate-controller frames
    // @Field: Repl: True if the entering mode already replaced the Circle output
    AP::logger().Write("GECE", "TimeUS,Run,Age,MFrm,GFrm,NFrm,Repl", "QBIIIIB",
                       AP_HAL::micros64(),
                       (uint8_t)copter.geometric_control.enabled(),
                       copter.geometric_control.output_age_ms(AP_HAL::millis()),
                       copter.main_rate_controller_frames(),
                       copter.geometric_motor_output_frames(),
                       copter.native_rate_controller_frames(),
                       (uint8_t)output_replaced);
#endif
}

// circle_run - runs the circle flight mode
// should be called at 100hz or more
void ModeCircle::run()
{
    _geometric_circle_authorization.acknowledge_if_not_requested(
        copter.circle_nav->geometric_motor_output_requested());

    // set speed and acceleration limits
    pos_control->NE_set_max_speed_accel_m(wp_nav->get_default_speed_NE_ms(), wp_nav->get_wp_acceleration_mss());
    pos_control->D_set_max_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());

    // Check for any change in params and update in real time
    copter.circle_nav->check_param_change();

    // pilot changes to circle rate and radius
    // skip if in radio failsafe
    if (rc().has_valid_input() && copter.circle_nav->pilot_control_enabled()) {
        // update the circle controller's radius target based on pilot pitch stick inputs
        const float radius_current_m = copter.circle_nav->get_radius_m();               // circle controller's radius target, which begins as the circle_radius parameter
        const float pitch_stick_norm = channel_pitch->norm_input_dz();                  // pitch stick normalized -1 to 1
        const float nav_speed_ms = copter.wp_nav->get_default_speed_NE_ms();            // copter WP_NAV parameter speed
        const float radius_pilot_change_m = (pitch_stick_norm * nav_speed_ms) * G_Dt;   // rate of change (pitch stick up reduces the radius, as in moving forward)
        const float radius_new_m = MAX(radius_current_m + radius_pilot_change_m,0);     // new radius target

        if (!is_equal(radius_current_m, radius_new_m)) {
            copter.circle_nav->set_radius_m(radius_new_m);
        }

        // update the orbicular rate target based on pilot roll stick inputs
#if AP_RC_TRANSMITTER_TUNING_ENABLED
        // skip if using transmitter based tuning knob for circle rate
        if (!copter.being_tuned(TUNING_CIRCLE_RATE)) {
#else
        {
#endif
            const float roll_stick_norm = channel_roll->norm_input_dz();         // roll stick normalized -1 to 1

            if (is_zero(roll_stick_norm)) {
                // no speed change, so reset speed changing flag
                speed_changing = false;
            } else {
                const float rate_degs = copter.circle_nav->get_rate_degs();           // circle controller's rate target, which begins as the circle_rate parameter
                const float rate_current_degs = copter.circle_nav->get_rate_current(); // current adjusted rate target, which is probably different from _rate_degs
                const float rate_pilot_change_degs = (roll_stick_norm * G_Dt);        // rate of change from 0 to 1 degrees per second
                float rate_new_degs = rate_current_degs;                              // new rate target
                if (is_positive(rate_degs)) {
                    // currently moving clockwise, constrain 0 to 90
                    rate_new_degs = constrain_float(rate_current_degs + rate_pilot_change_degs, 0, 90);

                } else if (is_negative(rate_degs)) {
                    // currently moving counterclockwise, constrain -90 to 0
                    rate_new_degs = constrain_float(rate_current_degs + rate_pilot_change_degs, -90, 0);

                } else if (is_zero(rate_degs) && !speed_changing) {
                    // Stopped, pilot has released the roll stick, and pilot now wants to begin moving with the roll stick
                    rate_new_degs = rate_pilot_change_degs;
                }

                speed_changing = true;
                copter.circle_nav->set_rate_degs(rate_new_degs);
            }
        }
    }

    // get pilot desired climb rate (or zero if in radio failsafe)
    float target_climb_rate_ms = get_pilot_desired_climb_rate_ms();

    // get avoidance adjusted climb rate
    target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        stop_geometric_circle_observer();
#if HAL_LOGGING_ENABLED
        if (_geometric_circle_log_counter++ % 5 == 0) {
            log_geometric_circle_observer_status(
                false,
                static_cast<AC_AttitudeControl::HeadingMode>(UINT8_MAX));
            copter.Log_Write_Geometric_Frame_Counters();
        }
#endif
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if AP_RANGEFINDER_ENABLED
    // update the vertical offset based on the surface measurement
    copter.surface_tracking.update_surface_offset();
#endif

    const bool circle_updated = copter.circle_nav->update_ms(target_climb_rate_ms);
    copter.failsafe_terrain_set_status(circle_updated);
    pos_control->D_update_controller();

    // Use the same final AutoYaw command for the native controller and the
    // geometric observer so observation cannot reinterpret Circle yaw semantics.
    const AC_AttitudeControl::HeadingCommand heading = auto_yaw.get_heading();
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), heading);
    update_geometric_circle_observer(circle_updated, heading);
}

bool ModeCircle::geometric_circle_reference_supported(
    bool circle_updated,
    const AC_AttitudeControl::HeadingCommand& heading) const
{
    if (copter.flightmode != this ||
        !circle_updated ||
        copter.circle_nav->is_panorama() ||
        copter.circle_nav->center_is_terrain_alt() ||
        copter.is_tradheli() ||
        copter.geometric_motor_output_blocked_by_rate_thread()) {
        return false;
    }
#if AP_RANGEFINDER_ENABLED
    if (copter.surface_tracking.active()) {
        return false;
    }
#endif
    return heading.heading_mode == AC_AttitudeControl::HeadingMode::Angle_Only ||
           heading.heading_mode == AC_AttitudeControl::HeadingMode::Angle_And_Rate;
}

void ModeCircle::update_geometric_circle_observer(
    bool circle_updated,
    const AC_AttitudeControl::HeadingCommand& heading)
{
    const bool reference_supported = geometric_circle_reference_supported(circle_updated, heading);
    _geometric_circle_reference_supported = reference_supported;
    const bool motor_output_requested = copter.circle_nav->geometric_motor_output_requested();
    const bool observer_requested = copter.geometric_control.output_enabled();
    if (!reference_supported || !observer_requested) {
        if (reference_supported) {
            _geometric_circle_authorization.reject_if_active(motor_output_requested);
        }
        stop_geometric_circle_observer();
#if HAL_LOGGING_ENABLED
        if (_geometric_circle_log_counter++ % 5 == 0) {
            log_geometric_circle_observer_status(reference_supported, heading.heading_mode);
            copter.Log_Write_Geometric_Frame_Counters();
        }
#endif
        return;
    }

    AC_TrajectoryReference reference;
    reference.meta = make_control_reference_meta(AC_ControlReferenceCapability::TRAJECTORY);
    reference.position_ned_m = pos_control->get_pos_desired_NED_m();
    reference.velocity_ned_ms = pos_control->get_vel_desired_NED_ms();
    reference.acceleration_ned_mss = pos_control->get_accel_desired_NED_mss();
    reference.heading = heading;

    const AC_GeometricReferencePolicy policy {
        true,
        false,
        false,
        false
    };
    AC_Geometric_State geometric_state {};
    if (!run_geometric_observer(reference, nullptr, policy, true, geometric_state)) {
        _geometric_circle_authorization.reject_if_active(motor_output_requested);
        _geometric_circle_authorization.stop();
#if HAL_LOGGING_ENABLED
        if (_geometric_circle_log_counter++ % 5 == 0) {
            log_geometric_circle_observer_status(reference_supported, heading.heading_mode);
            copter.Log_Write_Geometric_Frame_Counters();
        }
#endif
        return;
    }
    _geometric_circle_update_count = copter.geometric_controller_updates();

    const bool motor_output_prepared =
        copter.geometric_control.output_is_fresh(AP_HAL::millis(), circle_geometric_output_recent_ms) &&
        copter.geometric_motor_output_is_valid();
    _geometric_circle_authorization.update(motor_output_prepared, motor_output_requested);

#if HAL_LOGGING_ENABLED
    _geometric_circle_observer_frames++;
    if (_geometric_circle_log_counter++ % 5 == 0) {
        const uint32_t now_ms = AP_HAL::millis();
        const AC_Geometric_Output& output = copter.geometric_control.get_output();
        const uint32_t geometric_age_ms = copter.geometric_control.output_age_ms(now_ms);
        const uint32_t motor_output_age_ms = copter.geometric_motor_output_age_ms(now_ms);
        const bool motor_output_allowed = allows_geometric_motor_output();
        const bool rate_thread_active = copter.geometric_motor_output_blocked_by_rate_thread();
        const bool motor_output_written_recently = motor_output_age_ms <= circle_geometric_output_recent_ms;

        // @LoggerMessage: GECW
        // @Description: Circle neutral reference accepted by the geometric observer
        // @Field: TimeUS: Time since system startup
        // @Field: PX: Native desired local NED position, X-Axis
        // @Field: PY: Native desired local NED position, Y-Axis
        // @Field: PZ: Native desired local NED position, Z-Axis
        // @Field: VX: Native desired local NED velocity, X-Axis
        // @Field: VY: Native desired local NED velocity, Y-Axis
        // @Field: VZ: Native desired local NED velocity, Z-Axis
        // @Field: AX: Native desired local NED acceleration, X-Axis
        // @Field: AY: Native desired local NED acceleration, Y-Axis
        // @Field: AZ: Native desired local NED acceleration, Z-Axis
        // @Field: Yaw: Final Native AutoYaw angle reference
        // @Field: YR: Final Native AutoYaw rate reference
        // @Field: Frm: Neutral reference frame
        // @Field: Cap: Neutral reference capability
        // @Field: HMode: Heading command semantic mode
        // @Field: Age: Neutral reference age
        AP::logger().WriteStreaming("GECW", "TimeUS,PX,PY,PZ,VX,VY,VZ,AX,AY,AZ,Yaw,YR,Frm,Cap,HMode,Age", "QfffffffffffBBBI",
                                    AP_HAL::micros64(),
                                    (double)reference.position_ned_m.x,
                                    (double)reference.position_ned_m.y,
                                    (double)reference.position_ned_m.z,
                                    (double)reference.velocity_ned_ms.x,
                                    (double)reference.velocity_ned_ms.y,
                                    (double)reference.velocity_ned_ms.z,
                                    (double)reference.acceleration_ned_mss.x,
                                    (double)reference.acceleration_ned_mss.y,
                                    (double)reference.acceleration_ned_mss.z,
                                    (double)reference.heading.yaw_angle_rad,
                                    (double)reference.heading.yaw_rate_rads,
                                    (uint8_t)reference.meta.frame,
                                    (uint8_t)reference.meta.capability,
                                    (uint8_t)reference.heading.heading_mode,
                                    now_ms - reference.meta.timestamp_ms);
        log_geometric_circle_observer_status(true, heading.heading_mode);
        copter.Log_Write_Geometric_Attitude_Error(output.attitude);
        copter.Log_Write_Geometric_Output_State(motor_output_allowed,
                                                copter.geometric_control.output_enabled(),
                                                rate_thread_active,
                                                motor_output_written_recently,
                                                geometric_age_ms,
                                                motor_output_age_ms,
                                                output.mapped);
        copter.Log_Write_Geometric_Frame_Counters();
    }
#endif
}

void ModeCircle::stop_geometric_circle_observer()
{
    _geometric_circle_reference_supported = false;
    _geometric_circle_authorization.stop();
    copter.geometric_control.set_enabled(false);
}

#if HAL_LOGGING_ENABLED
void ModeCircle::log_geometric_circle_observer_status(
    bool reference_supported,
    AC_AttitudeControl::HeadingMode heading_mode)
{
#if AP_RANGEFINDER_ENABLED
    const bool surface_tracking_active = copter.surface_tracking.active();
#else
    const bool surface_tracking_active = false;
#endif

    // @LoggerMessage: GECS
    // @Description: Circle geometric observer support and cache status
    // @Field: TimeUS: Time since system startup
    // @Field: Rad: Current Circle radius
    // @Field: Rate: Current Circle angular rate
    // @Field: Run: True if the geometric controller is enabled
    // @Field: Sup: True if the current Circle reference is structurally supported
    // @Field: Req: True if GEO_OUT_EN requests geometric observation
    // @Field: AutoY: Native AutoYaw mode
    // @Field: HMode: Heading command semantic mode
    // @Field: Shp: True if the geometric controller reshaped the Native reference
    // @Field: Terr: True if the Circle center altitude is terrain-relative
    // @Field: Surf: True if rangefinder surface tracking is active
    // @Field: Age: Geometric controller output age
    // @Field: CFrm: Cumulative Circle geometric calculation frames
    AP::logger().WriteStreaming("GECS", "TimeUS,Rad,Rate,Run,Sup,Req,AutoY,HMode,Shp,Terr,Surf,Age,CFrm", "QffBBBBBBBBII",
                                AP_HAL::micros64(),
                                (double)copter.circle_nav->get_radius_m(),
                                (double)copter.circle_nav->get_rate_current(),
                                (uint8_t)copter.geometric_control.enabled(),
                                (uint8_t)reference_supported,
                                (uint8_t)copter.geometric_control.output_enabled(),
                                (uint8_t)auto_yaw.mode(),
                                (uint8_t)heading_mode,
                                (uint8_t)copter.geometric_control.shaper_active(),
                                (uint8_t)copter.circle_nav->center_is_terrain_alt(),
                                (uint8_t)surface_tracking_active,
                                copter.geometric_control.output_age_ms(AP_HAL::millis()),
                                _geometric_circle_observer_frames);

    // @LoggerMessage: GECA
    // @Description: Circle geometric motor-output authorization state
    // @Field: TimeUS: Time since system startup
    // @Field: Req: True if CIRCLE_OPTIONS requests geometric motor output
    // @Field: Prep: True if a fresh finite geometric output is prepared
    // @Field: Act: True if the mode authorizes geometric motor output
    // @Field: Rej: True if a hard-fault latch blocks geometric motor output
    AP::logger().WriteStreaming("GECA", "TimeUS,Req,Prep,Act,Rej", "QBBBB",
                                AP_HAL::micros64(),
                                (uint8_t)copter.circle_nav->geometric_motor_output_requested(),
                                (uint8_t)_geometric_circle_authorization.prepared,
                                (uint8_t)allows_geometric_motor_output(),
                                (uint8_t)_geometric_circle_authorization.rejected);
}
#endif

bool ModeCircle::allows_geometric_motor_output() const
{
    return _geometric_circle_reference_supported &&
           _geometric_circle_authorization.allows_output(
               copter.circle_nav->geometric_motor_output_requested());
}

void ModeCircle::handle_geometric_motor_output_fallback()
{
    const bool motor_output_requested = copter.circle_nav->geometric_motor_output_requested();
    if (motors->armed() &&
        _geometric_circle_reference_supported &&
        !copter.geometric_motor_output_blocked_by_rate_thread()) {
        _geometric_circle_authorization.reject_if_active(motor_output_requested);
    }
    _geometric_circle_authorization.acknowledge_if_not_requested(motor_output_requested);
    _geometric_circle_authorization.stop();
    Mode::handle_geometric_motor_output_fallback();
}

float ModeCircle::wp_distance_m() const
{
    return copter.circle_nav->get_distance_to_target_m();
}

float ModeCircle::wp_bearing_deg() const
{
    return degrees(copter.circle_nav->get_bearing_to_target_rad());
}

#endif
