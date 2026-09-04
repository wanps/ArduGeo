#include "Copter.h"

#if MODE_RTL_ENABLED

static constexpr uint32_t rtl_wpnav_geometric_output_recent_ms = 100;

// table of user settable parameters
const AP_Param::GroupInfo ModeRTL::var_info[] = {

    // @Param: ALT_M
    // @DisplayName: RTL Altitude
    // @Description: The minimum alt above home the vehicle will climb to before returning. If the vehicle is flying higher than this value it will return at its current altitude.
    // @Units: m
    // @Range: 0.30 3000
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("ALT_M", 1, ModeRTL, altitude_m, RTL_ALT_M_DEFAULT),

    // @Param: ALT_FINAL_M
    // @DisplayName: RTL Final Altitude
    // @Description: Altitude the vehicle will move to as the final stage of Returning to Launch or after completing a mission. Set to zero to land.
    // @Units: m
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("ALT_FINAL_M", 2, ModeRTL, alt_final_m, RTL_ALT_FINAL_M_DEFAULT),

    // @Param: CLIMB_MIN_M
    // @DisplayName: RTL minimum climb
    // @Description: The vehicle will climb this many meters during the initial climb portion of the RTL
    // @Units: m
    // @Range: 0 30
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("CLIMB_MIN_M", 3, ModeRTL, climb_min_m, RTL_CLIMB_MIN_M_DEFAULT),

    // @Param: SPEED_MS
    // @DisplayName: RTL speed
    // @Description: The speed in m/s which the aircraft will attempt to maintain horizontally while flying home. If this is set to zero, WP_SPD will be used instead.
    // @Units: m/s
    // @Range: 0 20
    // @Increment: 0.5
    // @User: Standard
    AP_GROUPINFO("SPEED_MS", 4, ModeRTL, speed_ms, 0),

    AP_GROUPEND
};

// constructor
ModeRTL::ModeRTL() : Mode()
{
    // load parameter defaults
    AP_Param::setup_object_defaults(this, var_info);
}

// convert parameters
void ModeRTL::convert_params()
{
    // PARAMETER_CONVERSION - Added: Jan 2026

    // return immediately if parameter conversion has already been performed
    if (altitude_m.configured() || speed_ms.configured() || alt_final_m.configured() || climb_min_m.configured()) {
        return;
    }

    static const AP_Param::ConversionInfo conversion_info[] = {
        { Parameters::k_param_rtl_altitude_cm, 0, AP_PARAM_INT32, "RTL_ALT_M" },        // RTL_ALT moved to RTL_ALT_M
        { Parameters::k_param_rtl_speed_cms, 0, AP_PARAM_INT16, "RTL_SPEED_MS" },       // RTL_SPEED moved to RTL_SPEED_MS
        { Parameters::k_param_rtl_alt_final_cm, 0, AP_PARAM_INT16, "RTL_ALT_FINAL_M" }, // RTL_ALT_FINAL moved to RTL_ALT_FINAL_M
        { Parameters::k_param_rtl_climb_min_cm, 0, AP_PARAM_INT16, "RTL_CLIMB_MIN_M" }, // RTL_CLIMB_MIN moved to RTL_CLIMB_MIN_M
    };
    AP_Param::convert_old_parameters_scaled(conversion_info, ARRAY_SIZE(conversion_info), 0.01, 0);
}

/*
 * Init and run calls for RTL flight mode
 *
 * There are two parts to RTL, the high level decision making which controls which state we are in
 * and the lower implementation of the waypoint or landing controllers within those states
 */

// rtl_init - initialise rtl controller
bool ModeRTL::init(bool ignore_checks)
{
    if (!ignore_checks) {
        if (!AP::ahrs().home_is_set()) {
            return false;
        }
    }
    _geometric_wpnav_authorization.reset();
    stop_geometric_wpnav_observer();
#if HAL_LOGGING_ENABLED
    _geometric_wpnav_log_counter = 0;
    _geometric_wpnav_observer_frames = 0;
#endif
    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init_m(speed_ms.get());
    _state = SubMode::STARTING;
    _state_complete = true; // see run() method below
    terrain_following_allowed = !copter.failsafe.terrain;
    // reset flag indicating if pilot has applied roll or pitch inputs during landing
    copter.ap.land_repo_active = false;

    // this will be set true if prec land is later active
    copter.ap.prec_land_active = false;

#if AC_PRECLAND_ENABLED
    // initialise precland state machine
    copter.precland_statemachine.init();
#endif

    return true;
}

void ModeRTL::exit()
{
    stop_geometric_wpnav_observer();
    _geometric_wpnav_authorization.reset();
}

// re-start RTL with terrain following disabled
void ModeRTL::restart_without_terrain()
{
    stop_geometric_wpnav_observer();
#if HAL_LOGGING_ENABLED
    LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::RESTARTED_RTL);
#endif
    terrain_following_allowed = false;
    _state = SubMode::STARTING;
    _state_complete = true;
    gcs().send_text(MAV_SEVERITY_CRITICAL,"Restarting RTL - Terrain data missing");
}

ModeRTL::RTLAltType ModeRTL::get_alt_type() const
{
    // sanity check parameter
    switch ((ModeRTL::RTLAltType)g.rtl_alt_type) {
    case RTLAltType::RELATIVE ... RTLAltType::TERRAIN:
        return g.rtl_alt_type;
    }
    // user has an invalid value
    return RTLAltType::RELATIVE;
}

// rtl_run - runs the return-to-launch controller
// should be called at 100hz or more
void ModeRTL::run(bool disarm_on_land)
{
    _geometric_wpnav_authorization.acknowledge_if_not_requested(
        option_is_enabled(Option::GeometricMotorOutput));

    if (!motors->armed()) {
        stop_geometric_wpnav_observer();
        return;
    }

    // check if we need to move to next state
    if (_state_complete) {
        switch (_state) {
        case SubMode::STARTING:
            build_path();
            climb_start();
            break;
        case SubMode::INITIAL_CLIMB:
            return_start();
            break;
        case SubMode::RETURN_HOME:
            loiterathome_start();
            break;
        case SubMode::LOITER_AT_HOME:
            if (rtl_path.land || copter.failsafe.radio) {
                land_start();
            } else {
                descent_start();
            }
            break;
        case SubMode::FINAL_DESCENT:
            // do nothing
            break;
        case SubMode::LAND:
            // do nothing - rtl_land_run will take care of disarming motors
            break;
        }
    }

    // call the correct run function
    switch (_state) {

    case SubMode::STARTING:
        // should not be reached:
        _state = SubMode::INITIAL_CLIMB;
        FALLTHROUGH;

    case SubMode::INITIAL_CLIMB:
    case SubMode::RETURN_HOME:
        climb_return_run();
        break;

    case SubMode::LOITER_AT_HOME:
        loiterathome_run();
        break;

    case SubMode::FINAL_DESCENT:
        descent_run();
        break;

    case SubMode::LAND:
        land_run(disarm_on_land);
        break;
    }
}

// rtl_climb_start - initialise climb to RTL altitude
void ModeRTL::climb_start()
{
    _state = SubMode::INITIAL_CLIMB;
    _state_complete = false;

    // set the destination
    if (!wp_nav->set_wp_destination_loc(rtl_path.climb_target) || !wp_nav->set_wp_destination_next_loc(rtl_path.return_target)) {
        // this should not happen because rtl_build_path will have checked terrain data was available
        gcs().send_text(MAV_SEVERITY_CRITICAL,"RTL: unexpected error setting climb target");
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_SET_DESTINATION);
        copter.set_mode(Mode::Number::LAND, ModeReason::TERRAIN_FAILSAFE);
        return;
    }

    // hold current yaw during initial climb
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);
}

// rtl_return_start - initialise return to home
void ModeRTL::return_start()
{
    _state = SubMode::RETURN_HOME;
    _state_complete = false;

    if (!wp_nav->set_wp_destination_loc(rtl_path.return_target)) {
        // failure must be caused by missing terrain data, restart RTL
        restart_without_terrain();
    }

    // initialise yaw to point home (maybe)
    auto_yaw.set_mode_to_default(true);
}

// rtl_climb_return_run - implements the initial climb, return home and descent portions of RTL which all rely on the wp controller
//      called by rtl_run at 100hz or more
void ModeRTL::climb_return_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    pos_control->D_update_controller();

    // Use the same final AutoYaw command for the native controller and the
    // geometric observer so observation cannot reinterpret RTL yaw semantics.
    const AC_AttitudeControl::HeadingCommand heading = auto_yaw.get_heading();
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), heading);
    update_geometric_wpnav_observer(heading);

    // check if we've completed this stage of RTL
    _state_complete = wp_nav->reached_wp_destination();
}

// loiterathome_start - initialise return to home
void ModeRTL::loiterathome_start()
{
    _state = SubMode::LOITER_AT_HOME;
    _state_complete = false;
    _loiter_start_time = millis();

    // yaw back to initial take-off heading yaw unless pilot has already overridden yaw
    if (auto_yaw.default_mode(true) != AutoYaw::Mode::HOLD) {
        auto_yaw.set_mode(AutoYaw::Mode::RESET_TO_ARMED_YAW);
    } else {
        auto_yaw.set_mode(AutoYaw::Mode::HOLD);
    }
}

// rtl_climb_return_descent_run - implements the initial climb, return home and descent portions of RTL which all rely on the wp controller
//      called by rtl_run at 100hz or more
void ModeRTL::loiterathome_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    pos_control->D_update_controller();

    // Use the same final AutoYaw command for the native controller and the
    // geometric observer so observation cannot reinterpret RTL yaw semantics.
    const AC_AttitudeControl::HeadingCommand heading = auto_yaw.get_heading();
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), heading);
    update_geometric_wpnav_observer(heading);

    // check if we've completed this stage of RTL
    if ((millis() - _loiter_start_time) >= (uint32_t)g.rtl_loiter_time.get()) {
        if (auto_yaw.mode() == AutoYaw::Mode::RESET_TO_ARMED_YAW) {
            // check if heading is within 2 degrees of heading when vehicle was armed
            // todo: Use the target heading instead of the actual heading to allow landing even if yaw control is lost.
            if (fabsf(wrap_PI(ahrs.get_yaw_rad() - copter.initial_armed_bearing_rad)) <= radians(2.0)) {
                _state_complete = true;
            }
        } else {
            // we have loitered long enough
            _state_complete = true;
        }
    }
}

// rtl_descent_start - initialise descent to final alt
void ModeRTL::descent_start()
{
    _state = SubMode::FINAL_DESCENT;
    _state_complete = false;
    stop_geometric_wpnav_observer();

    // initialise altitude target to stopping point
    pos_control->D_init_controller_stopping_point();

    // initialise yaw
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

#if AP_LANDINGGEAR_ENABLED
    // optionally deploy landing gear
    copter.landinggear.deploy_for_landing();
#endif
}

// rtl_descent_run - implements the final descent to the RTL_ALT_M
//      called by rtl_run at 100hz or more
void ModeRTL::descent_run()
{
    Vector2f vel_correction_ms;

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // process pilot's input
    if (rc().has_valid_input()) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            LOGGER_WRITE_EVENT(LogEvent::LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            if (!copter.set_mode(Mode::Number::LOITER, ModeReason::THROTTLE_LAND_ESCAPE)) {
                copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::THROTTLE_LAND_ESCAPE);
            }
        }

        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // convert pilot input to reposition velocity
            vel_correction_ms = get_pilot_desired_velocity(wp_nav->get_wp_acceleration_mss() * 0.5);

            // record if pilot has overridden roll or pitch
            if (!vel_correction_ms.is_zero()) {
                if (!copter.ap.land_repo_active) {
                    LOGGER_WRITE_EVENT(LogEvent::LAND_REPO_ACTIVE);
                }
                copter.ap.land_repo_active = true;
            }
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    Vector2f accel;
    pos_control->input_vel_accel_NE_m(vel_correction_ms, accel);
    pos_control->NE_update_controller();

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    pos_control->D_set_alt_target_with_slew_m(rtl_path.descent_target.alt * 0.01);
    pos_control->D_update_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    const AC_AttitudeControl::HeadingCommand heading = auto_yaw.get_heading();
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), heading);
    update_geometric_wpnav_observer(heading);

    // check if we've reached within 20cm of final altitude
    _state_complete = fabsf(rtl_path.descent_target.alt * 0.01 - pos_control->get_pos_estimate_U_m()) < 0.2;
}

// land_start - initialise controllers to loiter over home
void ModeRTL::land_start()
{
    _state = SubMode::LAND;
    _state_complete = false;
    stop_geometric_wpnav_observer();

    // set horizontal speed and acceleration limits
    pos_control->NE_set_max_speed_accel_m(wp_nav->get_default_speed_NE_ms(), wp_nav->get_wp_acceleration_mss());
    pos_control->NE_set_correction_speed_accel_m(wp_nav->get_default_speed_NE_ms(), wp_nav->get_wp_acceleration_mss());

    // initialise loiter target destination
    if (!pos_control->NE_is_active()) {
        pos_control->NE_init_controller();
    }

    // initialise the vertical position controller
    if (!pos_control->D_is_active()) {
        pos_control->D_init_controller();
    }

    // initialise yaw
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

#if AP_LANDINGGEAR_ENABLED
    // optionally deploy landing gear
    copter.landinggear.deploy_for_landing();
#endif
}

bool ModeRTL::is_landing() const
{
    return _state == SubMode::LAND;
}

// land_run - run the landing controllers to put the aircraft on the ground
// called by rtl_run at 100hz or more
void ModeRTL::land_run(bool disarm_on_land)
{
    // check if we've completed this stage of RTL
    _state_complete = copter.ap.land_complete;

    // disarm when the landing detector says we've landed
    if (disarm_on_land && copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
        copter.arming.disarm(AP_Arming::Method::LANDED);
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run normal landing or precision landing (if enabled)
    land_run_normal_or_precland();
    stop_geometric_wpnav_observer(true);
}

bool ModeRTL::geometric_wpnav_reference_supported(const AC_AttitudeControl::HeadingCommand& heading) const
{
    const bool wpnav_phase = _state == SubMode::RETURN_HOME ||
                             _state == SubMode::LOITER_AT_HOME;
    return copter.flightmode == this &&
           wpnav_phase &&
           !wp_nav->origin_and_destination_are_terrain_alt() &&
           !copter.is_tradheli() &&
           !copter.geometric_motor_output_blocked_by_rate_thread() &&
           (heading.heading_mode == AC_AttitudeControl::HeadingMode::Angle_Only ||
            heading.heading_mode == AC_AttitudeControl::HeadingMode::Angle_And_Rate);
}

void ModeRTL::update_geometric_wpnav_observer(const AC_AttitudeControl::HeadingCommand& heading)
{
    const bool reference_supported = geometric_wpnav_reference_supported(heading);
    _geometric_wpnav_reference_supported = reference_supported;
    const bool motor_output_requested = option_is_enabled(Option::GeometricMotorOutput);
    const bool observer_requested = copter.geometric_control.output_enabled();
    if (!reference_supported || !observer_requested) {
        if (reference_supported) {
            _geometric_wpnav_authorization.reject_if_active(motor_output_requested);
        }
        stop_geometric_wpnav_observer();
#if HAL_LOGGING_ENABLED
        if (_geometric_wpnav_log_counter++ % 5 == 0) {
            log_geometric_wpnav_observer_status(reference_supported, heading.heading_mode);
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
        _geometric_wpnav_authorization.reject_if_active(motor_output_requested);
        _geometric_wpnav_authorization.stop();
#if HAL_LOGGING_ENABLED
        if (_geometric_wpnav_log_counter++ % 5 == 0) {
            log_geometric_wpnav_observer_status(reference_supported, heading.heading_mode);
            copter.Log_Write_Geometric_Frame_Counters();
        }
#endif
        return;
    }

    const bool motor_output_prepared =
        copter.geometric_control.output_is_fresh(AP_HAL::millis(), rtl_wpnav_geometric_output_recent_ms) &&
        copter.geometric_motor_output_is_valid();
    _geometric_wpnav_authorization.update(motor_output_prepared, motor_output_requested);

#if HAL_LOGGING_ENABLED
    _geometric_wpnav_observer_frames++;
    if (_geometric_wpnav_log_counter++ % 5 == 0) {
        const uint32_t now_ms = AP_HAL::millis();
        const AC_Geometric_Output& output = copter.geometric_control.get_output();
        const uint32_t geometric_age_ms = copter.geometric_control.output_age_ms(now_ms);
        const uint32_t motor_output_age_ms = copter.geometric_motor_output_age_ms(now_ms);
        const bool motor_output_allowed = allows_geometric_motor_output();
        const bool rate_thread_active = copter.geometric_motor_output_blocked_by_rate_thread();
        const bool motor_output_written_recently = motor_output_age_ms <= rtl_wpnav_geometric_output_recent_ms;

        // @LoggerMessage: GERW
        // @Description: RTL WPNav neutral reference accepted by the geometric observer
        // @Field: TimeUS: Time since system startup
        // @Field: Phs: RTL phase
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
        // @Field: HMode: Heading command semantic mode
        // @Field: Age: Neutral reference age
        AP::logger().WriteStreaming("GERW", "TimeUS,Phs,PX,PY,PZ,VX,VY,VZ,AX,AY,AZ,Yaw,YR,Frm,HMode,Age", "QBfffffffffffBBI",
                                    AP_HAL::micros64(),
                                    (uint8_t)_state,
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
                                    (uint8_t)reference.heading.heading_mode,
                                    now_ms - reference.meta.timestamp_ms);
        log_geometric_wpnav_observer_status(true, heading.heading_mode);
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

void ModeRTL::stop_geometric_wpnav_observer(bool log_unsupported)
{
    _geometric_wpnav_reference_supported = false;
    _geometric_wpnav_authorization.stop();
    copter.geometric_control.set_enabled(false);
#if HAL_LOGGING_ENABLED
    if (log_unsupported && _geometric_wpnav_log_counter++ % 5 == 0) {
        log_geometric_wpnav_observer_status(false,
                                             static_cast<AC_AttitudeControl::HeadingMode>(UINT8_MAX));
        copter.Log_Write_Geometric_Frame_Counters();
    }
#else
    (void)log_unsupported;
#endif
}

#if HAL_LOGGING_ENABLED
void ModeRTL::log_geometric_wpnav_observer_status(bool reference_supported,
                                                  AC_AttitudeControl::HeadingMode heading_mode)
{
    // @LoggerMessage: GERS
    // @Description: RTL WPNav geometric observer support and cache status
    // @Field: TimeUS: Time since system startup
    // @Field: Phs: RTL phase
    // @Field: Run: True if the geometric controller is enabled
    // @Field: Sup: True if the current RTL reference is structurally supported
    // @Field: Req: True if RTL_OPTIONS requests geometric motor output
    // @Field: Prep: True if a fresh finite geometric output is prepared
    // @Field: Act: True if the mode authorizes geometric motor output
    // @Field: Rej: True if a hard-fault latch blocks geometric motor output
    // @Field: AutoY: Native AutoYaw mode
    // @Field: HMode: Heading command semantic mode
    // @Field: Shp: True if the geometric controller reshaped the Native reference
    // @Field: Age: Geometric controller output age
    // @Field: CFrm: Cumulative RTL WPNav geometric calculation frames
    AP::logger().WriteStreaming("GERS", "TimeUS,Phs,Run,Sup,Req,Prep,Act,Rej,AutoY,HMode,Shp,Age,CFrm", "QBBBBBBBBBBII",
                                AP_HAL::micros64(),
                                (uint8_t)_state,
                                (uint8_t)copter.geometric_control.enabled(),
                                (uint8_t)reference_supported,
                                (uint8_t)option_is_enabled(Option::GeometricMotorOutput),
                                (uint8_t)_geometric_wpnav_authorization.prepared,
                                (uint8_t)allows_geometric_motor_output(),
                                (uint8_t)_geometric_wpnav_authorization.rejected,
                                (uint8_t)auto_yaw.mode(),
                                (uint8_t)heading_mode,
                                (uint8_t)copter.geometric_control.shaper_active(),
                                copter.geometric_control.output_age_ms(AP_HAL::millis()),
                                _geometric_wpnav_observer_frames);
}
#endif

bool ModeRTL::allows_geometric_motor_output() const
{
    return _geometric_wpnav_reference_supported &&
           _geometric_wpnav_authorization.allows_output(
               option_is_enabled(Option::GeometricMotorOutput));
}

void ModeRTL::handle_geometric_motor_output_fallback()
{
    const bool motor_output_requested = option_is_enabled(Option::GeometricMotorOutput);
    if (motors->armed() &&
        _geometric_wpnav_reference_supported &&
        !copter.geometric_motor_output_blocked_by_rate_thread()) {
        _geometric_wpnav_authorization.reject_if_active(motor_output_requested);
    }
    _geometric_wpnav_authorization.acknowledge_if_not_requested(motor_output_requested);
    _geometric_wpnav_authorization.stop();
    Mode::handle_geometric_motor_output_fallback();
}

void ModeRTL::build_path()
{
    // origin point is our stopping point
    rtl_path.origin_point = get_stopping_point();
    rtl_path.origin_point.change_alt_frame(Location::AltFrame::ABOVE_HOME);

    // compute return target
    compute_return_target();

    // climb target is above our origin point at the return altitude
    rtl_path.climb_target = Location(rtl_path.origin_point.lat, rtl_path.origin_point.lng, rtl_path.return_target.alt, rtl_path.return_target.get_alt_frame());

    // descent target is below return target at rtl_alt_final_m
    rtl_path.descent_target = Location(rtl_path.return_target.lat, rtl_path.return_target.lng, alt_final_m.get() * 100, Location::AltFrame::ABOVE_HOME);

    // Target altitude is passed directly to the position controller so must be relative to origin
    rtl_path.descent_target.change_alt_frame(Location::AltFrame::ABOVE_ORIGIN);

    // set land flag
    rtl_path.land = alt_final_m.get() <= 0;
}

// compute the return target - home or rally point
//   return target's altitude is updated to a higher altitude that the vehicle can safely return at (frame may also be set)
void ModeRTL::compute_return_target()
{
    // set return target to nearest rally point or home position
#if HAL_RALLY_ENABLED
    rtl_path.return_target = copter.rally.calc_best_rally_or_home_location(copter.current_loc, ahrs.get_home().alt);
    rtl_path.return_target.change_alt_frame(Location::AltFrame::ABSOLUTE);
#else
    rtl_path.return_target = ahrs.get_home();
#endif

    // get position controller Z-axis offset in cm above EKF origin
    float pos_offset_u_m = pos_control->get_pos_offset_U_m();

    // curr_alt_m is current altitude, with any offset removed, above home or above terrain depending upon use_terrain
    float curr_alt_m = copter.current_loc.alt * 0.01 - pos_offset_u_m;

    // determine altitude type of return journey (alt-above-home, alt-above-terrain using range finder or alt-above-terrain using terrain database)
    ReturnTargetAltType alt_type = ReturnTargetAltType::RELATIVE;
    if (terrain_following_allowed && (get_alt_type() == RTLAltType::TERRAIN)) {
        // convert RTL_ALT_TYPE and WP_RFNG_USE parameters to ReturnTargetAltType
        switch (wp_nav->get_terrain_source()) {
        case AC_WPNav::TerrainSource::TERRAIN_UNAVAILABLE:
            alt_type = ReturnTargetAltType::RELATIVE;
            LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::RTL_MISSING_RNGFND);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "RTL: no terrain data, using alt-above-home");
            break;
        case AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER:
            alt_type = ReturnTargetAltType::RANGEFINDER;
            break;
        case AC_WPNav::TerrainSource::TERRAIN_FROM_TERRAINDATABASE:
            alt_type = ReturnTargetAltType::TERRAINDATABASE;
            break;
        }
    }

    // set curr_alt_m and return_target.alt from range finder
    if (alt_type == ReturnTargetAltType::RANGEFINDER) {
        if (copter.get_rangefinder_height_interpolated_m(curr_alt_m)) {
            // subtract vertical offset from altitude.
            curr_alt_m -= pos_offset_u_m;
            // set return_target.alt
            rtl_path.return_target.set_alt_m(MAX(curr_alt_m + MAX(0.0f, climb_min_m.get()), MAX(altitude_m.get(), RTL_ALT_MIN_M)), Location::AltFrame::ABOVE_TERRAIN);
        } else {
            // fallback to relative alt and warn user
            alt_type = ReturnTargetAltType::RELATIVE;
            gcs().send_text(MAV_SEVERITY_CRITICAL, "RTL: rangefinder unhealthy, using alt-above-home");
            LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::RTL_MISSING_RNGFND);
        }
    }

    // set curr_alt_m and return_target.alt from terrain database
    if (alt_type == ReturnTargetAltType::TERRAINDATABASE) {
        // set curr_alt_m to current altitude above terrain
        // convert return_target.alt from an abs (above MSL) to altitude above terrain
        //   Note: the return_target may be a rally point with the alt set above the terrain alt (like the top of a building)
        float curr_terr_alt_m;
        if (copter.current_loc.get_alt_m(Location::AltFrame::ABOVE_TERRAIN, curr_terr_alt_m) &&
            rtl_path.return_target.change_alt_frame(Location::AltFrame::ABOVE_TERRAIN)) {
            // subtract vertical offset from altitude.
            curr_alt_m = curr_terr_alt_m - pos_offset_u_m;
        } else {
            // fallback to relative alt and warn user
            alt_type = ReturnTargetAltType::RELATIVE;
            LOGGER_WRITE_ERROR(LogErrorSubsystem::TERRAIN, LogErrorCode::MISSING_TERRAIN_DATA);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "RTL: no terrain data, using alt-above-home");
        }
    }

    // for the default case we must convert return-target alt (which is an absolute alt) to alt-above-home
    if (alt_type == ReturnTargetAltType::RELATIVE) {
        if (!rtl_path.return_target.change_alt_frame(Location::AltFrame::ABOVE_HOME)) {
            // this should never happen but just in case
            rtl_path.return_target.set_alt_m(0, Location::AltFrame::ABOVE_HOME);
            gcs().send_text(MAV_SEVERITY_WARNING, "RTL: unexpected error calculating target alt");
        }
    }

    // set new target altitude to return target altitude
    // Note: this is alt-above-home or terrain-alt depending upon rtl_alt_type
    // Note: ignore negative altitudes which could happen if user enters negative altitude for rally point or terrain is higher at rally point compared to home
    float target_alt_m = MAX(rtl_path.return_target.alt, 0) * 0.01;

    // increase target to maximum of current altitude + climb_min and rtl altitude
    const float min_rtl_alt_m = MAX(RTL_ALT_MIN_M, curr_alt_m + MAX(0.0f, climb_min_m.get()));
    target_alt_m = MAX(target_alt_m, MAX(altitude_m.get(), min_rtl_alt_m));

    // reduce climb if close to return target
    float rtl_return_dist_m = rtl_path.return_target.get_distance(rtl_path.origin_point);
    // don't allow really shallow slopes
    if (g.rtl_cone_slope >= RTL_MIN_CONE_SLOPE) {
        target_alt_m = MIN(target_alt_m, MAX(rtl_return_dist_m * g.rtl_cone_slope, min_rtl_alt_m));
    }

    // set returned target alt to new target_alt_m (don't change altitude type)
    rtl_path.return_target.set_alt_m(target_alt_m, (alt_type == ReturnTargetAltType::RELATIVE) ? Location::AltFrame::ABOVE_HOME : Location::AltFrame::ABOVE_TERRAIN);

#if AP_FENCE_ENABLED
    // ensure not above fence altitude if alt fence is enabled
    // Note: because the rtl_path.climb_target's altitude is simply copied from the return_target's altitude,
    //       if terrain altitudes are being used, the code below which reduces the return_target's altitude can lead to
    //       the vehicle not climbing at all as RTL begins.  This can be overly conservative and it might be better
    //       to apply the fence alt limit independently on the origin_point and return_target
    if ((copter.fence.get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX) != 0) {
        // get return target in max alt frame so it can be compared to fence's alt
        if (rtl_path.return_target.get_alt_m(copter.fence.get_alt_max_frame(), target_alt_m)) {
            float fence_alt_m = copter.fence.get_safe_alt_max_m();
            if (target_alt_m > fence_alt_m) {
                // reduce target alt to the fence alt
                rtl_path.return_target.alt -= (target_alt_m - fence_alt_m) * 100.0;
            }
        }
    }
#endif

    // ensure we do not descend
    rtl_path.return_target.alt = MAX(rtl_path.return_target.alt, curr_alt_m * 100.0);
}

bool ModeRTL::get_wp(Location& destination) const
{
    // provide target in states which use wp_nav
    switch (_state) {
    case SubMode::STARTING:
    case SubMode::INITIAL_CLIMB:
    case SubMode::RETURN_HOME:
    case SubMode::LOITER_AT_HOME:
    case SubMode::FINAL_DESCENT:
        return wp_nav->get_oa_wp_destination(destination);
    case SubMode::LAND:
        return false;
    }

    // we should never get here but just in case
    return false;
}

float ModeRTL::wp_distance_m() const
{
    return wp_nav->get_wp_distance_to_destination_m();
}

float ModeRTL::wp_bearing_deg() const
{
    return degrees(wp_nav->get_wp_bearing_to_destination_rad());
}

// returns true if pilot's yaw input should be used to adjust vehicle's heading
bool ModeRTL::use_pilot_yaw(void) const
{
    // use land mode setting during descent
    if (_state == SubMode::FINAL_DESCENT || _state == SubMode::LAND) {
        return copter.mode_land.use_pilot_yaw();
    }
    return !option_is_enabled(Option::IgnorePilotYaw);
}

bool ModeRTL::set_speed_NE_ms(float speed_ne_ms)
{
    copter.wp_nav->set_speed_NE_ms(speed_ne_ms);
    return true;
}

bool ModeRTL::set_speed_up_ms(float speed_up_ms)
{
    copter.wp_nav->set_speed_up_ms(speed_up_ms);
    return true;
}

bool ModeRTL::set_speed_down_ms(float speed_down_ms)
{
    copter.wp_nav->set_speed_down_ms(speed_down_ms);
    return true;
}

bool ModeRTL::option_is_enabled(Option option) const
{
    return ((copter.g2.rtl_options & (uint32_t)option) != 0);
}

#endif
