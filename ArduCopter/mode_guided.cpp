#include "Copter.h"

#if MODE_GUIDED_ENABLED

/*
 * Init and run calls for guided flight mode
 */

static Vector3p guided_pos_target_ned_m;        // position target (used by posvel controller only)
static bool guided_is_terrain_alt;              // true if guided_pos_target_ned_m.z should be offset by the terrain altitude
static Vector3f guided_vel_target_ned_ms;       // velocity target (used by pos_vel_accel controller and vel_accel controller)
static Vector3f guided_accel_target_ned_mss;    // acceleration target (used by pos_vel_accel controller vel_accel controller and accel controller)
static uint32_t update_time_ms;                 // system time of last target update to pos_vel_accel, vel_accel or accel controller

struct {
    uint32_t update_time_ms;
    Quaternion attitude_quat;
    Vector3f ang_vel_body;
    float climb_rate_ms;    // climb rate in ms.  Used if use_thrust is false
    float thrust_norm;      // thrust from -1 to 1.  Used if use_thrust is true
    bool use_thrust;
} static guided_angle_state;

static uint8_t guided_geometric_log_counter;

struct Guided_Limit {
    uint32_t timeout_ms;        // timeout (in seconds) from the time that guided is invoked
    float alt_min_m;            // lower altitude limit in m above home (0 = no limit)
    float alt_max_m;            // upper altitude limit in m above home (0 = no limit)
    float horiz_max_m;          // horizontal position limit in m from where guided mode was initiated (0 = no limit)
    uint32_t start_time_ms;     // system time in milliseconds that control was handed to the external computer
    Vector3p start_pos_ned_m;   // start position as an offset from home in m. used for checking horiz_max limit
} static guided_limit;

bool ModeGuided::takeoff_complete;      // true once takeoff has completed (used to trigger retracting of landing gear)

// init - initialise guided controller
bool ModeGuided::init(bool ignore_checks)
{
    // start in velaccel control mode
    velaccel_control_start();
    guided_vel_target_ned_ms.zero();
    guided_accel_target_ned_mss.zero();
    send_notification = false;

    // clear pause state when entering guided mode
    _paused = false;

    return true;
}

// hold_position - bring vehicle to a stop and hold current position
// using velocity/acceleration control with zero velocity and acceleration targets
void ModeGuided::hold_position()
{
    // check we are in velocity and acceleration control mode
    if (guided_mode != SubMode::VelAccel) {
        velaccel_control_start();
    }
    guided_vel_target_ned_ms.zero();
    guided_accel_target_ned_mss.zero();
}

// run - runs the guided controller
// should be called at 100hz or more
void ModeGuided::run()
{
    // run pause control if the vehicle is paused
    if (_paused) {
        pause_control_run();
        return;
    }

    // call the correct auto controller
    switch (guided_mode) {

    case SubMode::TakeOff:
        // run takeoff controller
        takeoff_run();
        break;

    case SubMode::WP:
        // run waypoint controller
        wp_control_run();
        if (send_notification && wp_nav->reached_wp_destination()) {
            send_notification = false;
            gcs().send_mission_item_reached_message(0);
        }
        break;

    case SubMode::Pos:
        // run position controller
        pos_control_run();
        break;

    case SubMode::Accel:
        accel_control_run();
        break;

    case SubMode::VelAccel:
        velaccel_control_run();
        break;

    case SubMode::PosVelAccel:
        posvelaccel_control_run();
        break;

    case SubMode::Angle:
        angle_control_run();
        break;
    }
 }

// returns true if the Guided-mode-option is set (see GUID_OPTIONS)
bool ModeGuided::option_is_enabled(Option option) const
{
    return (copter.g2.guided_options.get() & (uint32_t)option) != 0;
}

bool ModeGuided::allows_arming(AP_Arming::Method method) const
{
    // always allow arming from the ground station or scripting
    if (AP_Arming::method_is_GCS(method) || method == AP_Arming::Method::SCRIPTING) {
        return true;
    }

    // optionally allow arming from the transmitter
    return option_is_enabled(Option::AllowArmingFromTX);
};

#if WEATHERVANE_ENABLED
bool ModeGuided::allows_weathervaning() const
{
    return option_is_enabled(Option::AllowWeatherVaning);
}
#endif

// determine EKF reset handling method based on Guide submode
bool ModeGuided::move_vehicle_on_ekf_reset() const
{
        // call the correct auto controller
    switch (guided_mode) {
    case SubMode::TakeOff:
    case SubMode::Accel:
    case SubMode::VelAccel:
    case SubMode::Angle:
        // these submodes have no absolute position target so we reset the position target
        return false;
    case SubMode::WP:
    case SubMode::Pos:
    case SubMode::PosVelAccel:
        // these submodes have absolute position targets so we smoothly slew the target upon an ekf reset
        return true;
    }

    // should never reach here but just in case
    return true;
}

// initialises position controller to implement take-off
// takeoff_alt_m is interpreted as alt-above-home (in m) or alt-above-terrain if a rangefinder is available
bool ModeGuided::do_user_takeoff_start_m(float takeoff_alt_m)
{
    // calculate target altitude and frame (either alt-above-ekf-origin or alt-above-terrain)
    float alt_target_m;
    bool alt_target_terrain = false;
#if AP_RANGEFINDER_ENABLED
    if (wp_nav->rangefinder_used_and_healthy() &&
        wp_nav->get_terrain_source() == AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER &&
        takeoff_alt_m < copter.rangefinder.max_distance_orient(ROTATION_PITCH_270)) {
        // can't takeoff downwards
        if (takeoff_alt_m <= copter.rangefinder_state.alt_m) {
            return false;
        }
        // provide target altitude as alt-above-terrain
        alt_target_m = takeoff_alt_m;
        alt_target_terrain = true;
    } else
#endif
    {
        // interpret altitude as alt-above-home
        Location target_loc = copter.current_loc;
        target_loc.set_alt_m(takeoff_alt_m, Location::AltFrame::ABOVE_HOME);

        // provide target altitude as alt-above-ekf-origin
        if (!target_loc.get_alt_m(Location::AltFrame::ABOVE_ORIGIN, alt_target_m)) {
            // this should never happen but we reject the command just in case
            return false;
        }
    }

    guided_mode = SubMode::TakeOff;

    // initialise yaw
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

    // clear i term when we're taking off
    pos_control->D_init_controller();

    // initialise alt for WP_NAVALT_MIN and set completion alt
    auto_takeoff.start_m(alt_target_m, alt_target_terrain);

    // record takeoff has not completed
    takeoff_complete = false;

    return true;
}

// initialise guided mode's waypoint navigation controller
void ModeGuided::wp_control_start()
{
    // set to position control mode
    guided_mode = SubMode::WP;

    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init_m();

    // initialise wpnav to stopping point
    Vector3p stopping_point_ned_m;
    wp_nav->get_wp_stopping_point_NED_m(stopping_point_ned_m);
    if (!wp_nav->set_wp_destination_NED_m(stopping_point_ned_m, false)) {
        // this should never happen because terrain data is not used
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    // initialise yaw
    auto_yaw.set_mode_to_default(false);
}

// run guided mode's waypoint navigation controller
void ModeGuided::wp_control_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->D_update_controller();

    // call attitude controller with auto yaw
    const AC_AttitudeControl::HeadingCommand heading = auto_yaw.get_heading();
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), heading);
    update_geometric_position_observer(guided_is_terrain_alt ? nullptr : &guided_pos_target_ned_m,
                                       guided_vel_target_ned_ms,
                                       guided_accel_target_ned_mss,
                                       heading);
}

// initialise position controller
void ModeGuided::pva_control_start()
{
    // initialise horizontal speed, acceleration
    pos_control->NE_set_max_speed_accel_m(wp_nav->get_default_speed_NE_ms(), wp_nav->get_wp_acceleration_mss());
    pos_control->NE_set_correction_speed_accel_m(wp_nav->get_default_speed_NE_ms(), wp_nav->get_wp_acceleration_mss());

    // initialize vertical speeds and acceleration
    pos_control->D_set_max_speed_accel_m(wp_nav->get_default_speed_down_ms(), wp_nav->get_default_speed_up_ms(), wp_nav->get_accel_D_mss());
    pos_control->D_set_correction_speed_accel_m(wp_nav->get_default_speed_down_ms(), wp_nav->get_default_speed_up_ms(), wp_nav->get_accel_D_mss());

    // initialise velocity controller
    pos_control->D_init_controller();
    pos_control->NE_init_controller();

    // initialise yaw
    auto_yaw.set_mode_to_default(false);

    // initialise terrain alt
    guided_is_terrain_alt = false;
}

// initialise guided mode's position controller
void ModeGuided::pos_control_start()
{
    // set to position control mode
    guided_mode = SubMode::Pos;

    // initialise position controller
    pva_control_start();
}

// initialise guided mode's acceleration controller
void ModeGuided::accel_control_start()
{
    // set guided_mode to acceleration controller
    guided_mode = SubMode::Accel;

    // initialise position controller
    pva_control_start();
}

// initialise guided mode's velocity and acceleration controller
void ModeGuided::velaccel_control_start()
{
    // set guided_mode to velocity and acceleration controller
    guided_mode = SubMode::VelAccel;

    // initialise position controller
    pva_control_start();
}

// initialise guided mode's position, velocity and acceleration controller
void ModeGuided::posvelaccel_control_start()
{
    // set guided_mode to position, velocity and acceleration controller
    guided_mode = SubMode::PosVelAccel;

    // initialise position controller
    pva_control_start();
}

bool ModeGuided::is_taking_off() const
{
    return guided_mode == SubMode::TakeOff && !takeoff_complete;
}

bool ModeGuided::set_speed_NE_ms(float speed_ne_ms)
{
    // initialise horizontal speed, acceleration
    pos_control->NE_set_max_speed_accel_m(speed_ne_ms, wp_nav->get_wp_acceleration_mss());
    pos_control->NE_set_correction_speed_accel_m(speed_ne_ms, wp_nav->get_wp_acceleration_mss());
    return true;
}

bool ModeGuided::set_speed_up_ms(float speed_up_ms)
{
    // initialize vertical speeds and acceleration
    pos_control->D_set_max_speed_accel_m(wp_nav->get_default_speed_down_ms(), speed_up_ms, wp_nav->get_accel_D_mss());
    pos_control->D_set_correction_speed_accel_m(wp_nav->get_default_speed_down_ms(), speed_up_ms, wp_nav->get_accel_D_mss());
    return true;
}

bool ModeGuided::set_speed_down_ms(float speed_down_ms)
{
    // initialize vertical speeds and acceleration
    pos_control->D_set_max_speed_accel_m(speed_down_ms, wp_nav->get_default_speed_up_ms(), wp_nav->get_accel_D_mss());
    pos_control->D_set_correction_speed_accel_m(speed_down_ms, wp_nav->get_default_speed_up_ms(), wp_nav->get_accel_D_mss());
    return true;
}

// initialise guided mode's angle controller
void ModeGuided::angle_control_start()
{
    // set guided_mode to velocity controller
    guided_mode = SubMode::Angle;

    // set vertical speed and acceleration limits
    pos_control->D_set_max_speed_accel_m(wp_nav->get_default_speed_down_ms(), wp_nav->get_default_speed_up_ms(), wp_nav->get_accel_D_mss());
    pos_control->D_set_correction_speed_accel_m(wp_nav->get_default_speed_down_ms(), wp_nav->get_default_speed_up_ms(), wp_nav->get_accel_D_mss());

    // initialise the vertical position controller
    if (!pos_control->D_is_active()) {
        pos_control->D_init_controller();
    }

    // initialise targets
    guided_angle_state.update_time_ms = millis();
    guided_angle_state.attitude_quat.from_euler(Vector3f{0.0, 0.0, attitude_control->get_att_target_euler_rad().z});
    guided_angle_state.ang_vel_body.zero();
    guided_angle_state.climb_rate_ms = 0.0f;
}

// set_pos_ned_m - sets guided mode's target pos_ned_m
// Returns true if the fence is enabled and guided waypoint is within the fence
// else return false if the waypoint is outside the fence
bool ModeGuided::set_pos_NED_m(const Vector3p& pos_ned_m, bool use_yaw, float yaw_rad, bool use_yaw_rate, float yaw_rate_rads, bool relative_yaw, bool is_terrain_alt)
{
#if AP_FENCE_ENABLED
    // reject destination if outside the fence
    const Location dest_loc = Location::from_ekf_offset_NED_m(pos_ned_m, is_terrain_alt ? Location::AltFrame::ABOVE_TERRAIN : Location::AltFrame::ABOVE_ORIGIN);
    if (!copter.fence.check_location_within_fence(dest_loc)) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // if configured to use wpnav for position control
    if (use_wpnav_for_position_control()) {
        // ensure we are in position control mode
        if (guided_mode != SubMode::WP) {
            wp_control_start();
        }

        // set yaw state
        set_yaw_state_rad(use_yaw, yaw_rad, use_yaw_rate, yaw_rate_rads, relative_yaw);

        // no need to check return status because terrain data is not used
        wp_nav->set_wp_destination_NED_m(pos_ned_m, is_terrain_alt);

#if HAL_LOGGING_ENABLED
        // log target
        copter.Log_Write_Guided_Position_Target(guided_mode, pos_ned_m, is_terrain_alt, Vector3f(), Vector3f());
#endif
        send_notification = true;
        return true;
    }

    // if configured to use position controller for position control
    // ensure we are in position control mode
    if (guided_mode != SubMode::Pos) {
        pos_control_start();
    }

    // initialise terrain following if needed
    if (is_terrain_alt) {
        // get current alt above terrain
        float terrain_d_m;
        if (!wp_nav->get_terrain_D_m(terrain_d_m)) {
            // if we don't have terrain altitude then stop
            hold_position();
            return false;
        }
        // convert origin to alt-above-terrain if necessary
        if (!guided_is_terrain_alt) {
            // new destination is alt-above-terrain, previous destination was alt-above-ekf-origin
            pos_control->init_pos_terrain_D_m(terrain_d_m);
        }
    } else {
        pos_control->init_pos_terrain_D_m(0.0);
    }

    // set yaw state
    set_yaw_state_rad(use_yaw, yaw_rad, use_yaw_rate, yaw_rate_rads, relative_yaw);

    // set position target and zero velocity and acceleration
    guided_pos_target_ned_m = pos_ned_m;
    guided_is_terrain_alt = is_terrain_alt;
    guided_vel_target_ned_ms.zero();
    guided_accel_target_ned_mss.zero();
    update_time_ms = millis();

#if HAL_LOGGING_ENABLED
    // log target
    copter.Log_Write_Guided_Position_Target(guided_mode, guided_pos_target_ned_m, guided_is_terrain_alt, guided_vel_target_ned_ms, guided_accel_target_ned_mss);
#endif

    send_notification = true;

    return true;
}

bool ModeGuided::get_wp(Location& destination) const
{
    switch (guided_mode) {
    case SubMode::WP:
        return wp_nav->get_oa_wp_destination(destination);
    case SubMode::Pos:
        destination = Location::from_ekf_offset_NED_m(guided_pos_target_ned_m, guided_is_terrain_alt ? Location::AltFrame::ABOVE_TERRAIN : Location::AltFrame::ABOVE_ORIGIN);
        return true;
    case SubMode::Angle:
    case SubMode::TakeOff:
    case SubMode::Accel:
    case SubMode::VelAccel:
    case SubMode::PosVelAccel:
        break;
    }

    return false;
}

// sets guided mode's target from a Location object
// returns false if destination could not be set (probably caused by missing terrain data)
// or if the fence is enabled and guided waypoint is outside the fence
bool ModeGuided::set_destination(const Location& dest_loc, bool use_yaw, float yaw_rad, bool use_yaw_rate, float yaw_rate_rads, bool relative_yaw)
{
#if AP_FENCE_ENABLED
    // reject destination outside the fence.
    // Note: there is a danger that a target specified as a terrain altitude might not be checked if the conversion to alt-above-home fails
    if (!copter.fence.check_location_within_fence(dest_loc)) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // if using wpnav for position control
    if (use_wpnav_for_position_control()) {
        if (guided_mode != SubMode::WP) {
            wp_control_start();
        }

        if (!wp_nav->set_wp_destination_loc(dest_loc)) {
            // failure to set destination can only be because of missing terrain data
            LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_SET_DESTINATION);
            // failure is propagated to GCS with NAK
            return false;
        }

        // set yaw state
        set_yaw_state_rad(use_yaw, yaw_rad, use_yaw_rate, yaw_rate_rads, relative_yaw);

#if HAL_LOGGING_ENABLED
        // log target
        copter.Log_Write_Guided_Position_Target(guided_mode, Vector3p(dest_loc.lat, dest_loc.lng, dest_loc.alt), (dest_loc.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN), Vector3f(), Vector3f());
#endif

        send_notification = true;
        return true;
    }

    // set position target and zero velocity and acceleration
    Vector3p pos_target_ned_m;
    bool is_terrain_alt;
    if (!wp_nav->get_vector_NED_m(dest_loc, pos_target_ned_m, is_terrain_alt)) {
        return false;
    }

    // if configured to use position controller for position control
    // ensure we are in position control mode
    if (guided_mode != SubMode::Pos) {
        pos_control_start();
    }

    // set yaw state
    set_yaw_state_rad(use_yaw, yaw_rad, use_yaw_rate, yaw_rate_rads, relative_yaw);

    // initialise terrain following if needed
    if (is_terrain_alt) {
        // get current alt above terrain
        float terrain_d_m;
        if (!wp_nav->get_terrain_D_m(terrain_d_m)) {
            // if we don't have terrain altitude then stop
            hold_position();
            return false;
        }
        // convert origin to alt-above-terrain if necessary
        if (!guided_is_terrain_alt) {
            // new destination is alt-above-terrain, previous destination was alt-above-ekf-origin
            pos_control->init_pos_terrain_D_m(terrain_d_m);
        }
    } else {
        pos_control->init_pos_terrain_D_m(0.0);
    }

    guided_pos_target_ned_m = pos_target_ned_m;
    guided_is_terrain_alt = is_terrain_alt;
    guided_vel_target_ned_ms.zero();
    guided_accel_target_ned_mss.zero();
    update_time_ms = millis();

    // log target
#if HAL_LOGGING_ENABLED
    copter.Log_Write_Guided_Position_Target(guided_mode, Vector3p(dest_loc.lat, dest_loc.lng, dest_loc.alt), guided_is_terrain_alt, guided_vel_target_ned_ms, guided_accel_target_ned_mss);
#endif

    send_notification = true;

    return true;
}

// set_vel_accel_NED_m - sets guided mode's target velocity and acceleration
void ModeGuided::set_accel_NED_mss(const Vector3f& accel_ned_mss, bool use_yaw, float yaw_rad, bool use_yaw_rate, float yaw_rate_rads, bool relative_yaw, bool log_request)
{
    // check we are in acceleration control mode
    if (guided_mode != SubMode::Accel) {
        accel_control_start();
    }

    // set yaw state
    set_yaw_state_rad(use_yaw, yaw_rad, use_yaw_rate, yaw_rate_rads, relative_yaw);

    // set velocity and acceleration targets and zero position
    guided_pos_target_ned_m.zero();
    guided_is_terrain_alt = false;
    guided_vel_target_ned_ms.zero();
    guided_accel_target_ned_mss = accel_ned_mss;
    update_time_ms = millis();

#if HAL_LOGGING_ENABLED
    // log target
    if (log_request) {
        copter.Log_Write_Guided_Position_Target(guided_mode, guided_pos_target_ned_m, guided_is_terrain_alt, guided_vel_target_ned_ms, guided_accel_target_ned_mss);
    }
#endif
}

// set_vel_NED_ms - sets guided mode's target velocity
void ModeGuided::set_vel_NED_ms(const Vector3f& vel_ned_ms, bool use_yaw, float yaw_rad, bool use_yaw_rate, float yaw_rate_rads, bool relative_yaw, bool log_request)
{
    set_vel_accel_NED_m(vel_ned_ms, Vector3f(), use_yaw, yaw_rad, use_yaw_rate, yaw_rate_rads, relative_yaw, log_request);
}

// set_vel_accel_NED_m - sets guided mode's target velocity and acceleration
void ModeGuided::set_vel_accel_NED_m(const Vector3f& vel_ned_ms, const Vector3f& accel_ned_mss, bool use_yaw, float yaw_rad, bool use_yaw_rate, float yaw_rate_rads, bool relative_yaw, bool log_request)
{
    // check we are in velocity and acceleration control mode
    if (guided_mode != SubMode::VelAccel) {
        velaccel_control_start();
    }

    // set yaw state
    set_yaw_state_rad(use_yaw, yaw_rad, use_yaw_rate, yaw_rate_rads, relative_yaw);

    // set velocity and acceleration targets and zero position
    guided_pos_target_ned_m.zero();
    guided_is_terrain_alt = false;
    guided_vel_target_ned_ms = vel_ned_ms;
    guided_accel_target_ned_mss = accel_ned_mss;
    update_time_ms = millis();

#if HAL_LOGGING_ENABLED
    // log target
    if (log_request) {
        copter.Log_Write_Guided_Position_Target(guided_mode, guided_pos_target_ned_m, guided_is_terrain_alt, guided_vel_target_ned_ms, guided_accel_target_ned_mss);
    }
#endif
}

// set guided mode position and velocity target
bool ModeGuided::set_pos_vel_NED_m(const Vector3p& pos_ned_m, const Vector3f& vel_ned_ms, bool use_yaw, float yaw_rad, bool use_yaw_rate, float yaw_rate_rads, bool relative_yaw)
{
    return set_pos_vel_accel_NED_m(pos_ned_m, vel_ned_ms, Vector3f(), use_yaw, yaw_rad, use_yaw_rate, yaw_rate_rads, relative_yaw);
}

// set_pos_vel_accel_NED_m - set guided mode position, velocity and acceleration target
bool ModeGuided::set_pos_vel_accel_NED_m(const Vector3p& pos_ned_m, const Vector3f& vel_ned_ms, const Vector3f& accel_ned_mss, bool use_yaw, float yaw_rad, bool use_yaw_rate, float yaw_rate_rads, bool relative_yaw)
{
#if AP_FENCE_ENABLED
    // reject destination if outside the fence
    const Location dest_loc = Location::from_ekf_offset_NED_m(pos_ned_m, Location::AltFrame::ABOVE_ORIGIN);
    if (!copter.fence.check_location_within_fence(dest_loc)) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // check we are in position, velocity and acceleration control mode
    if (guided_mode != SubMode::PosVelAccel) {
        posvelaccel_control_start();
    }

    // set yaw state
    set_yaw_state_rad(use_yaw, yaw_rad, use_yaw_rate, yaw_rate_rads, relative_yaw);

    update_time_ms = millis();
    guided_pos_target_ned_m = pos_ned_m;
    guided_is_terrain_alt = false;
    guided_vel_target_ned_ms = vel_ned_ms;
    guided_accel_target_ned_mss = accel_ned_mss;

#if HAL_LOGGING_ENABLED
    // log target
    copter.Log_Write_Guided_Position_Target(guided_mode, guided_pos_target_ned_m, guided_is_terrain_alt, guided_vel_target_ned_ms, guided_accel_target_ned_mss);
#endif
    return true;
}

// returns true if GUIDED_OPTIONS param suggests SET_ATTITUDE_TARGET's "thrust" field should be interpreted as thrust instead of climb rate
bool ModeGuided::set_attitude_target_provides_thrust() const
{
    return option_is_enabled(Option::SetAttitudeTarget_ThrustAsThrust);
}

// returns true if GUIDED_OPTIONS param specifies position should be controlled (when velocity and/or acceleration control is active)
bool ModeGuided::stabilizing_pos_NE() const
{
    return !option_is_enabled(Option::DoNotStabilizePositionXY);
}

// returns true if GUIDED_OPTIONS param specifies velocity should  be controlled (when acceleration control is active)
bool ModeGuided::stabilizing_vel_NE() const
{
    return !option_is_enabled(Option::DoNotStabilizeVelocityXY);
}

// returns true if GUIDED_OPTIONS param specifies waypoint navigation should be used for position control (allow path planning to be used but updates must be slower)
bool ModeGuided::use_wpnav_for_position_control() const
{
    return option_is_enabled(Option::WPNavUsedForPosControl);
}

// Sets guided's angular target submode: Using a rotation quaternion, angular velocity, and climbrate or thrust (depends on user option)
// attitude_quat: IF zero: ang_vel_body (body frame angular velocity) must be provided even if all zeroes
//                IF non-zero: attitude_control is performed using both the attitude quaternion and body frame angular velocity
// ang_vel_body: body frame angular velocity (rad/s)
// climb_rate_ms_or_thrust: represents either the climb_rate (m/s) or thrust scaled from [0, 1], unitless
// use_thrust: IF true: climb_rate_ms_or_thrust represents thrust
//             IF false: climb_rate_ms_or_thrust represents climb_rate (m/s)
void ModeGuided::set_angle(const Quaternion &attitude_quat, const Vector3f &ang_vel_body, float climb_rate_ms_or_thrust, bool use_thrust)
{
    // check we are in velocity control mode
    if (guided_mode != SubMode::Angle) {
        angle_control_start();
    } else if (!use_thrust && guided_angle_state.use_thrust) {
        // Already angle control but changing from thrust to climb rate
        pos_control->D_init_controller();
    }

    guided_angle_state.attitude_quat = attitude_quat;
    guided_angle_state.ang_vel_body = ang_vel_body;

    guided_angle_state.use_thrust = use_thrust;
    if (use_thrust) {
        guided_angle_state.thrust_norm = climb_rate_ms_or_thrust;
        guided_angle_state.climb_rate_ms = 0.0f;
    } else {
        guided_angle_state.thrust_norm = 0.0f;
        guided_angle_state.climb_rate_ms = climb_rate_ms_or_thrust;
    }

    guided_angle_state.update_time_ms = millis();

    // convert quaternion to euler angles
    float roll_rad, pitch_rad, yaw_rad;
    attitude_quat.to_euler(roll_rad, pitch_rad, yaw_rad);

#if HAL_LOGGING_ENABLED
    // log target
    copter.Log_Write_Guided_Attitude_Target(guided_mode, roll_rad, pitch_rad, yaw_rad, ang_vel_body, guided_angle_state.thrust_norm, guided_angle_state.climb_rate_ms);
#endif
}

// takeoff_run - takeoff in guided mode
//      called by guided_run at 100hz or more
void ModeGuided::takeoff_run()
{
    auto_takeoff.run();
    if (auto_takeoff.complete && !takeoff_complete) {
        takeoff_complete = true;
#if AP_FENCE_ENABLED
        copter.fence.auto_enable_fence_after_takeoff();
#endif
#if AP_LANDINGGEAR_ENABLED
        // optionally retract landing gear
        copter.landinggear.retract_after_takeoff();
#endif
    }
}

// pos_control_run - runs the guided position controller
// called from guided_run
void ModeGuided::pos_control_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // calculate terrain adjustments
    float terrain_d_m = 0.0f;
    if (guided_is_terrain_alt && !wp_nav->get_terrain_D_m(terrain_d_m)) {
        // failure to set destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // send position and velocity targets to position controller
    guided_accel_target_ned_mss.zero();
    guided_vel_target_ned_ms.zero();

    // stop rotating if no updates received within timeout_ms
    if (millis() - update_time_ms > get_timeout_ms()) {
        if ((auto_yaw.mode() == AutoYaw::Mode::RATE) || (auto_yaw.mode() == AutoYaw::Mode::ANGLE_RATE)) {
            auto_yaw.set_mode(AutoYaw::Mode::HOLD);
        }
    }

    float terrain_margin_m = 0.0; // Vertical buffer size in m
    if (guided_is_terrain_alt) {
        terrain_margin_m = MIN(copter.wp_nav->get_terrain_margin_m(), 0.5 * fabsF(guided_pos_target_ned_m.z));
    }
    pos_control->input_pos_NED_m(guided_pos_target_ned_m, terrain_d_m, terrain_margin_m);

    // run position controllers
    pos_control->NE_update_controller();
    pos_control->D_update_controller();

    // call attitude controller with auto yaw
    const AC_AttitudeControl::HeadingCommand heading = auto_yaw.get_heading();
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), heading);
    update_geometric_position_observer(guided_is_terrain_alt ? nullptr : &guided_pos_target_ned_m,
                                       guided_vel_target_ned_ms,
                                       guided_accel_target_ned_mss,
                                       heading);
}

// velaccel_control_run - runs the guided velocity controller
// called from guided_run
void ModeGuided::accel_control_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // set velocity to zero and stop rotating if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - update_time_ms > get_timeout_ms()) {
        guided_vel_target_ned_ms.zero();
        guided_accel_target_ned_mss.zero();
        if ((auto_yaw.mode() == AutoYaw::Mode::RATE) || (auto_yaw.mode() == AutoYaw::Mode::ANGLE_RATE)) {
            auto_yaw.set_mode(AutoYaw::Mode::HOLD);
        }
        pos_control->input_vel_accel_NE_m(guided_vel_target_ned_ms.xy(), guided_accel_target_ned_mss.xy(), false);
        pos_control->input_vel_accel_D_m(guided_vel_target_ned_ms.z, guided_accel_target_ned_mss.z, false);
    } else {
        // update position controller with new target
        pos_control->input_accel_NE_m(guided_accel_target_ned_mss.xy());
        if (!stabilizing_vel_NE()) {
            // set position and velocity errors to zero
            pos_control->NE_stop_vel_stabilisation();
        } else if (!stabilizing_pos_NE()) {
            // set position errors to zero
            pos_control->NE_stop_pos_stabilisation();
        }
        pos_control->input_accel_D_m(guided_accel_target_ned_mss.z);
    }

    // call velocity controller which includes z axis controller
    pos_control->NE_update_controller();
    pos_control->D_update_controller();

    // call attitude controller with auto yaw
    const AC_AttitudeControl::HeadingCommand heading = auto_yaw.get_heading();
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), heading);
    update_geometric_position_observer(nullptr,
                                       pos_control->get_vel_estimate_NED_ms(),
                                       guided_accel_target_ned_mss,
                                       heading);
}

// velaccel_control_run - runs the guided velocity and acceleration controller
// called from guided_run
void ModeGuided::velaccel_control_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // set velocity to zero and stop rotating if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - update_time_ms > get_timeout_ms()) {
        guided_vel_target_ned_ms.zero();
        guided_accel_target_ned_mss.zero();
        if ((auto_yaw.mode() == AutoYaw::Mode::RATE) || (auto_yaw.mode() == AutoYaw::Mode::ANGLE_RATE)) {
            auto_yaw.set_mode(AutoYaw::Mode::HOLD);
        }
    }

    bool do_avoid = false;
#if AP_AVOIDANCE_ENABLED
    // limit the velocity for obstacle/fence avoidance
    copter.avoid.adjust_velocity_NED_m(guided_vel_target_ned_ms, pos_control->NE_get_pos_p().kP(), pos_control->NE_get_max_accel_mss(), pos_control->D_get_pos_p().kP(), pos_control->D_get_max_accel_mss(), G_Dt);
    do_avoid = copter.avoid.limits_active();
#endif

    // update position controller with new target

    if (!stabilizing_vel_NE() && !do_avoid) {
        // set the current commanded xy vel to the desired vel
        guided_vel_target_ned_ms.xy() = pos_control->get_vel_desired_NED_ms().xy();
    }
    pos_control->input_vel_accel_NE_m(guided_vel_target_ned_ms.xy(), guided_accel_target_ned_mss.xy(), false);
    if (!stabilizing_vel_NE() && !do_avoid) {
        // set position and velocity errors to zero
        pos_control->NE_stop_vel_stabilisation();
    } else if (!stabilizing_pos_NE() && !do_avoid) {
        // set position errors to zero
        pos_control->NE_stop_pos_stabilisation();
    }
    pos_control->input_vel_accel_D_m(guided_vel_target_ned_ms.z, guided_accel_target_ned_mss.z, false);

    // call velocity controller which includes z axis controller
    pos_control->NE_update_controller();
    pos_control->D_update_controller();

    // call attitude controller with auto yaw
    const AC_AttitudeControl::HeadingCommand heading = auto_yaw.get_heading();
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), heading);
    update_geometric_position_observer(nullptr,
                                       guided_vel_target_ned_ms,
                                       guided_accel_target_ned_mss,
                                       heading);
}

// pause_control_run - runs the guided mode pause controller
// called from guided_run
void ModeGuided::pause_control_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // set the horizontal velocity and acceleration targets to zero
    Vector2f vel_xy_zero, accel_xy_zero;
    pos_control->input_vel_accel_NE_m(vel_xy_zero, accel_xy_zero, false);

    // set the vertical velocity and acceleration targets to zero
    float vel_d_zero = 0.0;
    pos_control->input_vel_accel_D_m(vel_d_zero, 0.0, false);

    // call velocity controller which includes z axis controller
    pos_control->NE_update_controller();
    pos_control->D_update_controller();

    // call attitude controller
    attitude_control->input_thrust_vector_rate_heading_rads(pos_control->get_thrust_vector(), 0.0);
}

// posvelaccel_control_run - runs the guided position, velocity and acceleration controller
// called from guided_run
void ModeGuided::posvelaccel_control_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // set velocity to zero and stop rotating if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - update_time_ms > get_timeout_ms()) {
        guided_vel_target_ned_ms.zero();
        guided_accel_target_ned_mss.zero();
        if ((auto_yaw.mode() == AutoYaw::Mode::RATE) || (auto_yaw.mode() == AutoYaw::Mode::ANGLE_RATE)) {
            auto_yaw.set_mode(AutoYaw::Mode::HOLD);
        }
    }

    // send position and velocity targets to position controller
    if (!stabilizing_vel_NE()) {
        // set the current commanded xy pos to the target pos and xy vel to the desired vel
        guided_pos_target_ned_m.xy() = pos_control->get_pos_desired_NED_m().xy();
        guided_vel_target_ned_ms.xy() = pos_control->get_vel_desired_NED_ms().xy();
    } else if (!stabilizing_pos_NE()) {
        // set the current commanded xy pos to the target pos
        guided_pos_target_ned_m.xy() = pos_control->get_pos_desired_NED_m().xy();
    }
    pos_control->input_pos_vel_accel_NE_m(guided_pos_target_ned_m.xy(), guided_vel_target_ned_ms.xy(), guided_accel_target_ned_mss.xy(), false);
    if (!stabilizing_vel_NE()) {
        // set position and velocity errors to zero
        pos_control->NE_stop_vel_stabilisation();
    } else if (!stabilizing_pos_NE()) {
        // set position errors to zero
        pos_control->NE_stop_pos_stabilisation();
    }

    // guided_pos_target z-axis should never be a terrain altitude
    if (guided_is_terrain_alt) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    float pz = guided_pos_target_ned_m.z;
    pos_control->input_pos_vel_accel_D_m(pz, guided_vel_target_ned_ms.z, guided_accel_target_ned_mss.z, false);
    guided_pos_target_ned_m.z = pz;

    // run position controllers
    pos_control->NE_update_controller();
    pos_control->D_update_controller();

    // call attitude controller with auto yaw
    const AC_AttitudeControl::HeadingCommand heading = auto_yaw.get_heading();
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), heading);
    update_geometric_position_observer(&guided_pos_target_ned_m,
                                       guided_vel_target_ned_ms,
                                       guided_accel_target_ned_mss,
                                       heading);
}

// angle_control_run - runs the guided angle controller
// called from guided_run
void ModeGuided::angle_control_run()
{
    float climb_rate_ms = 0.0f;
    if (!guided_angle_state.use_thrust) {
        // constrain climb rate
        climb_rate_ms = constrain_float(guided_angle_state.climb_rate_ms, -wp_nav->get_default_speed_down_ms(), wp_nav->get_default_speed_up_ms());

        // get avoidance adjusted climb rate
        climb_rate_ms = get_avoidance_adjusted_climbrate_ms(climb_rate_ms);
    }

    // check for timeout - set lean angles and climb rate to zero if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - guided_angle_state.update_time_ms > get_timeout_ms()) {
        guided_angle_state.attitude_quat.from_euler(Vector3f{0.0, 0.0, attitude_control->get_att_target_euler_rad().z});
        guided_angle_state.ang_vel_body.zero();
        climb_rate_ms = 0.0f;
        if (guided_angle_state.use_thrust) {
            // initialise vertical velocity controller
            pos_control->D_init_controller();
            guided_angle_state.use_thrust = false;
        }
    }

    // interpret positive climb rate or thrust as triggering take-off
    const bool positive_thrust_or_climbrate = is_positive(guided_angle_state.use_thrust ? guided_angle_state.thrust_norm : climb_rate_ms);
    if (motors->armed() && positive_thrust_or_climbrate) {
        copter.set_auto_armed(true);
    }

    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || !copter.ap.auto_armed || (copter.ap.land_complete && !positive_thrust_or_climbrate)) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // TODO: use get_alt_hold_state_D_ms
    // landed with positive desired climb rate or thrust, takeoff
    if (copter.ap.land_complete && positive_thrust_or_climbrate) {
        zero_throttle_and_relax_ac();
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        if (motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
            set_land_complete(false);
            pos_control->D_init_controller();
        }
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // call attitude controller
    if (guided_angle_state.attitude_quat.is_zero()) {
        attitude_control->input_rate_bf_roll_pitch_yaw_rads(guided_angle_state.ang_vel_body.x, guided_angle_state.ang_vel_body.y, guided_angle_state.ang_vel_body.z);
    } else {
        attitude_control->input_quaternion(guided_angle_state.attitude_quat, guided_angle_state.ang_vel_body);
    }
    update_geometric_angle_observer();

    // call position controller
    if (guided_angle_state.use_thrust) {
        attitude_control->set_throttle_out(guided_angle_state.thrust_norm, true, copter.g.throttle_filt);
    } else {
        pos_control->D_set_pos_target_from_climb_rate_ms(climb_rate_ms);
        pos_control->D_update_controller();
    }
}

void ModeGuided::update_geometric_observer(const AC_Geometric_Target& target)
{
    const bool enabled = option_is_enabled(Option::GeometricObserver);
    copter.geometric_control.set_enabled(enabled);
    if (!enabled) {
        return;
    }

    AC_Geometric_State geometric_state {};
    const Vector3p& pos_estimate_ned_m = pos_control->get_pos_estimate_NED_m();
    geometric_state.position_ned_m = Vector3f{float(pos_estimate_ned_m.x), float(pos_estimate_ned_m.y), float(pos_estimate_ned_m.z)};
    geometric_state.velocity_ned_ms = pos_control->get_vel_estimate_NED_ms();
    ahrs.get_quat_body_to_ned(geometric_state.attitude_body_to_ned);
    geometric_state.omega_body_rads = ahrs.get_gyro_latest();

    copter.geometric_control.update(geometric_state, target, G_Dt);

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: GEOA
    // @Description: Geometric guided attitude observer
    // @Field: TimeUS: Time since system startup
    // @Field: ERx: Lee attitude error, X-Axis
    // @Field: ERy: Lee attitude error, Y-Axis
    // @Field: ERz: Lee attitude error, Z-Axis
    // @Field: EOx: Angular velocity error, X-Axis
    // @Field: EOy: Angular velocity error, Y-Axis
    // @Field: EOz: Angular velocity error, Z-Axis
    // @Field: Mx: Geometric body-moment proxy, X-Axis
    // @Field: My: Geometric body-moment proxy, Y-Axis
    // @Field: Mz: Geometric body-moment proxy, Z-Axis
    // @Field: RTx: Body-rate target proxy, X-Axis
    // @Field: RTy: Body-rate target proxy, Y-Axis
    // @Field: RTz: Body-rate target proxy, Z-Axis

    // @LoggerMessage: GEOP
    // @Description: Geometric guided position observer
    // @Field: TimeUS: Time since system startup
    // @Field: PEx: Position error, X-Axis
    // @Field: PEy: Position error, Y-Axis
    // @Field: PEz: Position error, Z-Axis
    // @Field: SFx: Specific force command, X-Axis
    // @Field: SFy: Specific force command, Y-Axis
    // @Field: SFz: Specific force command, Z-Axis
    // @Field: Thr: Projected total thrust per mass
    // @Field: RCr: Commanded attitude roll
    // @Field: RCp: Commanded attitude pitch
    // @Field: ATr: ArduPilot attitude target roll
    // @Field: ATp: ArduPilot attitude target pitch
    // @Field: TVx: ArduPilot thrust vector, X-Axis
    // @Field: TVy: ArduPilot thrust vector, Y-Axis
    // @Field: TVz: ArduPilot thrust vector, Z-Axis

    // @LoggerMessage: GEOO
    // @Description: Geometric guided output mapper observer
    // @Field: TimeUS: Time since system startup
    // @Field: TRaw: Raw normalized throttle shadow output
    // @Field: TNorm: Limited normalized throttle shadow output
    // @Field: TLim: True if normalized throttle shadow output was limited
    // @Field: RCr: Shadow attitude roll
    // @Field: RCp: Shadow attitude pitch
    // @Field: RCy: Shadow attitude yaw
    // @Field: RTx: Shadow body-rate target, X-Axis
    // @Field: RTy: Shadow body-rate target, Y-Axis
    // @Field: RTz: Shadow body-rate target, Z-Axis

    // @LoggerMessage: GEOM
    // @Description: Geometric guided motor mapper observer
    // @Field: TimeUS: Time since system startup
    // @Field: RRaw: Raw normalized roll actuator shadow output
    // @Field: PRaw: Raw normalized pitch actuator shadow output
    // @Field: YRaw: Raw normalized yaw actuator shadow output
    // @Field: Roll: Limited normalized roll actuator shadow output
    // @Field: Pitch: Limited normalized pitch actuator shadow output
    // @Field: Yaw: Limited normalized yaw actuator shadow output
    // @Field: Lim: True if any actuator shadow output was limited
    if (guided_geometric_log_counter++ % 5 == 0) {
        const AC_Geometric_Output& output = copter.geometric_control.get_output();
        AP::logger().WriteStreaming("GEOA", "TimeUS,ERx,ERy,ERz,EOx,EOy,EOz,Mx,My,Mz,RTx,RTy,RTz", "Qffffffffffff",
                                    AP_HAL::micros64(),
                                    (double)output.attitude.attitude_error.x,
                                    (double)output.attitude.attitude_error.y,
                                    (double)output.attitude.attitude_error.z,
                                    (double)output.attitude.omega_error_rads.x,
                                    (double)output.attitude.omega_error_rads.y,
                                    (double)output.attitude.omega_error_rads.z,
                                    (double)output.attitude.moment.x,
                                    (double)output.attitude.moment.y,
                                    (double)output.attitude.moment.z,
                                    (double)output.attitude.rate_target_body_rads.x,
                                    (double)output.attitude.rate_target_body_rads.y,
                                    (double)output.attitude.rate_target_body_rads.z);

        float rc_roll_rad;
        float rc_pitch_rad;
        float rc_yaw_rad;
        output.position.attitude_body_to_ned.to_euler(rc_roll_rad, rc_pitch_rad, rc_yaw_rad);
        const Vector3f ap_attitude_target_rad = attitude_control->get_att_target_euler_rad();
        const Vector3f ap_thrust_vector_ned = pos_control->get_thrust_vector();
        AP::logger().WriteStreaming("GEOP", "TimeUS,PEx,PEy,PEz,SFx,SFy,SFz,Thr,RCr,RCp,ATr,ATp,TVx,TVy,TVz", "Qffffffffffffff",
                                    AP_HAL::micros64(),
                                    (double)output.position.position_error_m.x,
                                    (double)output.position.position_error_m.y,
                                    (double)output.position.position_error_m.z,
                                    (double)output.position.specific_force_ned_mss.x,
                                    (double)output.position.specific_force_ned_mss.y,
                                    (double)output.position.specific_force_ned_mss.z,
                                    (double)output.position.thrust,
                                    (double)rc_roll_rad,
                                    (double)rc_pitch_rad,
                                    (double)ap_attitude_target_rad.x,
                                    (double)ap_attitude_target_rad.y,
                                    (double)ap_thrust_vector_ned.x,
                                    (double)ap_thrust_vector_ned.y,
                                    (double)ap_thrust_vector_ned.z);

        float mapped_roll_rad;
        float mapped_pitch_rad;
        float mapped_yaw_rad;
        output.mapped.attitude_body_to_ned.to_euler(mapped_roll_rad, mapped_pitch_rad, mapped_yaw_rad);
        AP::logger().WriteStreaming("GEOO", "TimeUS,TRaw,TNorm,TLim,RCr,RCp,RCy,RTx,RTy,RTz", "QffBffffff",
                                    AP_HAL::micros64(),
                                    (double)output.mapped.throttle_norm_raw,
                                    (double)output.mapped.throttle_norm,
                                    (uint8_t)output.mapped.throttle_limited,
                                    (double)mapped_roll_rad,
                                    (double)mapped_pitch_rad,
                                    (double)mapped_yaw_rad,
                                    (double)output.mapped.rate_target_body_rads.x,
                                    (double)output.mapped.rate_target_body_rads.y,
                                    (double)output.mapped.rate_target_body_rads.z);

        AP::logger().WriteStreaming("GEOM", "TimeUS,RRaw,PRaw,YRaw,Roll,Pitch,Yaw,Lim", "QffffffB",
                                    AP_HAL::micros64(),
                                    (double)output.mapped.rpy_norm_raw.x,
                                    (double)output.mapped.rpy_norm_raw.y,
                                    (double)output.mapped.rpy_norm_raw.z,
                                    (double)output.mapped.rpy_norm.x,
                                    (double)output.mapped.rpy_norm.y,
                                    (double)output.mapped.rpy_norm.z,
                                    (uint8_t)output.mapped.rpy_limited);
    }
#endif
}

void ModeGuided::update_geometric_angle_observer()
{
    AC_Geometric_Target geometric_target {};
    const Vector3p& pos_estimate_ned_m = pos_control->get_pos_estimate_NED_m();
    geometric_target.position_ned_m = Vector3f{float(pos_estimate_ned_m.x), float(pos_estimate_ned_m.y), float(pos_estimate_ned_m.z)};
    geometric_target.velocity_ned_ms = pos_control->get_vel_estimate_NED_ms();
    geometric_target.attitude_body_to_ned = attitude_control->get_attitude_target_quat();
    geometric_target.omega_body_rads = attitude_control->get_attitude_target_ang_vel();
    geometric_target.yaw_rad = attitude_control->get_att_target_euler_rad().z;
    geometric_target.yaw_rate_rads = geometric_target.omega_body_rads.z;

    update_geometric_observer(geometric_target);
}

void ModeGuided::update_geometric_position_observer(const Vector3p* position_target_ned_m,
                                                    const Vector3f& velocity_target_ned_ms,
                                                    const Vector3f& accel_target_ned_mss,
                                                    const AC_AttitudeControl::HeadingCommand& heading)
{
    AC_Geometric_Target geometric_target {};
    if (position_target_ned_m != nullptr) {
        geometric_target.position_ned_m = Vector3f{float(position_target_ned_m->x),
                                                  float(position_target_ned_m->y),
                                                  float(position_target_ned_m->z)};
    } else {
        const Vector3p& pos_estimate_ned_m = pos_control->get_pos_estimate_NED_m();
        geometric_target.position_ned_m = Vector3f{float(pos_estimate_ned_m.x),
                                                  float(pos_estimate_ned_m.y),
                                                  float(pos_estimate_ned_m.z)};
    }
    geometric_target.velocity_ned_ms = velocity_target_ned_ms;
    geometric_target.accel_ned_mss = accel_target_ned_mss;
    geometric_target.build_attitude_from_position = true;
    // Use ArduPilot's shaped yaw target after input_thrust_vector_heading()
    // so the geometric R_c observer is compared against the same yaw state.
    geometric_target.yaw_rad = attitude_control->get_att_target_euler_rad().z;
    geometric_target.yaw_rate_rads = heading.yaw_rate_rads;
    geometric_target.omega_body_rads.z = heading.yaw_rate_rads;

    update_geometric_observer(geometric_target);
}

// helper function to set yaw state and targets
void ModeGuided::set_yaw_state_rad(bool use_yaw, float yaw_rad, bool use_yaw_rate, float yaw_rate_rads, bool relative_angle)
{
    if (use_yaw && relative_angle) {
        auto_yaw.set_fixed_yaw_rad(yaw_rad, 0.0f, 0, relative_angle);
    } else if (use_yaw && use_yaw_rate) {
        auto_yaw.set_yaw_angle_and_rate_rad(yaw_rad, yaw_rate_rads);
    } else if (use_yaw && !use_yaw_rate) {
        auto_yaw.set_yaw_angle_and_rate_rad(yaw_rad, 0.0f);
    } else if (use_yaw_rate) {
        auto_yaw.set_rate_rad(yaw_rate_rads);
    } else {
        auto_yaw.set_mode_to_default(false);
    }
}

// returns true if pilot's yaw input should be used to adjust vehicle's heading
bool ModeGuided::use_pilot_yaw(void) const
{
    return !option_is_enabled(Option::IgnorePilotYaw);
}

// Guided Limit code

// limit_clear - clear/turn off guided limits
void ModeGuided::limit_clear()
{
    guided_limit.timeout_ms = 0;
    guided_limit.alt_min_m = 0.0f;
    guided_limit.alt_max_m = 0.0f;
    guided_limit.horiz_max_m = 0.0f;
}

// limit_set - set guided timeout and movement limits
void ModeGuided::limit_set(uint32_t timeout_ms, float alt_min_m, float alt_max_m, float horiz_max_m)
{
    guided_limit.timeout_ms = timeout_ms;
    guided_limit.alt_min_m = alt_min_m;
    guided_limit.alt_max_m = alt_max_m;
    guided_limit.horiz_max_m = horiz_max_m;
}

// limit_init_time_and_pos - initialise guided start time and position as reference for limit checking
//  only called from AUTO mode's auto_nav_guided_start function
void ModeGuided::limit_init_time_and_pos()
{
    // initialise start time
    guided_limit.start_time_ms = AP_HAL::millis();

    // initialise start position from current position
    guided_limit.start_pos_ned_m = pos_control->get_pos_estimate_NED_m();
}

// limit_check - returns true if guided mode has breached a limit
//  used when guided is invoked from the NAV_GUIDED_ENABLE mission command
bool ModeGuided::limit_check()
{
    // check if we have passed the timeout
    if ((guided_limit.timeout_ms > 0) && (millis() - guided_limit.start_time_ms >= guided_limit.timeout_ms)) {
        return true;
    }

    // get current location
    const Vector3p& curr_pos_ned_m = pos_control->get_pos_estimate_NED_m();

    // check if we have gone below min alt
    if (!is_zero(guided_limit.alt_min_m) && (-curr_pos_ned_m.z < guided_limit.alt_min_m)) {
        return true;
    }

    // check if we have gone above max alt
    if (!is_zero(guided_limit.alt_max_m) && (-curr_pos_ned_m.z > guided_limit.alt_max_m)) {
        return true;
    }

    // check if we have gone beyond horizontal limit
    if (guided_limit.horiz_max_m > 0.0f) {
        const float horiz_offset_m = get_horizontal_distance(guided_limit.start_pos_ned_m.xy(), curr_pos_ned_m.xy());
        if (horiz_offset_m > guided_limit.horiz_max_m) {
            return true;
        }
    }

    // if we got this far we must be within limits
    return false;
}

const Vector3p &ModeGuided::get_target_pos_NED_m() const
{
    return guided_pos_target_ned_m;
}

const Vector3f& ModeGuided::get_target_vel_NED_ms() const
{
    return guided_vel_target_ned_ms;
}

const Vector3f& ModeGuided::get_target_accel_NED_mss() const
{
    return guided_accel_target_ned_mss;
}

float ModeGuided::wp_distance_m() const
{
    switch(guided_mode) {
    case SubMode::WP:
        return wp_nav->get_wp_distance_to_destination_m();
    case SubMode::Pos:
        return get_horizontal_distance(pos_control->get_pos_estimate_NED_m().xy().tofloat(), guided_pos_target_ned_m.xy().tofloat());
    case SubMode::PosVelAccel:
        return pos_control->get_pos_error_NE_m();
    default:
        return 0.0f;
    }
}

float ModeGuided::wp_bearing_deg() const
{
    switch(guided_mode) {
    case SubMode::WP:
        return degrees(wp_nav->get_wp_bearing_to_destination_rad());
    case SubMode::Pos:
        return degrees(get_bearing_rad(pos_control->get_pos_estimate_NED_m().xy().tofloat(), guided_pos_target_ned_m.xy().tofloat()));
    case SubMode::PosVelAccel:
        return degrees(pos_control->get_bearing_to_target_rad());
    case SubMode::TakeOff:
    case SubMode::Accel:
    case SubMode::VelAccel:
    case SubMode::Angle:
        // these do not have bearings
        return 0;
    }
    // compiler guarantees we don't get here
    return 0.0;
}

float ModeGuided::crosstrack_error_m() const
{
    switch (guided_mode) {
    case SubMode::WP:
        return wp_nav->crosstrack_error_m();
    case SubMode::Pos:
    case SubMode::TakeOff:
    case SubMode::Accel:
    case SubMode::VelAccel:
    case SubMode::PosVelAccel:
        return pos_control->crosstrack_error_m();
    case SubMode::Angle:
        // no track to have a crosstrack to
        return 0;
    }
    // compiler guarantees we don't get here
    return 0;
}

// return guided mode timeout in milliseconds. Only used for velocity, acceleration, angle control, and angular rates
uint32_t ModeGuided::get_timeout_ms() const
{
    return MAX(copter.g2.guided_timeout, 0.1) * 1000;
}

// pause guide mode
bool ModeGuided::pause()
{
    _paused = true;
    return true;
}

// resume guided mode
bool ModeGuided::resume()
{
    _paused = false;
    return true;
}

#endif
