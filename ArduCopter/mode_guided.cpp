#include "Copter.h"

#include <AC_GeometricControl/AC_Geometric_GuidedTargetManager.h>

#if MODE_GUIDED_ENABLED

/*
 * Init and run calls for guided flight mode
 */

static Vector3p guided_pos_target_ned_m;        // position target (used by posvel controller only)
static bool guided_is_terrain_alt;              // true if guided_pos_target_ned_m.z should be offset by the terrain altitude
static Vector3f guided_vel_target_ned_ms;       // velocity target (used by pos_vel_accel controller and vel_accel controller)
static Vector3f guided_accel_target_ned_mss;    // acceleration target (used by pos_vel_accel controller vel_accel controller and accel controller)
static uint32_t update_time_ms;                 // system time of last target update to pos_vel_accel, vel_accel or accel controller
static bool guided_geometric_position_was_active;
static AC_Geometric_GuidedTargetManager guided_geometric_target_manager;
static Vector3p guided_geometric_takeoff_target_ned_m;
static bool guided_geometric_takeoff_terrain_alt;
static bool guided_geometric_takeoff_spool_ready;
static Vector3p guided_geometric_land_hold_ned_m;
static float guided_geometric_land_yaw_rad;
static float guided_geometric_land_descent_ned_ms;
static Vector3p guided_pause_pos_ned_m;
static bool guided_pause_pos_valid;
static float guided_pause_yaw_rad;
static bool guided_pause_yaw_valid;
static uint8_t guided_geometric_heading_mode;
static bool guided_geometric_trajectory_yaw_allowed;

struct {
    uint32_t update_time_ms;
    Quaternion attitude_quat;
    Vector3f ang_vel_body;
    float climb_rate_ms;    // climb rate in ms.  Used if use_thrust is false
    float thrust_norm;      // thrust from -1 to 1.  Used if use_thrust is true
    bool use_thrust;
} static guided_angle_state;

static uint8_t guided_geometric_log_counter;
static constexpr uint32_t guided_geometric_output_recent_ms = 100;

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
    // Do not inherit another mode's external target ownership.  Supported
    // full-geometric entry publishes a fresh reference synchronously below.
    pos_control->clear_external_reference();

    // Full-geometric Guided starts in a supported current-state PVA hold.
    // This lets the disarmed mode continuously precompute a zero-collective
    // command so the first armed motor frame is geometric.  Native Guided
    // keeps its established VelAccel entry semantics.
    const bool full_geometric_entry = geometric_motor_output_options_requested();
    if (full_geometric_entry) {
        posvelaccel_control_start();
    } else {
        velaccel_control_start();
    }
    const Vector3p current_pos_ned_m = pos_control->get_pos_estimate_NED_m();
    const Vector3f current_vel_ned_ms = pos_control->get_vel_estimate_NED_ms();
    guided_pos_target_ned_m = current_pos_ned_m;
    guided_vel_target_ned_ms = full_geometric_entry ? current_vel_ned_ms : Vector3f{};
    guided_accel_target_ned_mss.zero();
    guided_is_terrain_alt = false;
    update_time_ms = millis();
    send_notification = false;

    // clear pause state when entering guided mode
    _paused = false;
    guided_pause_pos_valid = false;
    guided_pause_yaw_valid = false;
    guided_geometric_position_was_active = false;
    guided_geometric_target_manager.reset();
    guided_geometric_takeoff_target_ned_m.zero();
    guided_geometric_takeoff_terrain_alt = false;
    guided_geometric_takeoff_spool_ready = false;
    guided_geometric_land_hold_ned_m.zero();
    guided_geometric_land_yaw_rad = 0.0f;
    guided_geometric_land_descent_ned_ms = 0.0f;
    _geometric_motor_output_rejected = false;
    _geometric_motor_output_prepared = false;
    _geometric_boundary_pending = false;
    _geometric_boundary_id = 0;
    _geometric_prearm_main_frames = 0;
    _geometric_prearm_output_frames = 0;
    _geometric_prearm_native_frames = 0;
    _geometric_prearm_snapshot_valid = false;
    _geometric_arm_frame_logged = false;
    copter.geometric_control.reset();
    guided_geometric_heading_mode = 0;
    guided_geometric_trajectory_yaw_allowed = false;

    // Prepare a target synchronously with mode entry.  The main-loop rate
    // task runs before the first Guided update, so an airborne transition
    // must not reuse another mode's output or insert a native PID frame.
    if (full_geometric_entry && geometric_motor_output_configured()) {
        if (is_disarmed_or_landed()) {
            update_geometric_ground_safe_observer();
        } else {
            Vector3f zero_accel_ned_mss {};
            const AC_AttitudeControl::HeadingCommand heading {
                ahrs.get_yaw_rad(),
                0.0f,
                AC_AttitudeControl::HeadingMode::Angle_And_Rate
            };
            update_geometric_position_observer(&current_pos_ned_m,
                                               current_vel_ned_ms,
                                               zero_accel_ned_mss,
                                               heading,
                                               false,
                                               false);
        }
    }

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
    // MAVLink receive runs after update_flight_mode().  A one-shot boundary
    // record written here therefore encloses exactly the first rate frame
    // after a supported target was accepted synchronously.
    if (_geometric_boundary_pending) {
        write_geometric_boundary_frame(1);
        _geometric_boundary_pending = false;
    }

    // A hard in-flight gate failure is latched while the operator continues
    // requesting geometric motor output.  Clearing bit 8 is the explicit
    // acknowledgement that permits a later re-entry attempt.
    if (!option_is_enabled(Option::GeometricMotorOutput)) {
        _geometric_motor_output_rejected = false;
    }

    if (!motors->armed()) {
        _geometric_arm_frame_logged = false;
        if (geometric_motor_output_options_requested() &&
            geometric_motor_output_configured() &&
            _geometric_motor_output_prepared &&
            copter.geometric_control.output_enabled() &&
            copter.geometric_control.enabled() &&
            !copter.geometric_motor_output_blocked_by_rate_thread()) {
            _geometric_prearm_main_frames = copter.main_rate_controller_frames();
            _geometric_prearm_output_frames = copter.geometric_motor_output_frames();
            _geometric_prearm_native_frames = copter.native_rate_controller_frames();
            _geometric_prearm_snapshot_valid = true;
        } else {
            _geometric_prearm_snapshot_valid = false;
        }
    } else {
        write_geometric_prearm_frame_if_needed();
    }

    // Keep the full-geometric path prepared while Guided is armed or disarmed
    // on the ground.  The scheduler runs the rate controller before this mode
    // update, so a continuously fresh ground-hold output is required to ensure
    // the first takeoff-authorised frame cannot fall through to the native PID.
    if (guided_mode != SubMode::TakeOff &&
        geometric_motor_output_configured() &&
        !_geometric_motor_output_rejected &&
        copter.geometric_control.output_enabled() &&
        !copter.geometric_motor_output_blocked_by_rate_thread() &&
        is_disarmed_or_landed()) {
        update_geometric_ground_safe_observer();
    }

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
        if (send_notification && wp_destination_reached()) {
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

    case SubMode::Land:
        geometric_land_run();
        break;
    }
 }

// returns true if the Guided-mode-option is set (see GUID_OPTIONS)
bool ModeGuided::option_is_enabled(Option option) const
{
    return (copter.g2.guided_options.get() & (uint32_t)option) != 0;
}

bool ModeGuided::allows_geometric_motor_output() const
{
    return geometric_motor_output_requested() &&
           !_geometric_motor_output_rejected;
}

bool ModeGuided::geometric_motor_output_options_requested() const
{
    return mode_number() == Number::GUIDED &&
           option_is_enabled(Option::GeometricObserver) &&
           option_is_enabled(Option::GeometricMotorOutput) &&
           !copter.is_tradheli();
}

bool ModeGuided::geometric_motor_output_configured() const
{
    // Several other modes derive from ModeGuided.  Preserve exact-Guided
    // authorization and keep unvalidated terrain, direct-angle and helicopter
    // semantics on their established native actuator paths.
    return geometric_motor_output_options_requested() &&
           geometric_submode_supported();
}

bool ModeGuided::geometric_submode_supported() const
{
    switch (guided_mode) {
    case SubMode::TakeOff:
        return !guided_geometric_takeoff_terrain_alt;
    case SubMode::WP:
        // WPNav path/avoidance semantics are not yet reproduced by the
        // controller-independent geometric target generator.
        return false;
    case SubMode::Pos:
        return !guided_is_terrain_alt;
    case SubMode::PosVelAccel:
    case SubMode::Land:
        return true;
    case SubMode::VelAccel:
    case SubMode::Accel:
        // Native Guided stabilization-option semantics remain authoritative
        // until these target classes have dedicated equivalence tests.
        return false;
    case SubMode::Angle:
        // SET_ATTITUDE_TARGET may carry direct-thrust semantics that the
        // position-derived geometric mapper does not yet reproduce.
        return false;
    }
    return false;
}

bool ModeGuided::geometric_motor_output_requested() const
{
    // A compatible observer update must have run after leaving any unsupported
    // direct-angle or terrain path.  This prevents the rate loop from reusing
    // one stale, semantically incompatible geometric command on re-entry.
    return geometric_motor_output_configured() &&
           _geometric_motor_output_prepared;
}

void ModeGuided::handle_geometric_motor_output_fallback()
{
    pos_control->clear_external_reference();
    if (motors->armed() && geometric_motor_output_requested()) {
        // Do not oscillate between one native frame and geometric re-entry
        // while an output-disable, stale, invalid or rate-thread fault remains.
        _geometric_motor_output_rejected = true;
    }
    Mode::handle_geometric_motor_output_fallback();
}

bool ModeGuided::allows_arming(AP_Arming::Method method) const
{
    // Guided geometric landing waits for AP_Motors to reach GROUND_IDLE and
    // then disarms explicitly.  Do not let GUID_OPTIONS bit 0 turn the
    // internal LANDING method into an immediate touchdown disarm.
    if (method == AP_Arming::Method::LANDING &&
        guided_mode == SubMode::Land &&
        allows_geometric_motor_output()) {
        return false;
    }

    // Requesting the full-geometric actuator path is fail-closed on the
    // ground.  Native Guided remains armable when bit 8 is clear, but a
    // partially configured, stale, rejected or rate-thread-blocked geometric
    // path must not silently arm into native PID output.
    if (method != AP_Arming::Method::LANDING &&
        option_is_enabled(Option::GeometricMotorOutput) &&
        (!geometric_motor_output_options_requested() ||
         !geometric_motor_output_configured() ||
         !_geometric_motor_output_prepared ||
         _geometric_motor_output_rejected ||
         !copter.geometric_control.output_enabled() ||
         !copter.geometric_control.enabled() ||
         !copter.geometric_control.output_is_fresh(millis(), guided_geometric_output_recent_ms) ||
         !copter.geometric_motor_output_is_valid() ||
         copter.geometric_motor_output_blocked_by_rate_thread())) {
        return false;
    }

    // always allow arming from the ground station or scripting
    if (AP_Arming::method_is_GCS(method) || method == AP_Arming::Method::SCRIPTING) {
        return true;
    }

    // optionally allow arming from the transmitter
    return option_is_enabled(Option::AllowArmingFromTX);
};

bool ModeGuided::prepare_for_arming(AP_Arming::Method method)
{
    if (method != AP_Arming::Method::LANDING &&
        option_is_enabled(Option::GeometricMotorOutput)) {
        update_geometric_ground_safe_observer();
    }
    const bool armable = allows_arming(method);

    // Capture the exact counter baseline in the final arming hook, not only
    // in the disarmed mode loop.  SET_MODE(GUIDED), ARM and TAKEOFF may be
    // handled before Guided::run() gets a disarmed iteration; without this
    // synchronous snapshot the motor path is still geometric, but the phase-0
    // lifecycle oracle would be missing for that valid command ordering.
    if (method != AP_Arming::Method::LANDING) {
        _geometric_arm_frame_logged = false;
        if (option_is_enabled(Option::GeometricMotorOutput) &&
            armable &&
            geometric_motor_output_options_requested() &&
            geometric_motor_output_configured()) {
            _geometric_prearm_main_frames = copter.main_rate_controller_frames();
            _geometric_prearm_output_frames = copter.geometric_motor_output_frames();
            _geometric_prearm_native_frames = copter.native_rate_controller_frames();
            _geometric_prearm_snapshot_valid = true;
        } else {
            // A native Guided arm must invalidate any disarmed full-geometric
            // snapshot.  Enabling bit 8 later in flight must not manufacture
            // a stale phase-0 record from an earlier arming attempt.
            _geometric_prearm_snapshot_valid = false;
        }
    }

    return armable;
}

bool ModeGuided::is_landing() const
{
    return guided_mode == SubMode::Land;
}

bool ModeGuided::geometric_position_control_active() const
{
    if (!allows_geometric_motor_output()) {
        return false;
    }
    if (!copter.geometric_control.output_enabled()) {
        return false;
    }
    return !copter.geometric_motor_output_blocked_by_rate_thread();
}

bool ModeGuided::wp_destination_reached() const
{
    if (!geometric_position_control_active() || guided_mode != SubMode::WP || guided_is_terrain_alt) {
        return wp_nav->reached_wp_destination();
    }

    const Vector2f curr_pos_ne_m = pos_control->get_pos_estimate_NED_m().xy().tofloat();
    const Vector2f target_pos_ne_m = guided_pos_target_ned_m.xy().tofloat();
    return get_horizontal_distance(curr_pos_ne_m, target_pos_ne_m) <= wp_nav->get_wp_radius_m();
}

void ModeGuided::restore_native_position_control_after_geometric()
{
    pos_control->clear_external_reference();
    if (!guided_geometric_position_was_active) {
        return;
    }
    pos_control->NE_init_controller();
    pos_control->D_init_controller();
    guided_geometric_position_was_active = false;
    guided_geometric_target_manager.reset();
    copter.geometric_control.reset();
}

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
    case SubMode::Land:
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
    _geometric_motor_output_prepared = false;

    // initialise yaw
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

    // clear i term when we're taking off
    pos_control->D_init_controller();

    // initialise alt for WP_NAVALT_MIN and set completion alt
    auto_takeoff.start_m(alt_target_m, alt_target_terrain);

    // Keep a controller-independent NED takeoff target.  The geometric path
    // must not call _AutoTakeoff::run(), because that helper couples target
    // shaping to the native position and attitude feedback controllers.
    guided_geometric_takeoff_target_ned_m = pos_control->get_pos_estimate_NED_m();
    guided_geometric_takeoff_target_ned_m.z = -alt_target_m;
    guided_geometric_takeoff_terrain_alt = alt_target_terrain;
    guided_geometric_takeoff_spool_ready = false;

    // Retag the already-safe ground output for the TakeOff submode before the
    // next rate frame.  The first unrestricted target is generated later,
    // only after AP_Motors reports THROTTLE_UNLIMITED.
    if (geometric_motor_output_configured()) {
        update_geometric_ground_safe_observer();
    }

    // Normally the first armed Guided::run() has already emitted phase 0.
    // Keep the command handler robust to an arm/takeoff command pair arriving
    // before that mode update by emitting the stored exact pre-arm snapshot.
    write_geometric_prearm_frame_if_needed();

#if HAL_LOGGING_ENABLED
    copter.Log_Write_Geometric_Full_Lifecycle(1,
                                              copter.main_rate_controller_frames(),
                                              copter.geometric_motor_output_frames(),
                                              copter.native_rate_controller_frames());
#endif

    // record takeoff has not completed
    takeoff_complete = false;

    return true;
}

bool ModeGuided::start_geometric_landing()
{
    if (mode_number() != Number::GUIDED ||
        !geometric_position_control_active() ||
        !motors->armed() ||
        copter.ap.land_complete ||
        copter.is_tradheli()) {
        return false;
    }

    guided_mode = SubMode::Land;
    guided_geometric_land_hold_ned_m = pos_control->get_pos_estimate_NED_m();
    guided_geometric_land_yaw_rad = ahrs.get_yaw_rad();
    guided_geometric_land_descent_ned_ms = 0.0f;
    _paused = false;
    guided_pause_pos_valid = false;
    guided_pause_yaw_valid = false;
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

    // Generate the Land hold target synchronously so the first rate frame
    // after accepting MAV_CMD_NAV_LAND remains geometric.
    Vector3f zero_target {};
    const AC_AttitudeControl::HeadingCommand heading {
        guided_geometric_land_yaw_rad,
        0.0f,
        AC_AttitudeControl::HeadingMode::Angle_And_Rate
    };
    update_geometric_position_observer(&guided_geometric_land_hold_ned_m,
                                       zero_target,
                                       zero_target,
                                       heading,
                                       false,
                                       false);

#if AP_LANDINGGEAR_ENABLED
    copter.landinggear.deploy_for_landing();
#endif

#if HAL_LOGGING_ENABLED
    copter.Log_Write_Geometric_Full_Lifecycle(2,
                                              copter.main_rate_controller_frames(),
                                              copter.geometric_motor_output_frames(),
                                              copter.native_rate_controller_frames());
#endif
    return true;
}

// initialise guided mode's waypoint navigation controller
void ModeGuided::wp_control_start()
{
    // set to position control mode
    guided_mode = SubMode::WP;
    _geometric_motor_output_prepared = false;

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

    if (geometric_position_control_active() && !guided_is_terrain_alt) {
        guided_geometric_position_was_active = true;
        guided_vel_target_ned_ms.zero();
        guided_accel_target_ned_mss.zero();
        const AC_AttitudeControl::HeadingCommand heading = auto_yaw.get_heading();
        update_geometric_position_observer(&guided_pos_target_ned_m,
                                           guided_vel_target_ned_ms,
                                           guided_accel_target_ned_mss,
                                           heading,
                                           true,
                                           guided_geometric_target_manager.trajectory_yaw_allowed());
        return;
    }
    restore_native_position_control_after_geometric();

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->D_update_controller();

    // call attitude controller with auto yaw
    const AC_AttitudeControl::HeadingCommand heading = auto_yaw.get_heading();
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), heading);
    const Vector3p& position_target_ned_m = pos_control->get_pos_target_NED_m();
    update_geometric_position_observer(&position_target_ned_m,
                                       pos_control->get_vel_desired_NED_ms(),
                                       pos_control->get_accel_desired_NED_mss(),
                                       heading,
                                       false);
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
    _geometric_motor_output_prepared = false;

    // initialise position controller
    pva_control_start();
}

// initialise guided mode's velocity and acceleration controller
void ModeGuided::velaccel_control_start()
{
    // set guided_mode to velocity and acceleration controller
    guided_mode = SubMode::VelAccel;
    _geometric_motor_output_prepared = false;

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
    _geometric_motor_output_prepared = false;
    pos_control->clear_external_reference();

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
    guided_geometric_target_manager.reset();
}

// set_pos_ned_m - sets guided mode's target pos_ned_m
// Returns true if the fence is enabled and guided waypoint is within the fence
// else return false if the waypoint is outside the fence
bool ModeGuided::set_pos_NED_m(const Vector3p& pos_ned_m, bool use_yaw, float yaw_rad, bool use_yaw_rate, float yaw_rate_rads, bool relative_yaw, bool is_terrain_alt)
{
    const bool geometric_boundary_was_unsupported =
        motors->armed() &&
        !is_disarmed_or_landed() &&
        geometric_motor_output_options_requested() &&
        !geometric_motor_output_requested() &&
        !_geometric_motor_output_rejected &&
        copter.geometric_control.output_enabled() &&
        !copter.geometric_motor_output_blocked_by_rate_thread();

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

        _paused = false;
        guided_pause_pos_valid = false;
        guided_pause_yaw_valid = false;

        // set yaw state
        set_yaw_state_rad(use_yaw, yaw_rad, use_yaw_rate, yaw_rate_rads, relative_yaw);

        const Vector3p& current_pos_ned_m = pos_control->get_pos_estimate_NED_m();
        const Vector3p& adjusted_pos_ned_m = guided_geometric_target_manager.set_position_target(pos_ned_m,
                                                                                                 is_terrain_alt,
                                                                                                 geometric_position_control_active() ? &current_pos_ned_m : nullptr);
        guided_pos_target_ned_m = adjusted_pos_ned_m;
        guided_is_terrain_alt = is_terrain_alt;
        if (is_terrain_alt) {
            _geometric_motor_output_prepared = false;
        }
        guided_vel_target_ned_ms.zero();
        guided_accel_target_ned_mss.zero();
        update_time_ms = millis();

        // no need to check return status because terrain data is not used
        wp_nav->set_wp_destination_NED_m(adjusted_pos_ned_m, is_terrain_alt);

#if HAL_LOGGING_ENABLED
        // log target
        copter.Log_Write_Guided_Position_Target(guided_mode, adjusted_pos_ned_m, is_terrain_alt, Vector3f(), Vector3f());
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

    _paused = false;
    guided_pause_pos_valid = false;
    guided_pause_yaw_valid = false;

    // set yaw state
    set_yaw_state_rad(use_yaw, yaw_rad, use_yaw_rate, yaw_rate_rads, relative_yaw);

    // set position target and zero velocity and acceleration
    const Vector3p& current_pos_ned_m = pos_control->get_pos_estimate_NED_m();
    const Vector3p& adjusted_pos_ned_m = guided_geometric_target_manager.set_position_target(pos_ned_m,
                                                                                             is_terrain_alt,
                                                                                             geometric_position_control_active() ? &current_pos_ned_m : nullptr);
    guided_pos_target_ned_m = adjusted_pos_ned_m;
    guided_is_terrain_alt = is_terrain_alt;
    if (is_terrain_alt) {
        _geometric_motor_output_prepared = false;
    }
    guided_vel_target_ned_ms.zero();
    guided_accel_target_ned_mss.zero();
    update_time_ms = millis();

    // GCS input is processed after update_flight_mode().  Prepare the new
    // compatible target inside this handler so the very next rate frame uses
    // geometry instead of inserting one native PID bridge frame.
    prepare_geometric_position_observer(true,
                                        guided_geometric_target_manager.trajectory_yaw_allowed());
    begin_geometric_supported_boundary(geometric_boundary_was_unsupported);

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
    case SubMode::Land:
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
    const bool geometric_boundary_was_unsupported =
        motors->armed() &&
        !is_disarmed_or_landed() &&
        geometric_motor_output_options_requested() &&
        !geometric_motor_output_requested() &&
        !_geometric_motor_output_rejected &&
        copter.geometric_control.output_enabled() &&
        !copter.geometric_motor_output_blocked_by_rate_thread();

#if AP_FENCE_ENABLED
    // reject destination outside the fence.
    // Note: there is a danger that a target specified as a terrain altitude might not be checked if the conversion to alt-above-home fails
    if (!copter.fence.check_location_within_fence(dest_loc)) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    Vector3p pos_target_ned_m;
    bool is_terrain_alt;
    const bool have_pos_target = wp_nav->get_vector_NED_m(dest_loc, pos_target_ned_m, is_terrain_alt);

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

        _paused = false;
        guided_pause_pos_valid = false;
        guided_pause_yaw_valid = false;

        // set yaw state
        set_yaw_state_rad(use_yaw, yaw_rad, use_yaw_rate, yaw_rate_rads, relative_yaw);

        Vector3p adjusted_pos_target_ned_m;
        if (have_pos_target) {
            if (geometric_position_control_active()) {
                adjusted_pos_target_ned_m = guided_geometric_target_manager.set_destination_target(pos_target_ned_m, is_terrain_alt);
            } else {
                adjusted_pos_target_ned_m = guided_geometric_target_manager.set_position_target(pos_target_ned_m, is_terrain_alt);
            }
            guided_pos_target_ned_m = adjusted_pos_target_ned_m;
            guided_is_terrain_alt = is_terrain_alt;
            if (is_terrain_alt) {
                _geometric_motor_output_prepared = false;
            }
            guided_vel_target_ned_ms.zero();
            guided_accel_target_ned_mss.zero();
            update_time_ms = millis();

            if (geometric_position_control_active() && !is_terrain_alt) {
                // Keep the native WPNav fallback target aligned with the
                // geometric target after altitude hold adjustment.
                wp_nav->set_wp_destination_NED_m(adjusted_pos_target_ned_m, false);
            }
        }

#if HAL_LOGGING_ENABLED
        // log target
        if (have_pos_target) {
            copter.Log_Write_Guided_Position_Target(guided_mode, adjusted_pos_target_ned_m, is_terrain_alt, Vector3f(), Vector3f());
        } else {
            copter.Log_Write_Guided_Position_Target(guided_mode, Vector3p(dest_loc.lat, dest_loc.lng, dest_loc.alt), (dest_loc.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN), Vector3f(), Vector3f());
        }
#endif

        send_notification = true;
        return true;
    }

    // set position target and zero velocity and acceleration
    if (!have_pos_target) {
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

    _paused = false;
    guided_pause_pos_valid = false;
    guided_pause_yaw_valid = false;

    Vector3p adjusted_pos_target_ned_m;
    if (geometric_position_control_active()) {
        adjusted_pos_target_ned_m = guided_geometric_target_manager.set_destination_target(pos_target_ned_m, is_terrain_alt);
    } else {
        adjusted_pos_target_ned_m = guided_geometric_target_manager.set_position_target(pos_target_ned_m, is_terrain_alt);
    }
    guided_pos_target_ned_m = adjusted_pos_target_ned_m;
    guided_is_terrain_alt = is_terrain_alt;
    if (is_terrain_alt) {
        _geometric_motor_output_prepared = false;
    }
    guided_vel_target_ned_ms.zero();
    guided_accel_target_ned_mss.zero();
    update_time_ms = millis();

    prepare_geometric_position_observer(true,
                                        guided_geometric_target_manager.trajectory_yaw_allowed());
    begin_geometric_supported_boundary(geometric_boundary_was_unsupported);

    // log target
#if HAL_LOGGING_ENABLED
    copter.Log_Write_Guided_Position_Target(guided_mode, guided_pos_target_ned_m, guided_is_terrain_alt, guided_vel_target_ned_ms, guided_accel_target_ned_mss);
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
    guided_geometric_target_manager.reset();
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
    guided_geometric_target_manager.reset();
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
    const bool geometric_boundary_was_unsupported =
        motors->armed() &&
        !is_disarmed_or_landed() &&
        geometric_motor_output_options_requested() &&
        !geometric_motor_output_requested() &&
        !_geometric_motor_output_rejected &&
        copter.geometric_control.output_enabled() &&
        !copter.geometric_motor_output_blocked_by_rate_thread();

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
    const Vector3p& current_pos_ned_m = pos_control->get_pos_estimate_NED_m();
    guided_pos_target_ned_m = guided_geometric_target_manager.set_position_target(pos_ned_m,
                                                                                  false,
                                                                                  geometric_position_control_active() ? &current_pos_ned_m : nullptr);
    guided_is_terrain_alt = false;
    guided_vel_target_ned_ms = vel_ned_ms;
    guided_accel_target_ned_mss = accel_ned_mss;

    prepare_geometric_position_observer(true,
                                        guided_geometric_target_manager.trajectory_yaw_allowed());
    begin_geometric_supported_boundary(geometric_boundary_was_unsupported);

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
    if (geometric_position_control_active()) {
        guided_geometric_position_was_active = true;

        const Vector3p current_pos_ned_m = pos_control->get_pos_estimate_NED_m();
        Vector3f zero_velocity_ned_ms {};
        Vector3f zero_accel_ned_mss {};
        const AC_AttitudeControl::HeadingCommand heading = auto_yaw.get_heading();

        // AP_Motors still owns arm/interlock/spool safety.  While it cannot
        // apply unrestricted thrust, keep a fresh level geometric hold output
        // and reset the geometric integrators each cycle.  This prepares the
        // first armed rate frame without winding up against the spool limit.
        if (!motors->armed() || !copter.ap.auto_armed ||
            motors->get_spool_state() != AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
            if (motors->armed() && copter.ap.auto_armed) {
                motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
            } else {
                make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
            }
            guided_geometric_takeoff_spool_ready = false;
            update_geometric_ground_safe_observer();
            return;
        }

        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        Vector3p takeoff_target_ned_m = guided_geometric_takeoff_target_ned_m;
        if (guided_geometric_takeoff_terrain_alt) {
            float terrain_u_m = 0.0f;
            if (!wp_nav->get_terrain_U_m(terrain_u_m)) {
                copter.failsafe_terrain_on_event();
                return;
            }
            takeoff_target_ned_m.z -= terrain_u_m;
        }

        if (!guided_geometric_takeoff_spool_ready) {
            // Start the jerk-limited geometric reference at the measured state
            // exactly when AP_Motors grants full throttle authority.
            copter.geometric_control.reset();
            guided_geometric_target_manager.reset();
            guided_geometric_takeoff_spool_ready = true;
        }

        update_geometric_position_observer(&takeoff_target_ned_m,
                                           zero_velocity_ned_ms,
                                           zero_accel_ned_mss,
                                           heading,
                                           true,
                                           false);

        if (copter.ap.land_complete) {
            const AC_Geometric_Mapped_Output& mapped = copter.geometric_control.get_output().mapped;
            if (mapped.throttle_norm >= MIN(copter.g2.takeoff_throttle_max, 0.9f) ||
                pos_control->get_estimated_accel_U_mss() >= 0.5f * pos_control->D_get_max_accel_mss() ||
                pos_control->get_vel_estimate_U_ms() >= 0.1f * pos_control->get_max_speed_up_ms()) {
                set_land_complete(false);
            }
        }

        const float vel_threshold_fraction = 0.1f;
        const float stop_distance_m = 0.5f * sq(vel_threshold_fraction * pos_control->get_max_speed_up_ms()) /
                                      pos_control->D_get_max_accel_mss();
        const bool reached_altitude = fabsf(float(current_pos_ned_m.z - takeoff_target_ned_m.z)) <= MAX(stop_distance_m, 0.1f);
        const bool reached_climb_rate = fabsf(pos_control->get_vel_estimate_U_ms()) <
                                        pos_control->get_max_speed_up_ms() * vel_threshold_fraction;
        if (reached_altitude && reached_climb_rate && !takeoff_complete) {
            takeoff_complete = true;
#if AP_FENCE_ENABLED
            copter.fence.auto_enable_fence_after_takeoff();
#endif
#if AP_LANDINGGEAR_ENABLED
            copter.landinggear.retract_after_takeoff();
#endif
        }
        return;
    }

    restore_native_position_control_after_geometric();
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

// Run a position-derived geometric landing without entering native LAND mode.
// Horizontal position and yaw are held at the landing command point.  The
// vertical channel uses a smoothly-ramped NED descent velocity so contact keeps
// commanding reduced collective until the standard land detector fires.
void ModeGuided::geometric_land_run()
{
    if (!geometric_position_control_active()) {
        // A hard geometric gate failure exits this specialised submode to the
        // established native LAND failsafe.  This is outside the nominal
        // full-geometric lifecycle guarantee.
        copter.set_mode(Number::LAND, ModeReason::FAILSAFE);
        return;
    }

    guided_geometric_position_was_active = true;
    const Vector3p current_pos_ned_m = pos_control->get_pos_estimate_NED_m();
    const AC_AttitudeControl::HeadingCommand heading {
        guided_geometric_land_yaw_rad,
        0.0f,
        AC_AttitudeControl::HeadingMode::Angle_And_Rate
    };
    Vector3f target_velocity_ned_ms {};
    Vector3f target_accel_ned_mss {};

    if (!motors->armed()) {
        copter.geometric_control.set_enabled(false);
        return;
    }

    if (copter.ap.land_complete) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);

        // Keep the rate path geometric until AP_Motors has completed its safe
        // spool-down.  Resetting each cycle prevents contact integrator windup;
        // the spool state, not a native feedback controller, enforces ground output.
        update_geometric_ground_safe_observer();

        if (motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
#if HAL_LOGGING_ENABLED
            copter.Log_Write_Geometric_Full_Lifecycle(4,
                                                      copter.main_rate_controller_frames(),
                                                      copter.geometric_motor_output_frames(),
                                                      copter.native_rate_controller_frames());
#endif
            if (copter.arming.disarm(AP_Arming::Method::LANDED)) {
                // Close this landing epoch only after disarm succeeds.  Leaving
                // Guided in SubMode::Land would make a later re-arm immediately
                // execute the already-complete landing branch and disarm again.
                // Re-establish the same supported, zero-collective PVA ground
                // hold used at mode entry so another arm/takeoff can start
                // without leaving Guided.  If disarm fails, retaining Land
                // causes the next cycle to retry safely.
                posvelaccel_control_start();
                guided_pos_target_ned_m = pos_control->get_pos_estimate_NED_m();
                guided_vel_target_ned_ms = pos_control->get_vel_estimate_NED_ms();
                guided_accel_target_ned_mss.zero();
                guided_is_terrain_alt = false;
                update_time_ms = millis();
                guided_geometric_target_manager.reset();
                guided_geometric_takeoff_target_ned_m.zero();
                guided_geometric_takeoff_terrain_alt = false;
                guided_geometric_takeoff_spool_ready = false;
                guided_geometric_land_descent_ned_ms = 0.0f;
                takeoff_complete = false;
                update_geometric_ground_safe_observer();
                _geometric_arm_frame_logged = false;
                _geometric_prearm_snapshot_valid = false;
            }
        }
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    const float land_alt_low_m = copter.mode_land.get_land_alt_low_m();
    const float land_speed_ms = fabsf(copter.mode_land.get_land_speed_ms());
    const float configured_high_speed_ms = copter.mode_land.get_land_speed_high_ms();
    const float max_descent_speed_ms = MAX(configured_high_speed_ms > 0.0f ?
                                           configured_high_speed_ms : wp_nav->get_default_speed_down_ms(),
                                           land_speed_ms);
    const float climb_rate_ms = constrain_float(
        sqrt_controller(MAX(land_alt_low_m, 1.0f) - get_alt_above_ground_m(),
                        pos_control->D_get_pos_p().kP(),
                        pos_control->D_get_max_accel_mss(),
                        G_Dt),
        -max_descent_speed_ms,
        -land_speed_ms);
    const float requested_descent_ned_ms = -climb_rate_ms;
    const float descent_delta_max_ms = MAX(pos_control->D_get_max_accel_mss(), 0.1f) * G_Dt;
    guided_geometric_land_descent_ned_ms += constrain_float(
        requested_descent_ned_ms - guided_geometric_land_descent_ned_ms,
        -descent_delta_max_ms,
        descent_delta_max_ms);

    Vector3p landing_target_ned_m = guided_geometric_land_hold_ned_m;
    landing_target_ned_m.z = current_pos_ned_m.z;
    target_velocity_ned_ms.z = guided_geometric_land_descent_ned_ms;
    update_geometric_position_observer(&landing_target_ned_m,
                                       target_velocity_ned_ms,
                                       target_accel_ned_mss,
                                       heading,
                                       false,
                                       false);
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

    if (geometric_position_control_active() && !guided_is_terrain_alt) {
        guided_geometric_position_was_active = true;
        guided_accel_target_ned_mss.zero();
        guided_vel_target_ned_ms.zero();
        if (millis() - update_time_ms > get_timeout_ms()) {
            if ((auto_yaw.mode() == AutoYaw::Mode::RATE) || (auto_yaw.mode() == AutoYaw::Mode::ANGLE_RATE)) {
                auto_yaw.set_mode(AutoYaw::Mode::HOLD);
            }
        }
        const AC_AttitudeControl::HeadingCommand heading = auto_yaw.get_heading();
        update_geometric_position_observer(&guided_pos_target_ned_m,
                                           guided_vel_target_ned_ms,
                                           guided_accel_target_ned_mss,
                                           heading,
                                           true,
                                           guided_geometric_target_manager.trajectory_yaw_allowed());
        return;
    }
    restore_native_position_control_after_geometric();

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

    if (geometric_position_control_active() && !guided_is_terrain_alt) {
        guided_geometric_position_was_active = true;
        if (!guided_pause_pos_valid) {
            guided_pause_pos_ned_m = pos_control->get_pos_estimate_NED_m();
            guided_pause_pos_valid = true;
        }
        const AC_AttitudeControl::HeadingCommand heading {
            guided_pause_yaw_valid ? guided_pause_yaw_rad : ahrs.get_yaw_rad(),
            0.0f,
            AC_AttitudeControl::HeadingMode::Angle_And_Rate
        };
        Vector3f zero_target;
        zero_target.zero();
        update_geometric_position_observer(&guided_pause_pos_ned_m,
                                           zero_target,
                                           zero_target,
                                           heading,
                                           false,
                                           false);
        return;
    }
    restore_native_position_control_after_geometric();

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
                                       heading,
                                       true,
                                       guided_geometric_target_manager.trajectory_yaw_allowed());
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

bool ModeGuided::update_geometric_observer(const AC_Geometric_Target& target)
{
    // Historical name: this routine evaluates the shared geometric cascade.
    // In observer-only configurations its result is diagnostic; when full
    // output is prepared, the same mapped result becomes actuator intent after
    // the frame-exclusive ownership gate authorizes it.
    AC_Geometric_State geometric_state {};
    if (!run_geometric_observer(target,
                                option_is_enabled(Option::GeometricObserver),
                                geometric_state)) {
        return false;
    }

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
    // @Field: EIx: Geometric integral error, X-Axis
    // @Field: EIy: Geometric integral error, Y-Axis
    // @Field: EIz: Geometric integral error, Z-Axis
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

    // @LoggerMessage: GEOC
    // @Description: Geometric commanded attitude coupling observer
    // @Field: TimeUS: Time since system startup
    // @Field: RCr: Commanded attitude roll
    // @Field: RCp: Commanded attitude pitch
    // @Field: RCy: Commanded attitude yaw
    // @Field: ATr: ArduPilot attitude target roll
    // @Field: ATp: ArduPilot attitude target pitch
    // @Field: ATy: ArduPilot attitude target yaw
    // @Field: OCx: Position-generated commanded angular velocity, X-Axis
    // @Field: OCy: Position-generated commanded angular velocity, Y-Axis
    // @Field: OCz: Position-generated commanded angular velocity, Z-Axis
    // @Field: ODx: Position-generated commanded angular acceleration, X-Axis
    // @Field: ODy: Position-generated commanded angular acceleration, Y-Axis
    // @Field: ODz: Position-generated commanded angular acceleration, Z-Axis
    // @Field: YT: Input yaw target
    // @Field: YR: Input yaw-rate target

    // @LoggerMessage: GEOS
    // @Description: Geometric guided setpoint shaper observer
    // @Field: TimeUS: Time since system startup
    // @Field: RX: Raw position target, X-Axis
    // @Field: RY: Raw position target, Y-Axis
    // @Field: RZ: Raw position target, Z-Axis
    // @Field: SX: Shaped position target, X-Axis
    // @Field: SY: Shaped position target, Y-Axis
    // @Field: SZ: Shaped position target, Z-Axis
    // @Field: SAct: True if the geometric setpoint shaper was active
    // @Field: YTrj: True if yaw was derived from shaped trajectory velocity

    // @LoggerMessage: GEOT
    // @Description: Geometric guided target semantic observer
    // @Field: TimeUS: Time since system startup
    // @Field: TType: Geometric Guided target type
    // @Field: AutoY: AutoYaw mode
    // @Field: HMode: Heading command mode
    // @Field: Pause: True if Guided pause is active
    // @Field: Shape: True if geometric target shaping is requested
    // @Field: YTrj: True if yaw was derived from shaped trajectory velocity
    // @Field: Allow: True if this Guided target allows trajectory yaw-follow

    // @LoggerMessage: GESV
    // @Description: Geometric guided shaped velocity and acceleration observer
    // @Field: TimeUS: Time since system startup
    // @Field: VX: Shaped velocity target, X-Axis
    // @Field: VY: Shaped velocity target, Y-Axis
    // @Field: VZ: Shaped velocity target, Z-Axis
    // @Field: AX: Shaped acceleration target, X-Axis
    // @Field: AY: Shaped acceleration target, Y-Axis
    // @Field: AZ: Shaped acceleration target, Z-Axis
    // @Field: Yaw: Shaped yaw target
    // @Field: YR: Shaped yaw-rate target

    // @LoggerMessage: GEOZ
    // @Description: Geometric guided vertical-channel observer
    // @Field: TimeUS: Time since system startup
    // @Field: Z: Current NED position, Z-Axis
    // @Field: VZ: Current NED velocity, Z-Axis
    // @Field: RZ: Raw position target, Z-Axis
    // @Field: SZ: Shaped position target, Z-Axis
    // @Field: TVZ: Shaped velocity target, Z-Axis
    // @Field: TAZ: Shaped acceleration target, Z-Axis
    // @Field: PEz: Position error, Z-Axis
    // @Field: VEz: Velocity error, Z-Axis

    // @LoggerMessage: GEZI
    // @Description: Geometric guided vertical integral and throttle observer
    // @Field: TimeUS: Time since system startup
    // @Field: IEz: Position integral error, Z-Axis
    // @Field: SFz: Specific force command, Z-Axis
    // @Field: Thr: Projected total thrust per mass
    // @Field: TRw: Raw normalized throttle output
    // @Field: TN: Limited normalized throttle output
    // @Field: SAct: True if the geometric setpoint shaper was active
    // @Field: TLim: True if normalized throttle output was limited

    // @LoggerMessage: GEOO
    // @Description: Geometric guided output mapper
    // @Field: TimeUS: Time since system startup
    // @Field: TRaw: Raw candidate normalized collective intent
    // @Field: TNorm: Limited candidate normalized collective intent
    // @Field: TLim: True if candidate normalized collective intent was limited
    // @Field: RCr: Commanded geometric attitude roll
    // @Field: RCp: Commanded geometric attitude pitch
    // @Field: RCy: Commanded geometric attitude yaw
    // @Field: RTx: Legacy diagnostic rate-target proxy, X-Axis
    // @Field: RTy: Legacy diagnostic rate-target proxy, Y-Axis
    // @Field: RTz: Legacy diagnostic rate-target proxy, Z-Axis

    // @LoggerMessage: GEOM
    // @Description: Geometric guided candidate actuator-intent mapper
    // @Field: TimeUS: Time since system startup
    // @Field: RRaw: Raw candidate normalized roll actuator intent
    // @Field: PRaw: Raw candidate normalized pitch actuator intent
    // @Field: YRaw: Raw candidate normalized yaw actuator intent
    // @Field: Roll: Limited candidate normalized roll actuator intent
    // @Field: Pitch: Limited candidate normalized pitch actuator intent
    // @Field: Yaw: Limited candidate normalized yaw actuator intent
    // @Field: Lim: True if any candidate actuator intent was mapper-limited

    // @LoggerMessage: GEOX
    // @Description: Geometric motor-output hook status
    // @Field: TimeUS: Time since system startup
    // @Field: Allow: True if the current mode allows geometric motor output
    // @Field: OEn: True if GEO_OUT_EN allows geometric motor output
    // @Field: RT: True if rate thread is active
    // @Field: Wrote: True if the geometric path recently wrote AP_Motors
    // @Field: GAge: Geometric controller output age
    // @Field: WAge: Geometric AP_Motors write age
    // @Field: Roll: Limited normalized roll actuator output
    // @Field: Pitch: Limited normalized pitch actuator output
    // @Field: Yaw: Limited normalized yaw actuator output
    // @Field: Thr: Limited normalized throttle output
    // @Field: RLim: True if any roll/pitch/yaw actuator output was limited
    // @Field: TLim: True if normalized throttle output was limited

    // @LoggerMessage: GEFR
    // @Description: Geometric and native main-loop frame-counter snapshot
    // @Field: TimeUS: Time since system startup
    // @Field: MFrm: Cumulative main-loop rate-controller frames
    // @Field: GFrm: Cumulative geometric motor-output frames
    // @Field: NFrm: Cumulative native rate-controller frames
    if (guided_geometric_log_counter++ % 5 == 0) {
        const AC_Geometric_Output& output = copter.geometric_control.get_output();
        const uint32_t now_ms = AP_HAL::millis();
        const uint32_t geometric_age_ms = copter.geometric_control.output_age_ms(now_ms);
        const uint32_t motor_output_age_ms = copter.geometric_motor_output_age_ms(now_ms);
        const bool motor_output_allowed = allows_geometric_motor_output();
        const bool geometric_output_enabled = copter.geometric_control.output_enabled();
        const bool rate_thread_active = copter.geometric_motor_output_blocked_by_rate_thread();
        const bool motor_output_written_recently = motor_output_age_ms <= guided_geometric_output_recent_ms;
        const AC_Geometric_Target& raw_target = copter.geometric_control.get_raw_target();
        const AC_Geometric_Target& shaped_target = copter.geometric_control.get_shaped_target();
        const bool shaper_active = copter.geometric_control.shaper_active();
        AP::logger().WriteStreaming("GEOA", "TimeUS,ERx,ERy,ERz,EOx,EOy,EOz,Mx,My,Mz,EIx,EIy,EIz,RTx,RTy,RTz", "Qfffffffffffffff",
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
                                    (double)output.attitude.integral_error.x,
                                    (double)output.attitude.integral_error.y,
                                    (double)output.attitude.integral_error.z,
                                    (double)output.attitude.rate_target_body_rads.x,
                                    (double)output.attitude.rate_target_body_rads.y,
                                    (double)output.attitude.rate_target_body_rads.z);

        copter.Log_Write_Geometric_Attitude_Error(output.attitude);

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

        AP::logger().WriteStreaming("GEOC", "TimeUS,RCr,RCp,RCy,ATr,ATp,ATy,OCx,OCy,OCz,ODx,ODy,ODz,YT,YR", "Qffffffffffffff",
                                    AP_HAL::micros64(),
                                    (double)rc_roll_rad,
                                    (double)rc_pitch_rad,
                                    (double)rc_yaw_rad,
                                    (double)ap_attitude_target_rad.x,
                                    (double)ap_attitude_target_rad.y,
                                    (double)ap_attitude_target_rad.z,
                                    (double)output.position.omega_body_rads.x,
                                    (double)output.position.omega_body_rads.y,
                                    (double)output.position.omega_body_rads.z,
                                    (double)output.position.omega_dot_body_radss.x,
                                    (double)output.position.omega_dot_body_radss.y,
                                    (double)output.position.omega_dot_body_radss.z,
                                    (double)target.yaw_rad,
                                    (double)target.yaw_rate_rads);

        AP::logger().WriteStreaming("GEOS", "TimeUS,RX,RY,RZ,SX,SY,SZ,SAct,YTrj", "QffffffBB",
                                    AP_HAL::micros64(),
                                    (double)raw_target.position_ned_m.x,
                                    (double)raw_target.position_ned_m.y,
                                    (double)raw_target.position_ned_m.z,
                                    (double)shaped_target.position_ned_m.x,
                                    (double)shaped_target.position_ned_m.y,
                                    (double)shaped_target.position_ned_m.z,
                                    (uint8_t)shaper_active,
                                    (uint8_t)shaped_target.yaw_from_trajectory);

        AP::logger().WriteStreaming("GEOT", "TimeUS,TType,AutoY,HMode,Pause,Shape,YTrj,Allow", "QBBBBBBB",
                                    AP_HAL::micros64(),
                                    (uint8_t)guided_geometric_target_manager.target_type(),
                                    (uint8_t)auto_yaw.mode(),
                                    guided_geometric_heading_mode,
                                    (uint8_t)_paused,
                                    (uint8_t)target.shape_position_target,
                                    (uint8_t)shaped_target.yaw_from_trajectory,
                                    (uint8_t)guided_geometric_trajectory_yaw_allowed);

        AP::logger().WriteStreaming("GESV", "TimeUS,VX,VY,VZ,AX,AY,AZ,Yaw,YR", "Qffffffff",
                                    AP_HAL::micros64(),
                                    (double)shaped_target.velocity_ned_ms.x,
                                    (double)shaped_target.velocity_ned_ms.y,
                                    (double)shaped_target.velocity_ned_ms.z,
                                    (double)shaped_target.accel_ned_mss.x,
                                    (double)shaped_target.accel_ned_mss.y,
                                    (double)shaped_target.accel_ned_mss.z,
                                    (double)shaped_target.yaw_rad,
                                    (double)shaped_target.yaw_rate_rads);

        AP::logger().WriteStreaming("GEOZ", "TimeUS,Z,VZ,RZ,SZ,TVZ,TAZ,PEz,VEz", "Qffffffff",
                                    AP_HAL::micros64(),
                                    (double)geometric_state.position_ned_m.z,
                                    (double)geometric_state.velocity_ned_ms.z,
                                    (double)raw_target.position_ned_m.z,
                                    (double)shaped_target.position_ned_m.z,
                                    (double)shaped_target.velocity_ned_ms.z,
                                    (double)shaped_target.accel_ned_mss.z,
                                    (double)output.position.position_error_m.z,
                                    (double)output.position.velocity_error_ms.z);

        AP::logger().WriteStreaming("GEZI", "TimeUS,IEz,SFz,Thr,TRw,TN,SAct,TLim", "QfffffBB",
                                    AP_HAL::micros64(),
                                    (double)output.position.integral_error_m.z,
                                    (double)output.position.specific_force_ned_mss.z,
                                    (double)output.position.thrust,
                                    (double)output.mapped.throttle_norm_raw,
                                    (double)output.mapped.throttle_norm,
                                    (uint8_t)shaper_active,
                                    (uint8_t)output.mapped.throttle_limited);

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

        copter.Log_Write_Geometric_Output_State(motor_output_allowed,
                                                geometric_output_enabled,
                                                rate_thread_active,
                                                motor_output_written_recently,
                                                geometric_age_ms,
                                                motor_output_age_ms,
                                                output.mapped);
        copter.Log_Write_Geometric_Frame_Counters();
    }
#endif
    return true;
}

void ModeGuided::prepare_geometric_position_observer(bool shape_position_target,
                                                     bool allow_trajectory_yaw)
{
    // Guided command handlers can accept a new target between main-rate
    // frames. Prepare it synchronously so the next rate frame sees a fresh,
    // semantically compatible command instead of a native-PID bridge frame.
    if (!geometric_motor_output_configured() ||
        _geometric_motor_output_rejected) {
        return;
    }

    if (is_disarmed_or_landed()) {
        update_geometric_ground_safe_observer();
        return;
    }

    const AC_AttitudeControl::HeadingCommand heading = auto_yaw.get_heading();
    update_geometric_position_observer(&guided_pos_target_ned_m,
                                       guided_vel_target_ned_ms,
                                       guided_accel_target_ned_mss,
                                       heading,
                                       shape_position_target,
                                       allow_trajectory_yaw);
}

void ModeGuided::begin_geometric_supported_boundary(bool boundary_was_unsupported)
{
    if (!boundary_was_unsupported || !geometric_position_control_active()) {
        return;
    }

    _geometric_boundary_id++;
    write_geometric_boundary_frame(0);
    _geometric_boundary_pending = true;
}

void ModeGuided::write_geometric_boundary_frame(uint8_t phase) const
{
#if HAL_LOGGING_ENABLED
    // @LoggerMessage: GEFB
    // @Description: Exact unsupported-to-supported Guided geometric boundary frame snapshot
    // @Field: TimeUS: Time since system startup
    // @Field: Edge: Boundary sequence identifier
    // @Field: Phase: Boundary phase (0 target synchronously prepared, 1 after the next main-loop rate frame)
    // @Field: Sub: Guided submode
    // @Field: Prep: True when a semantically compatible geometric target is prepared
    // @Field: Allow: True when the mode authorizes geometric motor output
    // @Field: MFrm: Cumulative main-loop rate-controller frames
    // @Field: GFrm: Cumulative geometric motor-output frames
    // @Field: NFrm: Cumulative native rate-controller frames
    AP::logger().Write("GEFB", "TimeUS,Edge,Phase,Sub,Prep,Allow,MFrm,GFrm,NFrm", "QBBBBBIII",
                       AP_HAL::micros64(),
                       _geometric_boundary_id,
                       phase,
                       uint8_t(guided_mode),
                       uint8_t(_geometric_motor_output_prepared),
                       uint8_t(allows_geometric_motor_output()),
                       copter.main_rate_controller_frames(),
                       copter.geometric_motor_output_frames(),
                       copter.native_rate_controller_frames());
#else
    (void)phase;
#endif
}

void ModeGuided::write_geometric_prearm_frame_if_needed()
{
    if (_geometric_arm_frame_logged ||
        !_geometric_prearm_snapshot_valid ||
        !motors->armed() ||
        !geometric_motor_output_options_requested()) {
        return;
    }

#if HAL_LOGGING_ENABLED
    copter.Log_Write_Geometric_Full_Lifecycle(0,
                                              _geometric_prearm_main_frames,
                                              _geometric_prearm_output_frames,
                                              _geometric_prearm_native_frames);
#endif
    _geometric_arm_frame_logged = true;
}

void ModeGuided::update_geometric_ground_safe_observer()
{
    AC_Geometric_Target geometric_target {};
    const Vector3p& pos_estimate_ned_m = pos_control->get_pos_estimate_NED_m();
    geometric_target.position_ned_m = Vector3f{float(pos_estimate_ned_m.x),
                                               float(pos_estimate_ned_m.y),
                                               float(pos_estimate_ned_m.z)};
    geometric_target.velocity_ned_ms = pos_control->get_vel_estimate_NED_ms();
    geometric_target.accel_ned_mss = Vector3f{0.0f, 0.0f, GRAVITY_MSS};
    ahrs.get_quat_body_to_ned(geometric_target.attitude_body_to_ned);
    geometric_target.omega_body_rads = ahrs.get_gyro_latest();
    geometric_target.yaw_rad = ahrs.get_yaw_rad();
    geometric_target.build_attitude_from_position = false;
    geometric_target.shape_position_target = false;
    geometric_target.shape_yaw_target = false;
    guided_geometric_heading_mode = 0;
    guided_geometric_trajectory_yaw_allowed = false;

    // Exact current-state SO(3) pass-through plus +g NED feed-forward yields
    // finite zero collective with no attitude/rate tracking error.  The rigid
    // body transport feed-forward may still request a bounded moment if the
    // measured body rate is non-zero.  AP_Motors remains responsible
    // for arm/interlock/idle/spool constraints, but is never asked to clamp a
    // nominal hover command during ground pre-warm or touchdown spool-down.
    copter.geometric_control.reset();
    const bool observer_updated = update_geometric_observer(geometric_target);
    _geometric_motor_output_prepared = observer_updated &&
                                        geometric_submode_supported() &&
                                        publish_geometric_position_reference();
    if (!_geometric_motor_output_prepared) {
        pos_control->clear_external_reference();
    }
}

void ModeGuided::update_geometric_angle_observer()
{
    _geometric_motor_output_prepared = false;
    pos_control->clear_external_reference();
    AC_Geometric_Target geometric_target {};
    const Vector3p& pos_estimate_ned_m = pos_control->get_pos_estimate_NED_m();
    geometric_target.position_ned_m = Vector3f{float(pos_estimate_ned_m.x), float(pos_estimate_ned_m.y), float(pos_estimate_ned_m.z)};
    geometric_target.velocity_ned_ms = pos_control->get_vel_estimate_NED_ms();
    geometric_target.attitude_body_to_ned = attitude_control->get_attitude_target_quat();
    geometric_target.omega_body_rads = attitude_control->get_attitude_target_ang_vel();
    geometric_target.yaw_rad = attitude_control->get_att_target_euler_rad().z;
    geometric_target.yaw_rate_rads = geometric_target.omega_body_rads.z;
    guided_geometric_heading_mode = 0;
    guided_geometric_trajectory_yaw_allowed = false;

    update_geometric_observer(geometric_target);
}

bool ModeGuided::publish_geometric_position_reference()
{
    // Publish (x_d, v_d, a_d, R_c, Omega_c) as read-only
    // compatibility state. This does not run native feedback, refresh native
    // controller-run timestamps, or independently grant actuator ownership.
    // Observer-only, rejected, disabled and rate-thread paths leave native
    // caches native-owned.
    if (!geometric_motor_output_configured() ||
        _geometric_motor_output_rejected ||
        !copter.geometric_control.output_enabled() ||
        copter.geometric_motor_output_blocked_by_rate_thread()) {
        pos_control->clear_external_reference();
        return false;
    }

    const AC_Geometric_Target& shaped_target = copter.geometric_control.get_shaped_target();
    const bool position_published = pos_control->publish_external_reference_NED_m(
        shaped_target.position_ned_m.topostype(),
        shaped_target.velocity_ned_ms,
        shaped_target.accel_ned_mss);
    const AC_Geometric_Position_Output& geometric_position =
        copter.geometric_control.get_output().position;
    if (position_published &&
        attitude_control->set_external_attitude_target(
            geometric_position.attitude_body_to_ned,
            geometric_position.omega_body_rads)) {
        return true;
    }

    // A non-finite externally owned reference is a hard geometric-path
    // failure. Keep arming blocked on the ground and latch in flight so the
    // rate loop cannot alternate between native and geometric motor output.
    pos_control->clear_external_reference();
    _geometric_motor_output_prepared = false;
    if (motors->armed() && geometric_motor_output_options_requested()) {
        _geometric_motor_output_rejected = true;
    }
    return false;
}

void ModeGuided::update_geometric_position_observer(const Vector3p* position_target_ned_m,
                                                    const Vector3f& velocity_target_ned_ms,
                                                    const Vector3f& accel_target_ned_mss,
                                                    const AC_AttitudeControl::HeadingCommand& heading,
                                                    bool shape_position_target,
                                                    bool allow_trajectory_yaw)
{
    // Convert Guided command meaning into the common geometric PVA/yaw
    // contract. Optional shaping belongs to the Guided front end; the
    // downstream feedback cascade is independent of command source.
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
    geometric_target.shape_position_target = shape_position_target;
    guided_geometric_heading_mode = (uint8_t)heading.heading_mode;
    guided_geometric_trajectory_yaw_allowed = allow_trajectory_yaw;
    const bool auto_yaw_follows_path = (auto_yaw.mode() == AutoYaw::Mode::LOOK_AT_NEXT_WP) ||
                                       (auto_yaw.mode() == AutoYaw::Mode::LOOK_AHEAD);
    geometric_target.yaw_from_trajectory = shape_position_target &&
                                           allow_trajectory_yaw &&
                                           auto_yaw_follows_path;
    // Geometric yaw-follow must not read back AC_PosControl's native yaw
    // target. When AutoYaw is in an automatic path-following mode, the
    // setpoint shaper derives yaw from its own shaped velocity/acceleration.
    if (geometric_target.yaw_from_trajectory) {
        geometric_target.yaw_rad = ahrs.get_yaw_rad();
        geometric_target.yaw_rate_rads = 0.0f;
    } else if (!allow_trajectory_yaw && auto_yaw_follows_path) {
        geometric_target.yaw_rad = ahrs.get_yaw_rad();
        geometric_target.yaw_rate_rads = 0.0f;
    } else if (heading.heading_mode == AC_AttitudeControl::HeadingMode::Rate_Only) {
        geometric_target.yaw_rad = ahrs.get_yaw_rad();
        geometric_target.yaw_rate_rads = heading.yaw_rate_rads;
    } else {
        geometric_target.yaw_rad = heading.yaw_angle_rad;
        geometric_target.yaw_rate_rads = heading.yaw_rate_rads;
    }
    geometric_target.omega_body_rads.z = geometric_target.yaw_rate_rads;

    const bool observer_updated = update_geometric_observer(geometric_target);
    _geometric_motor_output_prepared = observer_updated &&
                                        geometric_submode_supported() &&
                                        publish_geometric_position_reference();
    if (!_geometric_motor_output_prepared) {
        pos_control->clear_external_reference();
    }
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
    case SubMode::Land:
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
    case SubMode::Land:
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
    guided_pause_pos_ned_m = pos_control->get_pos_estimate_NED_m();
    guided_pause_pos_valid = true;
    guided_pause_yaw_rad = ahrs.get_yaw_rad();
    guided_pause_yaw_valid = true;
    guided_geometric_target_manager.reset();
    copter.geometric_control.reset();
    return true;
}

// resume guided mode
bool ModeGuided::resume()
{
    _paused = false;
    guided_pause_pos_valid = false;
    guided_pause_yaw_valid = false;
    return true;
}

#endif
