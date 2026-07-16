#include "Copter.h"

#if MODE_LOITER_ENABLED

/*
 * Init and run calls for loiter flight mode
 */

static constexpr uint32_t loiter_geometric_output_recent_ms = 100;
static constexpr float loiter_geometric_avoidance_kp = 1.0f;
static constexpr float loiter_geometric_takeoff_position_tolerance_m = 0.02f;
static constexpr float loiter_geometric_takeoff_velocity_tolerance_ms = 0.05f;

// loiter_init - initialise loiter controller
bool ModeLoiter::init(bool ignore_checks)
{
    float target_roll_rad, target_pitch_rad;
    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, loiter_nav->get_angle_max_rad(), attitude_control->get_althold_lean_angle_max_rad());

    // process pilot's roll and pitch input
    loiter_nav->set_pilot_desired_acceleration_rad(target_roll_rad, target_pitch_rad);

    loiter_nav->init_target();

    // initialise the vertical position controller
    if (!pos_control->D_is_active()) {
        pos_control->D_init_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->D_set_max_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());
    pos_control->D_set_correction_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());

#if AC_PRECLAND_ENABLED
    _precision_loiter_active = false;
#endif
    _geometric_motor_output_active = false;
    _geometric_motor_output_rejected = false;
    reset_geometric_lifecycle();
    _geometric_log_counter = 0;
    _geometric_reference_frames = 0;
    _native_reference_frames = 0;
    _geometric_reference.reset();
    _geometric_reference_status = {};
    _geometric_takeoff_target_z_ned_m = 0.0f;
    _geometric_takeoff_target_valid = false;
    pos_control->clear_external_reference();
    init_geometric_ekf_reset_tracking();
    copter.geometric_control.set_enabled(false);
    copter.geometric_control.reset();

    // Pre-warm before the first Loiter rate frame.  A ground entry uses the
    // zero-thrust current-state target; an airborne mode entry must instead
    // initialise from Loiter's current stopping/PVA target so it never inserts
    // a zero-collective frame in flight.
    const AltHoldModeState initial_state = motors->armed() && !copter.ap.land_complete ?
                                           AltHoldModeState::Flying :
                                           AltHoldModeState::MotorStopped;
    bool takeoff_stopped_this_cycle = false;
    bool ground_safe_prepared = false;
    const bool dedicated_prepared = loiter_nav->geometric_motor_output_enabled() &&
                                    geometric_reference_supported() &&
                                    run_geometric_loiter_reference(initial_state,
                                                                   target_roll_rad,
                                                                   target_pitch_rad,
                                                                   0.0f,
                                                                   0.0f,
                                                                   takeoff_stopped_this_cycle,
                                                                   ground_safe_prepared);
    if (!dedicated_prepared) {
        update_geometric_observer(initial_state);
    }

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: GELI
    // @Description: Geometric Loiter mode-entry output and exact frame-counter snapshot
    // @Field: TimeUS: Time since system startup
    // @Field: Air: True if Loiter was entered while airborne
    // @Field: Act: True if geometric motor output was prepared and active
    // @Field: Thr: Prepared normalized geometric throttle
    // @Field: MFrm: Cumulative main-loop rate-controller frames
    // @Field: GFrm: Cumulative geometric motor-output frames
    // @Field: NFrm: Cumulative native rate-controller frames
    AP::logger().Write("GELI", "TimeUS,Air,Act,Thr,MFrm,GFrm,NFrm", "QBBfIII",
                       AP_HAL::micros64(),
                       uint8_t(initial_state == AltHoldModeState::Flying),
                       uint8_t(_geometric_motor_output_active),
                       (double)copter.geometric_control.get_output().mapped.throttle_norm,
                       copter.main_rate_controller_frames(),
                       copter.geometric_motor_output_frames(),
                       copter.native_rate_controller_frames());
#endif

    return true;
}

void ModeLoiter::exit()
{
    // Copter::set_mode() initialises the new mode before calling the old
    // mode's exit hook.  Do not disable/reset the shared geometric controller
    // here: a Loiter -> Guided transition may already have prepared its first
    // airborne output.  The next mode's gate (or the rate-loop fallback) owns
    // shared-controller cleanup.
    _geometric_motor_output_active = false;
    _geometric_motor_output_rejected = false;
    reset_geometric_lifecycle();
    _geometric_reference.reset();
    _geometric_reference_status = {};
    _geometric_takeoff_target_z_ned_m = 0.0f;
    _geometric_takeoff_target_valid = false;
}

#if AC_PRECLAND_ENABLED
bool ModeLoiter::do_precision_loiter()
{
    if (!_precision_loiter_enabled) {
        return false;
    }
    if (copter.ap.land_complete_maybe) {
        return false;        // don't move on the ground
    }
    // if the pilot *really* wants to move the vehicle, let them....
    if (loiter_nav->get_pilot_desired_acceleration_NE_mss().length() > 0.5) {
        return false;
    }
    if (!copter.precland.target_acquired()) {
        return false; // we don't have a good vector
    }
    return true;
}

void ModeLoiter::precision_loiter_xy()
{
    loiter_nav->clear_pilot_desired_acceleration();
    Vector2p target_pos_ne_m;
    Vector2f target_vel_ne_ms;
    if (!copter.precland.get_target_position_m(target_pos_ne_m)) {
        target_pos_ne_m = pos_control->get_pos_estimate_NED_m().xy();
    }
    // get the velocity of the target
    copter.precland.get_target_velocity_ms(pos_control->get_vel_estimate_NED_ms().xy(), target_vel_ne_ms);

    Vector2f zero;
    // target vel will remain zero if landing target is stationary
    pos_control->input_pos_vel_accel_NE_m(target_pos_ne_m, target_vel_ne_ms, zero);
    // run pos controller
    pos_control->NE_update_controller();
}
#endif

// loiter_run - runs the loiter controller
// should be called at 100hz or more
void ModeLoiter::run()
{
    float target_roll_rad, target_pitch_rad;
    float target_yaw_rate_rads = 0.0f;
    float target_climb_rate_ms = 0.0f;
    bool geometric_takeoff_stopped_this_cycle = false;
    bool geometric_ground_safe_prepared = false;

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, loiter_nav->get_angle_max_rad(), attitude_control->get_althold_lean_angle_max_rad());

    // get pilot's desired yaw rate
    target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads();

    // get pilot desired climb rate
    target_climb_rate_ms = get_pilot_desired_climb_rate_ms();

    // _TakeOff is shared across altitude-hold modes and its running flag is
    // intentionally independent of arming.  A manual disarm during the first
    // takeoff frames must terminate that epoch without stop() inferring
    // airborne state from the previous throttle sample; otherwise a same-mode
    // re-arm would enter Takeoff even with the stick low.
    if (!motors->armed()) {
        takeoff.reset();
    }

    // Loiter State Machine Determination
    AltHoldModeState loiter_state = get_alt_hold_state_D_ms(target_climb_rate_ms);

    const bool dedicated_requested = loiter_nav->geometric_motor_output_enabled();
    const bool dedicated_supported = geometric_reference_supported();

    // Native disarm-on-land normally fires as soon as land_complete changes.
    // Full-geometric Loiter deliberately blocks that early disarm so it can
    // retain rate-output ownership through a controlled spool-down.  Once a
    // real geometric lifecycle has landed, honour the same PILOT_THR_BHV
    // policy even if the pilot has returned the throttle stick to centre:
    // request GROUND_IDLE here, then finish_geometric_lifecycle() disarms only
    // after AP_Motors confirms that state.
    if (dedicated_requested &&
        dedicated_supported &&
        !_geometric_motor_output_rejected &&
        _geometric_lifecycle_in_progress &&
        _geometric_liftoff_confirmed &&
        copter.ap.land_complete &&
        (g.throttle_behavior & THR_BEHAVE_DISARM_ON_LAND_DETECT) != 0) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        loiter_state = motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE ?
                       AltHoldModeState::Landed_Ground_Idle :
                       AltHoldModeState::Landed_Pre_Takeoff;
    }

    if (dedicated_requested &&
        dedicated_supported &&
        !_geometric_motor_output_rejected) {
        if (run_geometric_loiter_reference(loiter_state,
                                           target_roll_rad,
                                           target_pitch_rad,
                                           target_yaw_rate_rads,
                                           target_climb_rate_ms,
                                           geometric_takeoff_stopped_this_cycle,
                                           geometric_ground_safe_prepared)) {
            finish_geometric_lifecycle(loiter_state,
                                       geometric_takeoff_stopped_this_cycle,
                                       geometric_ground_safe_prepared);
            return;
        }

        // A malformed reference or output is a hard fault.  Rebuild the
        // native controllers before this same frame falls through to them and
        // latch the rejection until the operator clears the request or
        // re-enters Loiter.
        _geometric_motor_output_rejected = true;
        copter.gcs().send_text(MAV_SEVERITY_WARNING,
                               "Loiter: geometric reference rejected");
        if (_geometric_motor_output_active) {
            deactivate_geometric_motor_output(true);
        } else {
            loiter_nav->init_target();
            pos_control->D_init_controller();
        }
    } else if (dedicated_requested && _geometric_motor_output_active) {
        // Precision Loiter and active surface tracking are not yet represented
        // by the dedicated reference.  Hand off atomically instead of running
        // either feature against stale geometric targets.
        _geometric_motor_output_rejected = true;
        copter.gcs().send_text(MAV_SEVERITY_WARNING,
                               "Loiter: geometric feature unsupported");
        deactivate_geometric_motor_output(true);
    }

    // Native fallback/observer path.  Every controller call below is skipped
    // by the successful dedicated branch above.
    pos_control->D_set_max_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());
    loiter_nav->set_pilot_desired_acceleration_rad(target_roll_rad, target_pitch_rad);
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Loiter State Machine
    switch (loiter_state) {

    case AltHoldModeState::MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate();
        pos_control->D_relax_controller(0.0f);   // forces throttle output to decay to zero
        loiter_nav->init_target();
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        loiter_nav->init_target();
        pos_control->D_relax_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHoldModeState::Takeoff: {
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start_m(constrain_float(g2.pilot_takeoff_alt_m, 0.0, 10.0));
        }

        // get avoidance adjusted climb rate
        target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);

        const bool takeoff_was_running = takeoff.running();
        takeoff.do_pilot_takeoff_ms(target_climb_rate_ms);
        geometric_takeoff_stopped_this_cycle = takeoff_was_running && !takeoff.running();

        // run loiter controller
        loiter_nav->update();
        break;
    }

    case AltHoldModeState::Flying:
#if AC_PRECLAND_ENABLED
        bool precision_loiter_old_state = _precision_loiter_active;
        if (do_precision_loiter()) {
            precision_loiter_xy();
            _precision_loiter_active = true;
        } else {
            _precision_loiter_active = false;
        }
        if (precision_loiter_old_state && !_precision_loiter_active) {
            // prec loiter was active, not any more, let's init again as user takes control
            loiter_nav->init_target();
        }
        // run loiter controller if we are not doing prec loiter
        if (!_precision_loiter_active) {
            loiter_nav->update();
        }
#else
        loiter_nav->update();
#endif


        // get avoidance adjusted climb rate
        target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);

#if AP_RANGEFINDER_ENABLED
        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();
#endif

        // Send the commanded climb rate to the position controller
        pos_control->D_set_pos_target_from_climb_rate_ms(target_climb_rate_ms);
        break;
    }

    // call attitude controller
    attitude_control->input_thrust_vector_rate_heading_rads(loiter_nav->get_thrust_vector(), target_yaw_rate_rads, false);
    // run the vertical position controller and set output throttle
    pos_control->D_update_controller();

    _native_reference_frames++;
    update_geometric_observer(loiter_state);
    finish_geometric_lifecycle(loiter_state,
                               geometric_takeoff_stopped_this_cycle,
                               false);
}

bool ModeLoiter::geometric_motor_output_requested() const
{
    if (!loiter_nav->geometric_motor_output_enabled() ||
        !copter.geometric_control.output_enabled() ||
        copter.geometric_motor_output_blocked_by_rate_thread() ||
        copter.is_tradheli() ||
        !geometric_reference_supported()) {
        return false;
    }

    return true;
}

bool ModeLoiter::geometric_reference_supported() const
{
#if AC_PRECLAND_ENABLED
    // Precision Loiter owns a moving external XY reference.  It remains on
    // the native path until that source is represented explicitly by the
    // dedicated geometric reference contract.
    if (_precision_loiter_enabled) {
        return false;
    }
#endif

#if AP_RANGEFINDER_ENABLED
    // Surface tracking owns a measured terrain/ceiling offset PVA.  Falling
    // back is safer than silently dropping that offset from the geometric Z
    // reference.
    if (copter.surface_tracking.active()) {
        return false;
    }
#endif

    return true;
}

bool ModeLoiter::reset_geometric_reference(bool ground_safe)
{
    (void)ground_safe;
    AC_Geometric_State state {};
    state.position_ned_m = pos_control->get_pos_estimate_NED_m().tofloat();
    state.velocity_ned_ms = pos_control->get_vel_estimate_NED_ms();
    ahrs.get_quat_body_to_ned(state.attitude_body_to_ned);
    state.omega_body_rads = ahrs.get_gyro_latest();
    const bool reset_ok = _geometric_reference.reset(state, ahrs.get_yaw_rad(), 0.0f);
    if (reset_ok) {
        init_geometric_ekf_reset_tracking();
    }
    return reset_ok;
}

void ModeLoiter::init_geometric_ekf_reset_tracking()
{
    Vector2f position_shift_ne_m;
    float position_shift_d_m = 0.0f;
    float yaw_shift_rad = 0.0f;
    _geometric_ekf_ne_reset_ms = ahrs.getLastPosNorthEastReset(position_shift_ne_m);
    _geometric_ekf_d_reset_ms = ahrs.getLastPosDownReset(position_shift_d_m);
    _geometric_ekf_yaw_reset_ms = ahrs.getLastYawResetAngle(yaw_shift_rad);
}

bool ModeLoiter::handle_geometric_ekf_resets()
{
    Vector3f position_shift_ned_m;
    float yaw_shift_rad = 0.0f;
    bool shifted = false;

    Vector2f position_shift_ne_m;
    const uint32_t ne_reset_ms = ahrs.getLastPosNorthEastReset(position_shift_ne_m);
    if (ne_reset_ms != 0 && ne_reset_ms != _geometric_ekf_ne_reset_ms) {
        position_shift_ned_m.x = position_shift_ne_m.x;
        position_shift_ned_m.y = position_shift_ne_m.y;
        _geometric_ekf_ne_reset_ms = ne_reset_ms;
        shifted = true;
    }

    float position_shift_d_m = 0.0f;
    const uint32_t d_reset_ms = ahrs.getLastPosDownReset(position_shift_d_m);
    if (d_reset_ms != 0 && d_reset_ms != _geometric_ekf_d_reset_ms) {
        position_shift_ned_m.z = position_shift_d_m;
        if (_geometric_takeoff_target_valid) {
            _geometric_takeoff_target_z_ned_m += position_shift_d_m;
        }
        _geometric_ekf_d_reset_ms = d_reset_ms;
        shifted = true;
    }

    const uint32_t yaw_reset_ms = ahrs.getLastYawResetAngle(yaw_shift_rad);
    if (yaw_reset_ms == 0 || yaw_reset_ms == _geometric_ekf_yaw_reset_ms) {
        yaw_shift_rad = 0.0f;
    } else {
        _geometric_ekf_yaw_reset_ms = yaw_reset_ms;
        shifted = true;
    }

    return !shifted || _geometric_reference.shift_reference(position_shift_ned_m,
                                                             yaw_shift_rad);
}

AC_Geometric_LoiterReference_Limits ModeLoiter::geometric_reference_limits() const
{
    const AC_Loiter::ReferenceConfig loiter_config = loiter_nav->get_reference_config();
    const AC_Geometric_LoiterReference_Profile profile =
        copter.geometric_control.get_loiter_reference_profile();

    float ekf_ground_speed_limit_ms = loiter_config.speed_max_ne_ms;
    float ahrs_control_scale_xy = 1.0f;
    ahrs.getControlLimits(ekf_ground_speed_limit_ms, ahrs_control_scale_xy);
    if (!isfinite(ekf_ground_speed_limit_ms) ||
        !is_positive(ekf_ground_speed_limit_ms)) {
        ekf_ground_speed_limit_ms = loiter_config.speed_max_ne_ms;
    }
    if (!isfinite(ahrs_control_scale_xy)) {
        ahrs_control_scale_xy = 1.0f;
    }
    const float control_scale_xy = constrain_float(ahrs_control_scale_xy, 0.0f, 1.0f);
    const float lean_angle_limit_rad = MIN(loiter_nav->get_angle_max_rad(),
                                           attitude_control->get_althold_lean_angle_max_rad());
    const float physical_accel_xy_max_mss =
        MAX(GRAVITY_MSS * tanf(lean_angle_limit_rad), 0.1f);

    AC_Geometric_LoiterReference_Limits limits;
    limits.speed_xy_max_ms = MAX(MIN(MIN(loiter_config.speed_max_ne_ms,
                                         profile.speed_xy_max_ms),
                                     ekf_ground_speed_limit_ms),
                                   0.1f);
    limits.accel_xy_max_mss = MAX(MIN(profile.accel_xy_max_mss * control_scale_xy,
                                      physical_accel_xy_max_mss),
                                  0.1f);
    limits.accel_xy_total_max_mss = physical_accel_xy_max_mss;
    limits.jerk_xy_max_msss = profile.jerk_xy_max_msss;
    limits.brake_delay_s = profile.brake_delay_s;
    limits.brake_accel_max_mss = MIN(profile.brake_accel_max_mss,
                                     physical_accel_xy_max_mss);
    limits.brake_jerk_max_msss = profile.brake_jerk_max_msss;
    limits.speed_up_max_ms = get_pilot_speed_up_ms();
    limits.speed_down_max_ms = get_pilot_speed_dn_ms();
    limits.accel_z_max_mss = get_pilot_accel_D_mss();
    limits.jerk_z_max_msss = profile.jerk_z_max_msss;
    limits.brake_accel_z_max_mss = MIN(profile.brake_accel_z_max_mss,
                                       limits.accel_z_max_mss);
    limits.brake_jerk_z_max_msss = profile.brake_jerk_z_max_msss;
    limits.yaw_rate_max_rads = MAX(radians(fabsf(g2.command_model_pilot_y.get_rate())), 0.01f);
    limits.yaw_accel_max_radss = profile.yaw_accel_max_radss;
    limits.yaw_jerk_max_radsss = profile.yaw_jerk_max_radsss;
    limits.yaw_brake_accel_max_radss = profile.yaw_brake_accel_max_radss;
    limits.yaw_brake_jerk_max_radsss = profile.yaw_brake_jerk_max_radsss;
    return limits;
}

void ModeLoiter::build_geometric_ground_safe_target(AC_Geometric_Target& target) const
{
    target = {};
    target.position_ned_m = pos_control->get_pos_estimate_NED_m().tofloat();
    // Match the measured velocity as well as position so estimator noise does
    // not create a synthetic -Kv*v command during pre-arm or touchdown
    // spool-down.  This makes the +g target a true zero-collective pass-through.
    target.velocity_ned_ms = pos_control->get_vel_estimate_NED_ms();
    // +g NED cancels gravity in the geometric position channel, producing a
    // finite zero-collective command while AP_Motors owns spool/idle state.
    target.accel_ned_mss = Vector3f{0.0f, 0.0f, GRAVITY_MSS};
    ahrs.get_quat_body_to_ned(target.attitude_body_to_ned);
    target.omega_body_rads = ahrs.get_gyro_latest();
    target.build_attitude_from_position = false;
    target.shape_position_target = false;
    target.shape_yaw_target = false;
    target.yaw_rad = ahrs.get_yaw_rad();
}

bool ModeLoiter::run_geometric_loiter_reference(AltHoldModeState loiter_state,
                                                float target_roll_rad,
                                                float target_pitch_rad,
                                                float target_yaw_rate_rads,
                                                float target_climb_rate_ms,
                                                bool& takeoff_stopped_this_cycle,
                                                bool& ground_safe_prepared)
{
    takeoff_stopped_this_cycle = false;
    ground_safe_prepared = false;

    const bool ground_safe = loiter_state == AltHoldModeState::MotorStopped ||
                             loiter_state == AltHoldModeState::Landed_Ground_Idle ||
                             loiter_state == AltHoldModeState::Landed_Pre_Takeoff;
    AC_Geometric_Target target {};
    if (ground_safe) {
        _geometric_takeoff_target_valid = false;
        if (!reset_geometric_reference(true)) {
            return false;
        }
        _geometric_reference_status = {};
        build_geometric_ground_safe_target(target);
        copter.geometric_control.reset();
        ground_safe_prepared = true;
        _geometric_reference_frames++;
        return update_geometric_observer(loiter_state, &target) &&
               _geometric_motor_output_active;
    }

    if (!_geometric_reference.initialized() && !reset_geometric_reference(false)) {
        return false;
    }
    if (!handle_geometric_ekf_resets()) {
        return false;
    }

    if (loiter_state == AltHoldModeState::Takeoff) {
        if (!takeoff.running()) {
            takeoff.start_m(constrain_float(g2.pilot_takeoff_alt_m, 0.0f, 10.0f));
            _geometric_takeoff_target_valid = false;
        }
        if (!_geometric_takeoff_target_valid) {
            _geometric_takeoff_target_z_ned_m = -takeoff.get_complete_alt_U_m();
            _geometric_takeoff_target_valid = true;
        }
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        if (!_geometric_lifecycle_in_progress) {
            _geometric_lifecycle_in_progress = true;
            _geometric_liftoff_confirmed = false;
            _geometric_touchdown_logged = false;
            _geometric_takeoff_start_alt_m = pos_control->get_pos_estimate_U_m();
            copter.geometric_control.reset();
            write_geometric_lifecycle_frame(1);
        }

        // A negative climb command cancels the user takeoff.  reset() is used
        // deliberately: _TakeOff::stop() reads the native throttle cache to
        // infer flight, which is not the owner of this reference.
        if (is_negative(target_climb_rate_ms)) {
            takeoff.reset();
            _geometric_takeoff_target_valid = false;
            takeoff_stopped_this_cycle = true;
            if (copter.ap.land_complete) {
                if (!reset_geometric_reference(true)) {
                    return false;
                }
                _geometric_reference_status = {};
                build_geometric_ground_safe_target(target);
                copter.geometric_control.reset();
                ground_safe_prepared = true;
                _geometric_reference_frames++;
                return update_geometric_observer(AltHoldModeState::Landed_Pre_Takeoff,
                                                  &target) &&
                       _geometric_motor_output_active;
            }
        }
    }

    const AC_Loiter::ReferenceConfig loiter_config = loiter_nav->get_reference_config();
    AC_Geometric_LoiterReference_Input input;
    input.pilot_accel_ne_mss =
        pos_control->lean_angles_rad_to_accel_NED_mss(
            Vector3f{target_roll_rad, target_pitch_rad, ahrs.get_yaw_rad()}).xy();
    input.pilot_xy_active = rc().has_valid_input() &&
                            (!is_zero(channel_roll->norm_input_dz()) ||
                             !is_zero(channel_pitch->norm_input_dz()));
    input.pilot_yaw_active = rc().has_valid_input() &&
                             !is_zero(channel_yaw->norm_input_dz());
    input.coordinated_turn = loiter_config.coordinated_turn_enabled;
    input.climb_rate_up_ms = target_climb_rate_ms;
    input.pilot_z_active = rc().has_valid_input() &&
                           !is_zero(target_climb_rate_ms);
    input.yaw_rate_rads = target_yaw_rate_rads;
    if (loiter_state == AltHoldModeState::Takeoff &&
        takeoff.running() &&
        !is_negative(target_climb_rate_ms)) {
        input.use_z_position_target = true;
        input.position_z_target_ned_m = _geometric_takeoff_target_z_ned_m;
    }

    AC_Geometric_LoiterReference_Limits limits = geometric_reference_limits();
    if (input.use_z_position_target && is_positive(target_climb_rate_ms)) {
        // Preserve the native manual-takeoff contract: throttle-stick
        // deflection controls the climb-speed boundary instead of merely
        // acting as a binary trigger for a fixed-speed trajectory.
        limits.speed_up_max_ms = MIN(limits.speed_up_max_ms,
                                     target_climb_rate_ms);
    }
    if (!_geometric_reference.update(input,
                                     limits,
                                     G_Dt,
                                     target,
                                     _geometric_reference_status)) {
        return false;
    }

#if AP_AVOIDANCE_ENABLED
    AC_Avoid* avoid = AP::ac_avoid();
    if (avoid != nullptr && avoid->enabled()) {
        Vector3f adjusted_velocity_ned_ms = target.velocity_ned_ms;
        avoid->adjust_velocity_NED_m(adjusted_velocity_ned_ms,
                                    loiter_geometric_avoidance_kp,
                                    limits.accel_xy_total_max_mss,
                                    loiter_geometric_avoidance_kp,
                                    limits.accel_z_max_mss,
                                    G_Dt);
        if (!_geometric_reference.apply_velocity_constraint(adjusted_velocity_ned_ms,
                                                             target,
                                                             _geometric_reference_status)) {
            return false;
        }
    }
#endif

    // A velocity-controlled descent must not integrate its position reference
    // indefinitely through the ground.  Anchor Z to the measured position
    // while the shaped reference is descending; velocity and acceleration
    // remain dedicated, bounded geometric references.  This is the same
    // closed-loop landing boundary used by the pure-geometric Guided landing
    // path and lets the standard land detector observe a level, low-throttle
    // contact instead of an inverted target many metres below the vehicle.
    if (loiter_state == AltHoldModeState::Flying &&
        is_positive(_geometric_reference.velocity_ref_ned_ms().z) &&
        !_geometric_reference.anchor_position_z(pos_control->get_pos_estimate_NED_m().z,
                                                 target,
                                                 _geometric_reference_status)) {
        return false;
    }

    if (loiter_state == AltHoldModeState::Takeoff && takeoff.running()) {
        const float position_error_z_m =
            fabsf(float(_geometric_reference.position_ref_ned_m().z) -
                  _geometric_takeoff_target_z_ned_m);
        if (position_error_z_m <= loiter_geometric_takeoff_position_tolerance_m &&
            fabsf(_geometric_reference.velocity_ref_ned_ms().z) <=
                loiter_geometric_takeoff_velocity_tolerance_ms) {
            takeoff.reset();
            _geometric_takeoff_target_valid = false;
            takeoff_stopped_this_cycle = true;
        }
    }

    _geometric_reference_frames++;
    return update_geometric_observer(loiter_state, &target) &&
           _geometric_motor_output_active;
}

void ModeLoiter::finish_geometric_lifecycle(AltHoldModeState loiter_state,
                                            bool takeoff_stopped_this_cycle,
                                            bool ground_safe_prepared)
{
    // A pilot can cancel takeoff before liftoff.  Replace any last non-zero
    // target synchronously, then close that lifecycle epoch.
    if (takeoff_stopped_this_cycle &&
        _geometric_lifecycle_in_progress &&
        !_geometric_liftoff_confirmed) {
        if (copter.ap.land_complete) {
            if (!ground_safe_prepared) {
                update_geometric_observer(AltHoldModeState::Landed_Pre_Takeoff);
            }
            write_geometric_lifecycle_frame(5);
            reset_geometric_lifecycle();
        } else {
            _geometric_liftoff_confirmed = true;
        }
    }

    if (_geometric_lifecycle_in_progress &&
        _geometric_liftoff_confirmed &&
        copter.ap.land_complete &&
        !_geometric_touchdown_logged &&
        motors->armed()) {
        _geometric_touchdown_logged = true;
        write_geometric_lifecycle_frame(3);
    }

    if (_geometric_lifecycle_in_progress &&
        _geometric_touchdown_logged &&
        loiter_state == AltHoldModeState::Landed_Ground_Idle &&
        motors->armed() &&
        motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
        if (!_geometric_ground_idle_logged) {
            write_geometric_lifecycle_frame(4);
            _geometric_ground_idle_logged = true;
        }
        // Preserve PILOT_THR_BHV semantics.  Geometry delays the optional
        // disarm until the safe GROUND_IDLE boundary, but must not turn an
        // armed-after-landing configuration into an unconditional disarm. If
        // disarming is requested, retain the lifecycle until it succeeds so a
        // transient refusal is retried instead of silently abandoning the
        // landed epoch.
        if ((g.throttle_behavior & THR_BEHAVE_DISARM_ON_LAND_DETECT) != 0) {
            if (copter.arming.disarm(AP_Arming::Method::LANDED)) {
                reset_geometric_lifecycle();
            }
        } else {
            reset_geometric_lifecycle();
        }
    }
}

bool ModeLoiter::allows_arming(AP_Arming::Method method) const
{
    if (!loiter_nav->geometric_motor_output_requested() ||
        copter.is_tradheli() ||
        copter.geometric_motor_output_blocked_by_rate_thread()) {
        // Full-geometric motor output is intentionally unsupported on these
        // platforms.  LOIT_OPTIONS keeps its common default, but Loiter must
        // retain the established native arming/landing behaviour.
        return true;
    }

    // Full-geometric Loiter keeps ownership after land detection until
    // AP_Motors reaches GROUND_IDLE, then explicitly disarms.  Prevent the
    // optional immediate-on-land path from truncating that spool-down window.
    // Once a hard fault has deactivated geometric output, allow the native
    // landing path to complete its normal disarm instead of leaving the
    // vehicle armed indefinitely.
    if (method == AP_Arming::Method::LANDING) {
        return !_geometric_motor_output_active;
    }

    // A raw full-output request without its observer prerequisite is an
    // invalid configuration, not an instruction to arm silently on the
    // native rate PID.  Keep LANDING above this check so an in-flight malformed
    // parameter change cannot prevent the native fallback from disarming.
    if (!loiter_nav->geometric_motor_output_enabled()) {
        return false;
    }

    return _geometric_motor_output_active &&
           geometric_motor_output_requested() &&
           copter.geometric_control.enabled() &&
           copter.geometric_control.output_is_fresh(AP_HAL::millis(), loiter_geometric_output_recent_ms) &&
           geometric_output_safe_for_active(false);
}

bool ModeLoiter::prepare_for_arming(AP_Arming::Method method)
{
    if (method != AP_Arming::Method::LANDING &&
        loiter_nav->geometric_motor_output_requested() &&
        !copter.is_tradheli() &&
        !copter.geometric_motor_output_blocked_by_rate_thread()) {
        bool takeoff_stopped_this_cycle = false;
        bool ground_safe_prepared = false;
        if (!geometric_reference_supported() ||
            _geometric_motor_output_rejected ||
            !run_geometric_loiter_reference(AltHoldModeState::MotorStopped,
                                             0.0f,
                                             0.0f,
                                             0.0f,
                                             0.0f,
                                             takeoff_stopped_this_cycle,
                                             ground_safe_prepared)) {
            copter.gcs().send_text(MAV_SEVERITY_WARNING,
                                   "Loiter: geometric pre-arm failed");
            update_geometric_observer(AltHoldModeState::MotorStopped);
        }
    }
    const bool armable = allows_arming(method);
    if (armable &&
        method != AP_Arming::Method::LANDING &&
        loiter_nav->geometric_motor_output_enabled() &&
        !copter.is_tradheli() &&
        !copter.geometric_motor_output_blocked_by_rate_thread()) {
        write_geometric_lifecycle_frame(0);
    }
    return armable;
}

bool ModeLoiter::geometric_output_safe_for_active(bool allow_safety_bypass) const
{
    (void)allow_safety_bypass;
    if (!copter.geometric_motor_output_is_valid()) {
        return false;
    }
    const float cos_tilt = ahrs.cos_roll() * ahrs.cos_pitch();
    if (!isfinite(cos_tilt)) {
        return false;
    }
    // Saturation and large-but-finite tracking errors are normal control
    // boundaries.  The mapper already constrains actuator commands, so they
    // must not silently transfer the aircraft back to the native rate PID.
    return true;
}

void ModeLoiter::reset_geometric_lifecycle()
{
    _geometric_lifecycle_in_progress = false;
    _geometric_liftoff_confirmed = false;
    _geometric_touchdown_logged = false;
    _geometric_ground_idle_logged = false;
    _geometric_takeoff_start_alt_m = 0.0f;
    _geometric_takeoff_target_z_ned_m = 0.0f;
    _geometric_takeoff_target_valid = false;
}

void ModeLoiter::deactivate_geometric_motor_output(bool reset_loiter_targets)
{
    // Fail closed before touching the native controller state.
    _geometric_motor_output_active = false;
    pos_control->clear_external_reference();

    Mode::handle_geometric_motor_output_fallback();

    if (reset_loiter_targets) {
        // A hard/operator fallback terminates the exact-frame lifecycle.  A
        // later explicit re-entry must begin from a new takeoff phase instead
        // of inheriting stale touchdown or liftoff state.
        reset_geometric_lifecycle();
        _geometric_reference.reset();
        _geometric_reference_status = {};

        // Native position controllers did not run during dedicated geometric
        // frames.  Re-enter Loiter at the current stopping point.
        loiter_nav->init_target();
        pos_control->D_init_controller();
    }
}

void ModeLoiter::handle_geometric_motor_output_fallback()
{
    if (!_geometric_motor_output_active) {
        return;
    }

    // Disarming is the normal end of a lifecycle, not a controller fault.
    // The next disarmed Loiter update will immediately pre-warm a fresh
    // ground-safe output for the following arming cycle.
    if (!motors->armed()) {
        _geometric_motor_output_rejected = false;
        reset_geometric_lifecycle();
        deactivate_geometric_motor_output(false);
        return;
    }

    // A hard fault while bit 2 remains requested must not automatically
    // re-enter.  Clearing the operator request, or leaving/re-entering Loiter,
    // clears this latch.
    copter.gcs().send_text(MAV_SEVERITY_WARNING,
                           "Loiter: geometric output handoff");
    _geometric_motor_output_rejected = loiter_nav->geometric_motor_output_enabled();
    deactivate_geometric_motor_output(true);
}

void ModeLoiter::write_geometric_lifecycle_frame(uint8_t phase) const
{
#if HAL_LOGGING_ENABLED
    copter.Log_Write_Geometric_Loiter_Lifecycle(
        phase,
        copter.main_rate_controller_frames(),
        copter.geometric_motor_output_frames(),
        copter.native_rate_controller_frames(),
        copter.geometric_control.get_output().mapped.throttle_norm);
#else
    (void)phase;
#endif
}

bool ModeLoiter::update_geometric_observer(AltHoldModeState loiter_state,
                                           const AC_Geometric_Target* reference_target)
{
    const bool dedicated_reference = reference_target != nullptr;
    const bool motor_output_option_requested = loiter_nav->geometric_motor_output_enabled();
    const bool motor_output_requested = geometric_motor_output_requested();
    if (!motor_output_option_requested) {
        if (_geometric_motor_output_active) {
            deactivate_geometric_motor_output(true);
        }
        _geometric_motor_output_rejected = false;
    } else if (!motor_output_requested) {
        if (_geometric_motor_output_active) {
            _geometric_motor_output_rejected = true;
            deactivate_geometric_motor_output(true);
        }
    }

    // A dedicated reference may publish compatibility PVA/attitude only
    // while the full-geometric path owns actuator intent.  Observer-only,
    // rejected, output-disabled and rate-thread paths remain native-owned.
    if (dedicated_reference &&
        (!motor_output_requested || _geometric_motor_output_rejected)) {
        pos_control->clear_external_reference();
        return false;
    }

    const bool flying = (loiter_state == AltHoldModeState::Takeoff) ||
                        (loiter_state == AltHoldModeState::Flying);
    const bool enabled = loiter_nav->geometric_observer_enabled() &&
                         (flying || motor_output_option_requested);
    if (!enabled) {
        if (dedicated_reference) {
            pos_control->clear_external_reference();
        }
        copter.geometric_control.set_enabled(false);
        return false;
    }

    AC_Geometric_Target target = dedicated_reference ? *reference_target : AC_Geometric_Target{};
    const bool ground_safe = loiter_state == AltHoldModeState::MotorStopped ||
                             loiter_state == AltHoldModeState::Landed_Ground_Idle ||
                             loiter_state == AltHoldModeState::Landed_Pre_Takeoff;
    if (!dedicated_reference && ground_safe) {
        // A position hold target with zero acceleration would request hover
        // thrust even before takeoff.  Direct SO(3) pass-through plus +g NED
        // feed-forward cancels gravity in the position channel and produces a
        // finite zero-thrust command with no attitude/rate tracking error.
        // Rigid-body transport feed-forward may retain a bounded moment at a
        // non-zero measured body rate.  AP_Motors owns arming, interlock, idle
        // and spool constraints.
        const Vector3p& position_estimate_ned_m = pos_control->get_pos_estimate_NED_m();
        target.position_ned_m = Vector3f{float(position_estimate_ned_m.x),
                                        float(position_estimate_ned_m.y),
                                        float(position_estimate_ned_m.z)};
        target.velocity_ned_ms = pos_control->get_vel_estimate_NED_ms();
        target.accel_ned_mss = Vector3f{0.0f, 0.0f, GRAVITY_MSS};
        ahrs.get_quat_body_to_ned(target.attitude_body_to_ned);
        target.omega_body_rads = ahrs.get_gyro_latest();
        target.build_attitude_from_position = false;
        copter.geometric_control.reset();
    } else if (!dedicated_reference) {
        const Vector3p& position_target_ned_m = pos_control->get_pos_target_NED_m();
        target.position_ned_m = Vector3f{float(position_target_ned_m.x),
                                        float(position_target_ned_m.y),
                                        float(position_target_ned_m.z)};
        target.velocity_ned_ms = pos_control->get_vel_desired_NED_ms();
        target.accel_ned_mss = pos_control->get_accel_desired_NED_mss();
        target.omega_body_rads = attitude_control->get_attitude_target_ang_vel();
        target.build_attitude_from_position = true;
        target.yaw_rad = attitude_control->get_att_target_euler_rad().z;
        target.yaw_rate_rads = attitude_control->get_rate_ef_target_rads().z;
    }
    target.shape_position_target = false;
    target.shape_yaw_target = false;

    AC_Geometric_State state {};
    if (!run_geometric_observer(target, true, state)) {
        if (dedicated_reference) {
            pos_control->clear_external_reference();
        }
        return false;
    }

    if (dedicated_reference) {
        // Several vehicle-level safety and telemetry consumers read these
        // caches.  Mirror the independently generated reference without
        // running any native position, attitude or rate feedback controller.
        if (!pos_control->publish_external_reference_NED_m(target.position_ned_m.topostype(),
                                                           target.velocity_ned_ms,
                                                           target.accel_ned_mss)) {
            pos_control->clear_external_reference();
            _geometric_motor_output_rejected = true;
            if (_geometric_motor_output_active) {
                deactivate_geometric_motor_output(true);
            }
            return false;
        }
        const AC_Geometric_Position_Output& geometric_position =
            copter.geometric_control.get_output().position;
        if (!attitude_control->set_external_attitude_target(
                geometric_position.attitude_body_to_ned,
                geometric_position.omega_body_rads)) {
            pos_control->clear_external_reference();
            _geometric_motor_output_rejected = true;
            if (_geometric_motor_output_active) {
                deactivate_geometric_motor_output(true);
            }
            return false;
        }
    }

    if (motor_output_requested && !_geometric_motor_output_rejected) {
        if (!geometric_output_safe_for_active(false)) {
            if (dedicated_reference) {
                pos_control->clear_external_reference();
            }
            _geometric_motor_output_rejected = true;
            if (_geometric_motor_output_active) {
                deactivate_geometric_motor_output(true);
            }
            return false;
        }
        if (!_geometric_motor_output_active) {
            attitude_control->reset_rate_controller_I_terms();
            _geometric_motor_output_active = true;
        }
    }

    // Entering or explicitly recovering full-geometric Loiter while already
    // airborne has no Takeoff state in which to create phase 1.  Start an
    // airborne lifecycle here so a later touchdown still owns the complete
    // geometric spool-down and explicit disarm path.
    if (_geometric_motor_output_active &&
        loiter_state == AltHoldModeState::Flying &&
        !_geometric_lifecycle_in_progress) {
        _geometric_lifecycle_in_progress = true;
        _geometric_liftoff_confirmed = true;
        _geometric_touchdown_logged = false;
        _geometric_takeoff_start_alt_m = pos_control->get_pos_estimate_U_m();
        write_geometric_lifecycle_frame(2);
    }

    if (loiter_state == AltHoldModeState::Takeoff &&
        copter.ap.land_complete &&
        motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED &&
        _geometric_motor_output_active &&
        copter.geometric_control.get_output().mapped.throttle_norm > get_non_takeoff_throttle()) {
        set_land_complete(false);
        _geometric_liftoff_confirmed = true;
    }

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: GEOL
    // @Description: Geometric Loiter target and output-gate observer
    // @Field: TimeUS: Time since system startup
    // @Field: St: Loiter altitude-control state
    // @Field: Act: True if geometric motor output is active
    // @Field: Wrote: True if the geometric path recently wrote AP_Motors
    // @Field: Shp: True if a geometric target shaper was active
    // @Field: PX: Selected Loiter position reference, X-Axis
    // @Field: PY: Selected Loiter position reference, Y-Axis
    // @Field: PZ: Selected Loiter position reference, Z-Axis
    // @Field: VX: Selected Loiter velocity reference, X-Axis
    // @Field: VY: Selected Loiter velocity reference, Y-Axis
    // @Field: VZ: Selected Loiter velocity reference, Z-Axis
    // @Field: AX: Selected Loiter acceleration reference, X-Axis
    // @Field: AY: Selected Loiter acceleration reference, Y-Axis
    // @Field: AZ: Selected Loiter acceleration reference, Z-Axis
    // @Field: Yaw: Selected Loiter yaw reference
    // @Field: YRN: Selected Loiter yaw-rate reference

    // @LoggerMessage: GELR
    // @Description: Controller-independent geometric Loiter PVA reference and shaping state
    // @Field: TimeUS: Time since system startup
    // @Field: St: Loiter altitude-control state
    // @Field: Brk: Delayed horizontal braking is active
    // @Field: XL: Horizontal speed was hard limited
    // @Field: ZL: Vertical speed was hard limited
    // @Field: Ext: External avoidance adjusted the final velocity
    // @Field: Anc: Vertical position was anchored to the measured position
    // @Field: PX: Dedicated reference position, X-Axis
    // @Field: PY: Dedicated reference position, Y-Axis
    // @Field: PZ: Dedicated reference position, Z-Axis
    // @Field: VX: Dedicated reference velocity, X-Axis
    // @Field: VY: Dedicated reference velocity, Y-Axis
    // @Field: VZ: Dedicated reference velocity, Z-Axis
    // @Field: AX: Dedicated reference acceleration, X-Axis
    // @Field: AY: Dedicated reference acceleration, Y-Axis
    // @Field: AZ: Dedicated reference acceleration, Z-Axis

    // @LoggerMessage: GELO
    // @Description: Controller-independent geometric Loiter yaw reference and ownership counters
    // @Field: TimeUS: Time since system startup
    // @Field: YL: Yaw rate was limited
    // @Field: YB: Dedicated yaw braking is active
    // @Field: ZB: Dedicated vertical braking is active
    // @Field: ZS: Vertical braking has settled at zero speed and the stick remains neutral
    // @Field: NER: NE position reference is fresh from either controller source
    // @Field: DR: D position reference is fresh from either controller source
    // @Field: NEN: Native NE position feedback controller ran recently
    // @Field: DN: Native D position feedback controller ran recently
    // @Field: Yaw: Dedicated yaw reference
    // @Field: YR: Dedicated yaw-rate reference
    // @Field: TFrm: Total Loiter reference frames
    // @Field: GFrm: Dedicated geometric reference frames
    // @Field: NFrm: Native Loiter reference frames

    // @LoggerMessage: GELC
    // @Description: Geometric and native Loiter position-control comparison
    // @Field: TimeUS: Time since system startup
    // @Field: PEx: Geometric position error, X-Axis
    // @Field: PEy: Geometric position error, Y-Axis
    // @Field: PEz: Geometric position error, Z-Axis
    // @Field: SFx: Geometric specific force command, X-Axis
    // @Field: SFy: Geometric specific force command, Y-Axis
    // @Field: SFz: Geometric specific force command, Z-Axis
    // @Field: Thr: Geometric projected total thrust per mass
    // @Field: RCr: Geometric commanded attitude roll
    // @Field: RCp: Geometric commanded attitude pitch
    // @Field: ATr: Native ArduPilot attitude target roll
    // @Field: ATp: Native ArduPilot attitude target pitch
    // @Field: TVx: Native ArduPilot thrust vector, X-Axis
    // @Field: TVy: Native ArduPilot thrust vector, Y-Axis
    // @Field: TVz: Native ArduPilot thrust vector, Z-Axis

    // GEOX uses the mode-generic motor-output status schema documented by
    // ModeGuided::update_geometric_observer().
    if (_geometric_log_counter++ % 5 == 0) {
        const AC_Geometric_Output& output = copter.geometric_control.get_output();
        const uint32_t now_ms = AP_HAL::millis();
        const uint32_t geometric_age_ms = copter.geometric_control.output_age_ms(now_ms);
        const uint32_t motor_output_age_ms = copter.geometric_motor_output_age_ms(now_ms);
        const bool motor_output_allowed = allows_geometric_motor_output();
        const bool geometric_output_enabled = copter.geometric_control.output_enabled();
        const bool rate_thread_active = copter.geometric_motor_output_blocked_by_rate_thread();
        const bool motor_output_written_recently = motor_output_age_ms <= loiter_geometric_output_recent_ms;
        AP::logger().WriteStreaming("GEOL", "TimeUS,St,Act,Wrote,Shp,PX,PY,PZ,VX,VY,VZ,AX,AY,AZ,Yaw,YRN", "QBBBBfffffffffff",
                                    AP_HAL::micros64(),
                                    (uint8_t)loiter_state,
                                    (uint8_t)copter.geometric_motor_output_active(),
                                    (uint8_t)motor_output_written_recently,
                                    (uint8_t)copter.geometric_control.shaper_active(),
                                    (double)target.position_ned_m.x,
                                    (double)target.position_ned_m.y,
                                    (double)target.position_ned_m.z,
                                    (double)target.velocity_ned_ms.x,
                                    (double)target.velocity_ned_ms.y,
                                    (double)target.velocity_ned_ms.z,
                                    (double)target.accel_ned_mss.x,
                                    (double)target.accel_ned_mss.y,
                                    (double)target.accel_ned_mss.z,
                                    (double)target.yaw_rad,
                                    (double)target.yaw_rate_rads);

        if (dedicated_reference) {
            // AP_Logger dynamic messages are limited to 16 fields and a
            // 64-character label string.  Keep the high-rate PVA state in
            // GELR and publish yaw/ownership counters separately in GELO.
            AP::logger().WriteStreaming("GELR", "TimeUS,St,Brk,XL,ZL,Ext,Anc,PX,PY,PZ,VX,VY,VZ,AX,AY,AZ", "QBBBBBBfffffffff",
                                        AP_HAL::micros64(),
                                        (uint8_t)loiter_state,
                                        (uint8_t)_geometric_reference_status.braking,
                                        (uint8_t)_geometric_reference_status.speed_xy_limited,
                                        (uint8_t)_geometric_reference_status.speed_z_limited,
                                        (uint8_t)_geometric_reference_status.velocity_constraint_applied,
                                        (uint8_t)_geometric_reference_status.vertical_position_anchored,
                                        (double)target.position_ned_m.x,
                                        (double)target.position_ned_m.y,
                                        (double)target.position_ned_m.z,
                                        (double)target.velocity_ned_ms.x,
                                        (double)target.velocity_ned_ms.y,
                                        (double)target.velocity_ned_ms.z,
                                        (double)target.accel_ned_mss.x,
                                        (double)target.accel_ned_mss.y,
                                        (double)target.accel_ned_mss.z);
            AP::logger().WriteStreaming("GELO", "TimeUS,YL,YB,ZB,ZS,NER,DR,NEN,DN,Yaw,YR,TFrm,GFrm,NFrm", "QBBBBBBBBffIII",
                                        AP_HAL::micros64(),
                                        (uint8_t)_geometric_reference_status.yaw_rate_limited,
                                        (uint8_t)_geometric_reference_status.yaw_braking,
                                        (uint8_t)_geometric_reference_status.z_braking,
                                        (uint8_t)_geometric_reference_status.z_settled,
                                        (uint8_t)pos_control->NE_reference_is_active(),
                                        (uint8_t)pos_control->D_reference_is_active(),
                                        (uint8_t)pos_control->NE_is_active(),
                                        (uint8_t)pos_control->D_is_active(),
                                        (double)target.yaw_rad,
                                        (double)target.yaw_rate_rads,
                                        _geometric_reference_frames + _native_reference_frames,
                                        _geometric_reference_frames,
                                        _native_reference_frames);
        }

        // @LoggerMessage: GELT
        // @Description: Full-geometric Loiter takeoff progress
        // @Field: TimeUS: Time since system startup
        // @Field: Air: True after the geometric path clears land complete
        // @Field: DAlt: Altitude gained from the start of Loiter takeoff
        // @Field: VU: Estimated upward velocity
        const float takeoff_altitude_gain_m = _geometric_lifecycle_in_progress ?
            pos_control->get_pos_estimate_U_m() - _geometric_takeoff_start_alt_m : 0.0f;
        AP::logger().WriteStreaming("GELT", "TimeUS,Air,DAlt,VU", "QBff",
                                    AP_HAL::micros64(),
                                    (uint8_t)_geometric_liftoff_confirmed,
                                    (double)takeoff_altitude_gain_m,
                                    (double)pos_control->get_vel_estimate_U_ms());

        if (!dedicated_reference) {
            Vector3f geometric_attitude_rad;
            output.position.attitude_body_to_ned.to_euler(geometric_attitude_rad.x,
                                                          geometric_attitude_rad.y,
                                                          geometric_attitude_rad.z);
            const Vector3f native_attitude_target_rad = attitude_control->get_att_target_euler_rad();
            const Vector3f native_thrust_vector_ned = pos_control->get_thrust_vector();
            AP::logger().WriteStreaming("GELC", "TimeUS,PEx,PEy,PEz,SFx,SFy,SFz,Thr,RCr,RCp,ATr,ATp,TVx,TVy,TVz", "Qffffffffffffff",
                                        AP_HAL::micros64(),
                                        (double)output.position.position_error_m.x,
                                        (double)output.position.position_error_m.y,
                                        (double)output.position.position_error_m.z,
                                        (double)output.position.specific_force_ned_mss.x,
                                        (double)output.position.specific_force_ned_mss.y,
                                        (double)output.position.specific_force_ned_mss.z,
                                        (double)output.position.thrust,
                                        (double)geometric_attitude_rad.x,
                                        (double)geometric_attitude_rad.y,
                                        (double)native_attitude_target_rad.x,
                                        (double)native_attitude_target_rad.y,
                                        (double)native_thrust_vector_ned.x,
                                        (double)native_thrust_vector_ned.y,
                                        (double)native_thrust_vector_ned.z);
        }

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

float ModeLoiter::wp_distance_m() const
{
    return loiter_nav->get_distance_to_target_m();
}

float ModeLoiter::wp_bearing_deg() const
{
    return degrees(loiter_nav->get_bearing_to_target_rad());
}

#endif
