#include "Copter.h"

namespace {

constexpr uint32_t GEOMETRIC_OUTPUT_MAX_AGE_MS = 100;

constexpr uint8_t GEO_FAIL_MODE = 1U << 0;
constexpr uint8_t GEO_FAIL_OUTPUT_DISABLED = 1U << 1;
constexpr uint8_t GEO_FAIL_RATE_THREAD = 1U << 2;
constexpr uint8_t GEO_FAIL_DISARMED = 1U << 3;
constexpr uint8_t GEO_FAIL_CONTROLLER_DISABLED = 1U << 4;
constexpr uint8_t GEO_FAIL_STALE = 1U << 5;
constexpr uint8_t GEO_FAIL_INVALID = 1U << 6;
constexpr uint8_t GEO_FAIL_MODE_UNSAFE = 1U << 7;

}

/*************************************************************
 *  Attitude Rate controllers and timing
 ****************************************************************/

/*
  update rate controller when run from main thread (normal operation)
*/
void Copter::run_rate_controller_main()
{
    main_rate_controller_frame_count++;

    // set attitude and position controller loop time
    const float last_loop_time_s = AP::scheduler().get_last_loop_time_s();
    pos_control->set_dt_s(last_loop_time_s);
    attitude_control->set_dt_s(last_loop_time_s);

    // Frame-exclusive arbiter: when the main loop owns rate control, exactly
    // one branch below writes roll, pitch, yaw and collective intent. The
    // rate-thread configuration is rejected by the geometric gate and writes
    // through its own native path. This is ownership selection, not blending.
    const uint8_t geometric_failure_flags = geometric_motor_output_failure_flags();
    const bool geometric_output_active = geometric_failure_flags == 0;
    if (geometric_motor_output_was_active && !geometric_output_active) {
        // Falling-edge state transfer rebuilds native attitude/rate state for
        // fallback. In the normal main-thread case, rate_controller_run()
        // below completes the handoff in this frame.
        const uint8_t mode_number = flightmode == nullptr ?
                                    UINT8_MAX :
                                    uint8_t(flightmode->mode_number());
        if (flightmode != nullptr) {
            flightmode->handle_geometric_motor_output_fallback();
        }
        geometric_motor_output_was_active = false;

#if HAL_LOGGING_ENABLED
        // @LoggerMessage: GEOH
        // @Description: Geometric motor-output handoff event, logged after mode handoff and before the native rate controller runs
        // @Field: TimeUS: Time since system startup
        // @Field: Mode: Flight mode at the output-path falling edge
        // @Field: Fail: Geometric gate failure bitmask
        // @Field: Prev: True because the previous control frame wrote geometric output
        // @Field: Act: False because geometric output is no longer authorized
        // @Field: Hand: True after the native-controller handoff completed
        // @Field: RT: True if the rate thread blocked geometric output
        AP::logger().Write("GEOH", "TimeUS,Mode,Fail,Prev,Act,Hand,RT", "QBBBBBB",
                           AP_HAL::micros64(),
                           mode_number,
                           geometric_failure_flags,
                           uint8_t(1),
                           uint8_t(0),
                           uint8_t(1),
                           uint8_t(using_rate_thread));
#endif
    }

    if (!using_rate_thread) {
        motors->set_dt_s(last_loop_time_s);
        // Only one path should write roll/pitch/yaw/throttle to AP_Motors
        // before motors_output_main() pushes the values to the HAL.
        if (geometric_output_active) {
            // Keep throttle-mix state moving for the landing detector without
            // running the native rate PIDs or writing native motor commands.
            attitude_control->rate_controller_update_throttle_mix();
            geometric_motor_output_to_motors();
            geometric_motor_output_frame_count++;
            geometric_motor_output_was_active = true;
        } else {
            // only run the rate controller if we are not using the rate thread
            attitude_control->rate_controller_run();
            native_rate_controller_frame_count++;
        }
    }
    // reset sysid and other temporary inputs
    attitude_control->rate_controller_target_reset();
}

bool Copter::geometric_motor_output_active() const
{
    return geometric_motor_output_failure_flags() == 0;
}

bool Copter::geometric_motor_output_is_valid() const
{
    const AC_Geometric_Output& output = geometric_control.get_output();
    const AC_Geometric_Mapped_Output& mapped = output.mapped;
    return !mapped.rpy_norm.is_nan() &&
           !mapped.rpy_norm.is_inf() &&
           !mapped.rpy_norm_raw.is_nan() &&
           !mapped.rpy_norm_raw.is_inf() &&
           isfinite(mapped.throttle_norm) &&
           isfinite(mapped.throttle_norm_raw) &&
           !output.position.position_error_m.is_nan() &&
           !output.position.position_error_m.is_inf() &&
           !output.position.specific_force_ned_mss.is_nan() &&
           !output.position.specific_force_ned_mss.is_inf() &&
           !output.attitude.attitude_error.is_nan() &&
           !output.attitude.attitude_error.is_inf() &&
           !output.attitude.moment.is_nan() &&
           !output.attitude.moment.is_inf();
}

uint8_t Copter::geometric_motor_output_failure_flags() const
{
    // Geometric-output authorization predicate. Every mode, output-enable,
    // scheduling, arming, controller-enable, freshness, finiteness and
    // mode-safety term must pass before geometry is eligible.
    uint8_t failure_flags = 0;
    if (flightmode == nullptr || !flightmode->allows_geometric_motor_output()) {
        failure_flags |= GEO_FAIL_MODE;
    }
    if (!geometric_control.output_enabled()) {
        failure_flags |= GEO_FAIL_OUTPUT_DISABLED;
    }
    if (geometric_motor_output_blocked_by_rate_thread()) {
        failure_flags |= GEO_FAIL_RATE_THREAD;
    }
    if (!motors->armed()) {
        failure_flags |= GEO_FAIL_DISARMED;
    }
    if (!geometric_control.enabled()) {
        failure_flags |= GEO_FAIL_CONTROLLER_DISABLED;
    }
    if (!geometric_control.output_is_fresh(millis(), GEOMETRIC_OUTPUT_MAX_AGE_MS)) {
        failure_flags |= GEO_FAIL_STALE;
    }
    if (!geometric_motor_output_is_valid()) {
        failure_flags |= GEO_FAIL_INVALID;
    }
    if (flightmode != nullptr && !flightmode->geometric_motor_output_is_safe()) {
        failure_flags |= GEO_FAIL_MODE_UNSAFE;
    }

    return failure_flags;
}

bool Copter::geometric_motor_output_blocked_by_rate_thread() const
{
    return using_rate_thread;
}

uint32_t Copter::geometric_motor_output_age_ms(uint32_t now_ms) const
{
    if (geometric_motor_output_last_ms == 0) {
        return UINT32_MAX;
    }
    return now_ms - geometric_motor_output_last_ms;
}

void Copter::geometric_motor_output_to_motors()
{
    const AC_Geometric_Mapped_Output& mapped = geometric_control.get_output().mapped;

    // Mapper/mixer boundary: mapped is normalized actuator intent
    // u_geo=(u_R,u_P,u_Y,u_T), not individual motor thrust. AP_Motors later
    // applies spool logic, saturation and the configured frame mixer to form
    // the per-motor command u_mot.
    geometric_motor_output_last_ms = millis();

    motors->set_roll(mapped.rpy_norm.x);
    // Do not add stale native feed-forward to geometric R/P/Y intent.
    motors->set_roll_ff(0.0f);
    motors->set_pitch(mapped.rpy_norm.y);
    motors->set_pitch_ff(0.0f);
    motors->set_yaw(mapped.rpy_norm.z);
    motors->set_yaw_ff(0.0f);
    // Geometry remains the sole collective command source, but use the
    // attitude-control throttle sink for AP_Motors bookkeeping (throttle-in,
    // throttle-mix headroom and landing/takeoff consumers).  This does not run
    // either the native attitude or rate feedback controller.
    attitude_control->set_throttle_out(mapped.throttle_norm, false, 0.0f);
}

/*************************************************************
 *  throttle control
 ****************************************************************/

// update estimated throttle required to hover (if necessary)
//  called at 100hz
void Copter::update_throttle_hover()
{
    // if not armed or landed or on standby then exit
    if (!motors->armed() || ap.land_complete || standby_active) {
        return;
    }

    // do not update in manual throttle modes or Drift
    if (flightmode->has_manual_throttle() || (copter.flightmode->mode_number() == Mode::Number::DRIFT)) {
        return;
    }

    // do not update while climbing or descending
    if (!is_zero(pos_control->get_vel_desired_U_ms())) {
        return;
    }

    // do not update if no vertical velocity estimate
    float vel_d_ms;
    if (!AP::ahrs().get_velocity_D(vel_d_ms, vibration_check.high_vibes)) {
        return;
    }

    // get throttle output
    float throttle = motors->get_throttle();

    // calc average throttle if we are in a level hover.  accounts for heli hover roll trim
    if ((throttle > 0.0f) && (fabsf(vel_d_ms) < 0.6) &&
        (fabsf(ahrs.get_roll_rad() - attitude_control->get_roll_trim_rad()) < radians(5)) && (fabsf(ahrs.get_pitch_rad()) < radians(5))) {
        // Can we set the time constant automatically
        motors->update_throttle_hover(0.01f);
#if HAL_GYROFFT_ENABLED
        gyro_fft.update_freq_hover(0.01f, motors->get_throttle_out());
#endif
    }
}

// get_pilot_desired_climb_rate_ms - transform pilot's throttle input to climb rate in m/s
// without any deadzone at the bottom.
// The max climb and descent rates are adjusted for any climb/descent rate currently being
// commanded by surface tracking, so full stick deflection always yields net vehicle climb
// or descent at PILOT_SPD_UP  / PILOT_SPD_DN  (see get_pilot_speed_up_adjusted_ms()).
float Copter::get_pilot_desired_climb_rate_ms()
{
    // throttle failsafe check
    if (!rc().has_valid_input()) {
        return 0.0f;
    }

    float throttle_control = copter.channel_throttle->get_control_in();

#if TOY_MODE_ENABLED
    if (g2.toy_mode.enabled()) {
        // allow throttle to be reduced after throttle arming and for
        // slower descent close to the ground
        g2.toy_mode.throttle_adjust(throttle_control);
    }
#endif

    // ensure a reasonable throttle value
    throttle_control = constrain_float(throttle_control, 0.0f, 1000.0f);

    // ensure a reasonable deadzone
    g.throttle_deadzone.set(constrain_int16(g.throttle_deadzone, 0, 400));

    float desired_rate_ms = 0.0f;
    const float mid_stick = get_throttle_mid();
    const float deadband_top = mid_stick + g.throttle_deadzone;
    const float deadband_bottom = mid_stick - g.throttle_deadzone;

    // check throttle is above, below or in the deadband
    if (throttle_control < deadband_bottom) {
        // below the deadband
        desired_rate_ms = get_pilot_speed_dn_adjusted_ms() * (throttle_control - deadband_bottom) / deadband_bottom;
    } else if (throttle_control > deadband_top) {
        // above the deadband
        desired_rate_ms = get_pilot_speed_up_adjusted_ms() * (throttle_control - deadband_top) / (1000.0 - deadband_top);
    } else {
        // must be in the deadband
        desired_rate_ms = 0.0f;
    }

    return desired_rate_ms;
}

// get_non_takeoff_throttle - a throttle somewhere between min and mid throttle which should not lead to a takeoff
float Copter::get_non_takeoff_throttle()
{
    return MAX(0,motors->get_throttle_hover() / 2.0);
}

// set_accel_throttle_I_from_pilot_throttle - smoothes transition from pilot controlled throttle to autopilot throttle
void Copter::set_accel_throttle_I_from_pilot_throttle()
{
    // get last throttle input sent to attitude controller
    float pilot_throttle = constrain_float(attitude_control->get_throttle_in(), 0.0, 1.0);
    // shift difference between pilot's throttle and hover throttle into accelerometer I
    pos_control->D_get_accel_pid().set_integrator(-(pilot_throttle - motors->get_throttle_hover()));
}

// It will return the PILOT_SPD_DN value if non zero, otherwise if zero it returns the PILOT_SPD_UP value.
float Copter::get_pilot_speed_dn_ms() const
{
    if (is_zero(g2.pilot_speed_dn_ms)) {
        return fabsf(g2.pilot_speed_up_ms);
    } else {
        return fabsf(g2.pilot_speed_dn_ms);
    }
}

// Returns the maximum pilot climb rate (m/s) adjusted for the climb/descent rate currently
// being commanded by surface tracking, to always result in at least PILOT_SPD_UP on full up stick.
float Copter::get_pilot_speed_up_adjusted_ms() const
{
    // terrain velocity in Up-positive frame (m/s)
    const float terrain_climb_ms = -pos_control->get_vel_terrain_D_ms();
    // floored at zero to prevent sign flip if terrain velocity exceeds PILOT_SPD_UP
    return MAX(0.0f, g2.pilot_speed_up_ms - terrain_climb_ms);
}

// Returns the maximum pilot descent rate (m/s, positive magnitude) adjusted for the
// climb/descent rate currently being commanded by surface tracking, to always result in
// at least PILOT_SPD_DN on full down stick.
float Copter::get_pilot_speed_dn_adjusted_ms() const
{
    // terrain velocity in Up-positive frame (m/s)
    const float terrain_climb_ms = -pos_control->get_vel_terrain_D_ms();
    // floored at zero to prevent sign flip if terrain velocity exceeds PILOT_SPD_DN
    return MAX(0.0f, get_pilot_speed_dn_ms() + terrain_climb_ms);
}
