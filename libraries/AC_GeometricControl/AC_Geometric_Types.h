#pragma once

#include <AP_Math/AP_Math.h>

// Shared data contract between the geometric position, attitude and mapping layers.
// All quantities use SI units unless the name states otherwise.
// Frame convention:
// - Translational vectors use NED: X north, Y east, Z down.
// - Body vectors use ArduPilot FRD: X forward, Y right, Z down.
// - Attitude quaternions are body-to-NED. A matching rotation matrix has its
//   columns as body basis vectors expressed in NED.
// Paper notation is kept in comments only. Lee's SE(3) paper writes the
// position-channel resultant command as A, while Gao uses
// F_d for desired resultant force, f_d for projected total thrust, and M_d
// for desired resultant moment.
struct AC_Geometric_State {
    // Current vehicle translational state in NED.
    Vector3f position_ned_m;
    Vector3f velocity_ned_ms;
    // Current body-to-NED attitude R and body-frame angular velocity Omega.
    Quaternion attitude_body_to_ned;
    Vector3f omega_body_rads;
};

struct AC_Geometric_Target {
    // Desired translational state for the Lee SE(3) position channel.
    Vector3f position_ned_m;
    Vector3f velocity_ned_ms;
    Vector3f accel_ned_mss;
    // Desired body-to-NED attitude state R_d for direct SO(3) attitude tracking. In the
    // full SE(3) path, the position channel will generate the commanded
    // body-to-NED attitude R_c from the NED resultant force direction and yaw reference.
    Quaternion attitude_body_to_ned;
    Vector3f omega_body_rads;
    Vector3f omega_dot_body_radss;
    // False keeps direct SO(3) target pass-through for Guided angle observer.
    // True asks the position channel to construct R_c from position/velocity
    // errors, feed-forward acceleration, and yaw reference.
    bool build_attitude_from_position = false;
    // True lets AC_GeometricControl apply its optional setpoint shaper. Set
    // false when the target already comes from ArduPilot's trajectory shapers.
    bool shape_position_target = true;
    // True lets AC_GeometricControl shape explicit yaw or trajectory yaw.
    // Set false when the target yaw has already been shaped by the source
    // flight mode, as in native Loiter pilot-yaw handling.
    bool shape_yaw_target = true;
    // Yaw targets are kept separate so the position channel can later build
    // the full desired attitude from thrust direction plus heading.
    float yaw_rad = 0.0f;
    float yaw_rate_rads = 0.0f;
    // True asks the geometric shaper to derive yaw/yaw-rate from its own
    // shaped horizontal velocity/acceleration. This keeps Guided WP yaw-follow
    // independent from AC_PosControl's native yaw target.
    bool yaw_from_trajectory = false;
};

// Limits for the optional geometric reference shaper. The shaper converts raw
// Guided targets into smooth position, velocity and acceleration references
// using ArduPilot's jerk-limited square-root shaping helpers.
// Explicit yaw commands are separately gated because Copter Guided may already
// provide a shaped yaw target through AutoYaw. Geometric trajectory yaw-follow
// still uses the yaw rate/acceleration limits when yaw_from_trajectory is true.
struct AC_Geometric_Setpoint_Shaper_Limits {
    float vel_xy_max_ms = 0.0f;
    float accel_xy_max_mss = 0.0f;
    float vel_up_max_ms = 0.0f;
    float vel_down_max_ms = 0.0f;
    float accel_z_max_mss = 0.0f;
};

// Limits for the geometric yaw reference shaper. It follows ArduPilot's
// angle/velocity/acceleration shaping helper but owns its own yaw state so the
// geometric path does not depend on AC_AttitudeControl's yaw target cache.
struct AC_Geometric_Yaw_Shaper_Limits {
    bool explicit_yaw_enabled = false;
    float yaw_rate_max_rads = 0.0f;
    float yaw_accel_max_radss = 0.0f;
    float trajectory_min_speed_ms = 0.0f;
};

// Per-axis PID gains for the geometric position channel.
struct AC_Geometric_Position_Gains {
    Vector3f p;
    Vector3f i;
    Vector3f d;
    // Position geometric integral weight c_x in
    // e_XI = integral(e_v + c_x * e_x).
    Vector3f integral_error_p;
};

// Per-axis PID gains for the Lee SO(3) attitude channel.
struct AC_Geometric_Attitude_Gains {
    Vector3f attitude_p;
    Vector3f omega_p;
    // Geometric integral gains. Roll/pitch default to zero parameters for now
    // to avoid coupling attitude integral action back into the position-generated R_c.
    Vector3f attitude_i;
    Vector3f integral_error_p;
};

// Diagonal rigid-body inertia model used by the Lee SO(3) moment formula.
// Defaults follow the Gao quadrotor reference model
// J = 10^-2 diag(1.1, 2.0, 2.3) kg*m*m and should be identified per vehicle.
struct AC_Geometric_Attitude_Model {
    Vector3f inertia {0.011f, 0.020f, 0.023f};
};

// Optional first-order low-pass cutoff frequencies. A value of zero bypasses
// the corresponding filter and keeps the current unfiltered control path.
struct AC_Geometric_Position_Filter_Hz {
    float position_error = 0.0f;
    float velocity_error = 0.0f;
    // Filters applied to the position-generated commanded angular terms
    // Omega_c and dot(Omega_c) before they enter the SO(3) attitude channel.
    float omega_c = 0.0f;
    float omega_dot_c = 0.0f;
};

// Per-axis limits for the geometric position integral state e_XI. The state has
// units of m because it integrates velocity error plus c_x times position error.
struct AC_Geometric_Position_Integral_Limits {
    Vector3f integral_error_m;
};

struct AC_Geometric_Attitude_Filter_Hz {
    // Optional cutoff for the angular-rate error used by the SO(3) channel.
    float omega_error = 0.0f;
};

struct AC_Geometric_Attitude_Integral_Limits {
    // Per-axis bounds for e_I. A zero axis disables that integrator.
    Vector3f integral_error;
};

struct AC_Geometric_Position_Output {
    // Commanded body-to-NED attitude/rate passed from the position channel
    // to the attitude channel. In Lee notation this is R_c, not the external
    // desired attitude R_d.
    Quaternion attitude_body_to_ned;
    Vector3f omega_body_rads;
    Vector3f omega_dot_body_radss;
    // Placeholder collective thrust command. This corresponds to the projected
    // thrust quantity f_d in Gao notation, but is not connected to
    // ArduPilot normalized throttle yet.
    float thrust = 0.0f;
    // Resultant command per unit mass in NED. This is Lee's A/m and Gao's
    // F_d/m. Hover is approximately {0, 0, -GRAVITY_MSS}; negative Z means
    // upward because NED uses positive down.
    Vector3f specific_force_ned_mss;
    // Unnormalised ArduPilot-style thrust vector in NED used to construct R_c.
    Vector3f thrust_vector_ned;
    // Errors are exposed for logging and future Gao-style compensation terms.
    Vector3f position_error_m;
    Vector3f velocity_error_ms;
    Vector3f integral_error_m;
};

struct AC_Geometric_Attitude_Output {
    // Lee attitude and angular-rate errors used by the PD moment calculation.
    Vector3f attitude_error;
    Vector3f omega_error_rads;
    // Geometric body-frame moment proxy. This corresponds to M_d in Gao
    // notation, but is not sent directly to AP_Motors.
    Vector3f moment;
    // Geometric integral state e_I = integral(e_Omega + c*e_R).
    Vector3f integral_error;
    // Temporary body-frame compatibility output for the existing ArduPilot rate-control path.
    Vector3f rate_target_body_rads;
};

struct AC_Geometric_Mapped_Output {
    // Shadow ArduPilot-facing attitude command. This is R_c in body-to-NED
    // form and is not applied to attitude_control yet.
    Quaternion attitude_body_to_ned;
    // Shadow body-frame rate command in rad/s. This is diagnostic only.
    Vector3f rate_target_body_rads;
    // Raw normalized throttle before limiting. Computed from f_d/m using the
    // mapper hover throttle reference.
    float throttle_norm_raw = 0.0f;
    // Limited normalized throttle in ArduPilot's 0..1 command range.
    float throttle_norm = 0.0f;
    bool throttle_limited = false;
    // Shadow AP_Motors roll/pitch/yaw actuator commands. Vector axes map to
    // set_roll(), set_pitch(), and set_yaw(); raw is before limiting.
    Vector3f rpy_norm_raw;
    Vector3f rpy_norm;
    bool rpy_limited = false;
};

struct AC_Geometric_Output {
    AC_Geometric_Position_Output position;
    AC_Geometric_Attitude_Output attitude;
    AC_Geometric_Mapped_Output mapped;
};
