#pragma once

#include <AP_Math/AP_Math.h>

// Shared data contract between the geometric position, attitude and mapping layers.
// All quantities use SI units unless the name states otherwise.
// Frame convention:
// - Translational vectors use NED: X north, Y east, Z down.
// - Body vectors use ArduPilot FRD: X forward, Y right, Z down.
// - Attitude quaternions are body-to-NED. A matching rotation matrix has its
//   columns as body basis vectors expressed in NED.
// Geometric data path: X and X_d feed the position layer, which produces
// (A, R_ref, Omega_ref, dot(Omega_ref), f); the attitude layer produces
// (e_R, e_Omega, e_I^R, M); the mapper produces normalized intent u_geo.
// R_ref is position-derived R_c or, for direct SO(3) tracking, supplied R_d.
// Symbols remain comments only; storage names follow ArduPilot conventions.
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

// Geometric frontend policy remains separate from controller-neutral
// reference data while the legacy target path is retained.
struct AC_GeometricReferencePolicy {
    constexpr AC_GeometricReferencePolicy(bool build_attitude = true,
                                          bool shape_position = false,
                                          bool shape_yaw = false,
                                          bool trajectory_yaw = false) :
        build_attitude_from_position(build_attitude),
        shape_position_target(shape_position),
        shape_yaw_target(shape_yaw),
        yaw_from_trajectory(trajectory_yaw)
    {}

    bool build_attitude_from_position = true;
    bool shape_position_target = false;
    bool shape_yaw_target = false;
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
    Vector3f p; // K_x: position-error gain.
    Vector3f i; // K_I: integral-state gain.
    Vector3f d; // K_v: velocity-error gain, not a numerical derivative.
    // Position geometric integral weight C_x in
    // e_I^x = integral(e_v + C_x*e_x).
    Vector3f integral_error_p;
};

// Per-axis PID gains for the Lee SO(3) attitude channel.
struct AC_Geometric_Attitude_Gains {
    Vector3f attitude_p; // K_R: SO(3) attitude-error gain.
    Vector3f omega_p; // K_Omega: body angular-rate-error gain.
    // Geometric integral gains. Roll/pitch default to zero parameters for now
    // to avoid coupling attitude integral action back into the position-generated R_c.
    Vector3f attitude_i; // K_I: geometric attitude-integral gain.
    Vector3f integral_error_p; // C_R in integral(e_Omega + C_R*e_R).
};

// Diagonal rigid-body inertia model used by the SO(3) moment equation. These
// member initializers are the direct-construction/test fallback; Copter
// refreshes the runtime model from GEO_ATT_J_* on every controller update.
// The parameters should be identified for the actual vehicle.
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

// Per-axis limits for the geometric position integral state e_I^x. The state
// has units of m because it integrates velocity error plus C_x*e_x.
struct AC_Geometric_Position_Integral_Limits {
    Vector3f integral_error_m;
};

struct AC_Geometric_Attitude_Filter_Hz {
    // Optional cutoff for the angular-rate error used by the SO(3) channel.
    float omega_error = 0.0f;
};

struct AC_Geometric_Attitude_Integral_Limits {
    // Per-axis bounds for e_I^R. A zero axis disables that integrator.
    Vector3f integral_error;
};

struct AC_Geometric_Position_Output {
    // Body-to-NED reference attitude/rate passed to the attitude channel. The
    // coupled path supplies position-derived (R_c,Omega_c,dot(Omega_c)); the
    // direct SO(3) path passes through (R_d,Omega_d,dot(Omega_d)).
    Quaternion attitude_body_to_ned;
    Vector3f omega_body_rads;
    Vector3f omega_dot_body_radss;
    // Applied collective specific-force proxy f [m/s^2]. In the nominal domain
    // f=-A^T R e_D; boundary handling may project it or set it to zero. The
    // mapper converts f to u_T; this is neither newtons nor rotor force.
    float thrust = 0.0f;
    // Resultant command per unit mass in NED. This is Lee's A/m and Gao's
    // F_d/m. Hover is approximately {0, 0, -GRAVITY_MSS}; negative Z means
    // upward because NED uses positive down.
    Vector3f specific_force_ned_mss;
    // Feasible, regularized copy of A used to construct b3c and R_c. It may
    // differ from specific_force_ned_mss near the unidirectional or
    // near-zero-collective boundary.
    Vector3f thrust_vector_ned;
    // Errors are exposed for logging and future Gao-style compensation terms.
    Vector3f position_error_m;
    Vector3f velocity_error_ms;
    Vector3f integral_error_m;
};

struct AC_Geometric_Attitude_Output {
    // Lee SO(3) attitude-error vector e_R.
    Vector3f attitude_error;
    // Scalar SO(3) attitude diagnostics. The configuration error is
    // Psi_R = 0.5*tr(I - R_ref^T*R) in [0, 2], while the principal relative
    // rotation angle is in [0, pi]. Unlike norm(attitude_error), these
    // remain informative at the antipodal 180-degree attitude.
    float attitude_configuration_error = 0.0f;
    float attitude_error_angle_rad = 0.0f;
    Vector3f omega_error_rads; // Angular-rate error e_Omega.
    // Geometric body-frame moment proxy M. The output mapper normalizes it;
    // it is not an individual motor torque command.
    Vector3f moment;
    // Geometric integral state e_I^R = integral(e_Omega + C_R*e_R).
    Vector3f integral_error;
    // Legacy diagnostic rate-target proxy. It is not a physical angular-rate
    // reference and the active geometric path does not feed it to the native
    // rate PID.
    Vector3f rate_target_body_rads;
};

struct AC_Geometric_Mapped_Output {
    // R_ref and the legacy rate-target proxy are compatibility/diagnostic
    // mirrors; they are not passed through native attitude or rate feedback.
    Quaternion attitude_body_to_ned;
    Vector3f rate_target_body_rads;
    // Raw normalized collective intent before limiting. It is computed from
    // the scalar f using the mapper hover-throttle reference.
    float throttle_norm_raw = 0.0f;
    // Limited collective intent u_T in ArduPilot's 0..1 command range. When
    // authorized, vehicle code passes it to the normal AP_Motors-facing path.
    float throttle_norm = 0.0f;
    bool throttle_limited = false;
    // Normalized actuator intent (u_R,u_P,u_Y). On an authorized geometric
    // frame these axes feed AP_Motors set_roll(), set_pitch(), and set_yaw();
    // raw is the value before mapper limiting.
    Vector3f rpy_norm_raw;
    Vector3f rpy_norm;
    // Mapper saturation precedes AP_Motors and does not report per-motor
    // saturation or remaining mixer authority.
    bool rpy_limited = false;
};

// Snapshot of the three shared-cascade stages:
// position=(A,R_ref,Omega_ref,dot(Omega_ref),f),
// attitude=(e_R,e_Omega,e_I^R,M), and mapped=u_geo.
struct AC_Geometric_Output {
    AC_Geometric_Position_Output position;
    AC_Geometric_Attitude_Output attitude;
    AC_Geometric_Mapped_Output mapped;
};
