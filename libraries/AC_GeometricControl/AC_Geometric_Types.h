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
    // Yaw targets are kept separate so the position channel can later build
    // the full desired attitude from thrust direction plus heading.
    float yaw_rad = 0.0f;
    float yaw_rate_rads = 0.0f;
};

// Per-axis PID gains for the geometric position channel.
struct AC_Geometric_Position_Gains {
    Vector3f p;
    Vector3f i;
    Vector3f d;
};

// Per-axis PD gains for the Lee SO(3) attitude channel.
struct AC_Geometric_Attitude_Gains {
    Vector3f attitude_p;
    Vector3f omega_p;
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
};

struct AC_Geometric_Attitude_Output {
    // Lee attitude and angular-rate errors used by the PD moment calculation.
    Vector3f attitude_error;
    Vector3f omega_error_rads;
    // Geometric body-frame moment proxy. This corresponds to M_d in Gao
    // notation, but is not sent directly to AP_Motors.
    Vector3f moment;
    // Temporary body-frame compatibility output for the existing ArduPilot rate-control path.
    Vector3f rate_target_body_rads;
};

struct AC_Geometric_Output {
    AC_Geometric_Position_Output position;
    AC_Geometric_Attitude_Output attitude;
};
