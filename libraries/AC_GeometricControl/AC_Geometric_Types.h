#pragma once

#include <AP_Math/AP_Math.h>

// Shared data contract between the geometric position, attitude and mapping layers.
// All quantities use SI units unless the name states otherwise.
struct AC_Geometric_State {
    // Current vehicle translational state in NED.
    Vector3f position_ned_m;
    Vector3f velocity_ned_ms;
    // Current passive body-to-NED attitude and body-frame angular velocity.
    Quaternion attitude_body_to_ned;
    Vector3f omega_body_rads;
};

struct AC_Geometric_Target {
    // Desired translational state for the Lee SE(3) position channel.
    Vector3f position_ned_m;
    Vector3f velocity_ned_ms;
    Vector3f accel_ned_mss;
    // Desired attitude state for direct SO(3) attitude tracking or for the
    // attitude target produced by the position channel.
    Quaternion attitude_body_to_ned;
    Vector3f omega_body_rads;
    Vector3f omega_dot_body_radss;
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
    // Desired attitude/rate passed from the position channel to the attitude channel.
    Quaternion attitude_body_to_ned;
    Vector3f omega_body_rads;
    Vector3f omega_dot_body_radss;
    // Placeholder collective thrust command. The ArduPilot throttle mapping is not connected yet.
    float thrust = 0.0f;
    // Errors are exposed for logging and for future SANM/adaptive compensation.
    Vector3f position_error_m;
    Vector3f velocity_error_ms;
};

struct AC_Geometric_Attitude_Output {
    // Lee attitude and angular-rate errors used by the PD moment calculation.
    Vector3f attitude_error;
    Vector3f omega_error_rads;
    // Geometric body-moment proxy. This is not sent directly to AP_Motors.
    Vector3f moment;
    // Temporary compatibility output for the existing ArduPilot rate-control path.
    Vector3f rate_target_body_rads;
};

struct AC_Geometric_Output {
    AC_Geometric_Position_Output position;
    AC_Geometric_Attitude_Output attitude;
};
