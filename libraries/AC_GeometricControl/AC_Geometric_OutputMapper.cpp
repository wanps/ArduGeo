#include "AC_Geometric_OutputMapper.h"

namespace {

float safe_moment_norm(float norm)
{
    // Avoid division by zero while preserving the user's selected scale.
    return MAX(fabsf(norm), 1.0e-3f);
}

}

void AC_Geometric_OutputMapper::update(const AC_Geometric_Position_Output& position,
                                       const AC_Geometric_Attitude_Output& attitude,
                                       float hover_throttle_norm,
                                       const Vector3f& moment_norm,
                                       AC_Geometric_Mapped_Output& output) const
{
    const float hover_throttle = constrain_float(hover_throttle_norm, 0.0f, 1.0f);
    const Vector3f safe_norm {
        safe_moment_norm(moment_norm.x),
        safe_moment_norm(moment_norm.y),
        safe_moment_norm(moment_norm.z)
    };

    output.attitude_body_to_ned = position.attitude_body_to_ned;
    output.rate_target_body_rads = attitude.rate_target_body_rads;

    // First shadow mapping: f_d/m -> normalized throttle using hover throttle
    // as the scale reference. Hover has thrust ~= GRAVITY_MSS.
    output.throttle_norm_raw = hover_throttle * position.thrust / GRAVITY_MSS;
    output.throttle_norm = constrain_float(output.throttle_norm_raw, 0.0f, 1.0f);
    output.throttle_limited = !is_equal(output.throttle_norm, output.throttle_norm_raw);

    // M_d proxy -> AP_Motors roll/pitch/yaw command scale. This is still a
    // shadow command; AP_Motors inputs are normalized, not physical moments.
    output.rpy_norm_raw = Vector3f{
        attitude.moment.x / safe_norm.x,
        attitude.moment.y / safe_norm.y,
        attitude.moment.z / safe_norm.z
    };
    output.rpy_norm = Vector3f{
        constrain_float(output.rpy_norm_raw.x, -1.0f, 1.0f),
        constrain_float(output.rpy_norm_raw.y, -1.0f, 1.0f),
        constrain_float(output.rpy_norm_raw.z, -1.0f, 1.0f)
    };
    output.rpy_limited = !is_equal(output.rpy_norm.x, output.rpy_norm_raw.x) ||
                         !is_equal(output.rpy_norm.y, output.rpy_norm_raw.y) ||
                         !is_equal(output.rpy_norm.z, output.rpy_norm_raw.z);
}
