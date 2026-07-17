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

    // Collective mapping u_T = sat_[0,1](h*f/g). position.thrust is the
    // specific-force scalar f, and h is the normalized hover-throttle
    // reference (f ~= g at hover).
    output.throttle_norm_raw = hover_throttle * position.thrust / GRAVITY_MSS;
    output.throttle_norm = constrain_float(output.throttle_norm_raw, 0.0f, 1.0f);
    output.throttle_limited = !is_equal(output.throttle_norm, output.throttle_norm_raw);

    // u_i = sat_[-1,1](M_i/n_i), i in {roll, pitch, yaw}. The normalizers are
    // interface normalization scales, not an actuator-effectiveness matrix.
    // AP_Motors performs the downstream frame-specific mixing.
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
