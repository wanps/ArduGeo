#include "AC_Geometric_OutputMapper.h"

void AC_Geometric_OutputMapper::update(const AC_Geometric_Position_Output& position,
                                       const AC_Geometric_Attitude_Output& attitude,
                                       float hover_throttle_norm,
                                       AC_Geometric_Mapped_Output& output) const
{
    const float hover_throttle = constrain_float(hover_throttle_norm, 0.0f, 1.0f);

    output.attitude_body_to_ned = position.attitude_body_to_ned;
    output.rate_target_body_rads = attitude.rate_target_body_rads;

    // First shadow mapping: f_d/m -> normalized throttle using hover throttle
    // as the scale reference. Hover has thrust ~= GRAVITY_MSS.
    output.throttle_norm_raw = hover_throttle * position.thrust / GRAVITY_MSS;
    output.throttle_norm = constrain_float(output.throttle_norm_raw, 0.0f, 1.0f);
    output.throttle_limited = !is_equal(output.throttle_norm, output.throttle_norm_raw);
}
