#include "AC_Geometric_OutputMapper.h"

#define AC_GEOMETRIC_HOVER_THROTTLE_DEFAULT 0.0f
#define AC_GEOMETRIC_MOMENT_NORM_X_DEFAULT 4.0f
#define AC_GEOMETRIC_MOMENT_NORM_Y_DEFAULT 4.0f
#define AC_GEOMETRIC_MOMENT_NORM_Z_DEFAULT 2.0f

const AP_Param::GroupInfo AC_Geometric_OutputMapper_Params::var_info[] = {
    // Local indices mirror the pre-module flat layout for reviewability. The
    // nested storage identity is new; legacy_var_info performs the upgrade copy.

    // @Param: HOV_THR
    // @DisplayName: Geometric hover throttle
    // @Description: Hover throttle reference used to normalize the geometric thrust output. Zero uses the vehicle hover throttle estimate from MOT_THST_HOVER. Non-zero values override that vehicle hover throttle reference. This affects geometric observer logging and, when geometric motor output is enabled, the active normalized throttle output.
    // @Range: 0 0.95
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("HOV_THR", 13, AC_Geometric_OutputMapper_Params, _hover_throttle_norm, AC_GEOMETRIC_HOVER_THROTTLE_DEFAULT),

    // @Param: MOM_NORM_X
    // @DisplayName: Geometric roll moment normalization
    // @Description: Body X moment proxy magnitude that maps to full normalized roll actuator output. This affects geometric observer logging and, when geometric motor output is enabled, the active roll output scale.
    // @Range: 0.01 100
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("MOM_NORM_X", 14, AC_Geometric_OutputMapper_Params, _moment_norm_x, AC_GEOMETRIC_MOMENT_NORM_X_DEFAULT),

    // @Param: MOM_NORM_Y
    // @DisplayName: Geometric pitch moment normalization
    // @Description: Body Y moment proxy magnitude that maps to full normalized pitch actuator output. This affects geometric observer logging and, when geometric motor output is enabled, the active pitch output scale.
    // @Range: 0.01 100
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("MOM_NORM_Y", 15, AC_Geometric_OutputMapper_Params, _moment_norm_y, AC_GEOMETRIC_MOMENT_NORM_Y_DEFAULT),

    // @Param: MOM_NORM_Z
    // @DisplayName: Geometric yaw moment normalization
    // @Description: Body Z moment proxy magnitude that maps to full normalized yaw actuator output. This affects geometric observer logging and, when geometric motor output is enabled, the active yaw output scale.
    // @Range: 0.01 100
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("MOM_NORM_Z", 16, AC_Geometric_OutputMapper_Params, _moment_norm_z, AC_GEOMETRIC_MOMENT_NORM_Z_DEFAULT),

    AP_GROUPEND
};

const AP_Param::GroupInfo AC_Geometric_OutputMapper_Params::legacy_var_info[] = {
    AP_GROUPINFO("", 13, AC_Geometric_OutputMapper_Params, _hover_throttle_norm, 0.0f),
    AP_GROUPINFO("", 14, AC_Geometric_OutputMapper_Params, _moment_norm_x, 0.0f),
    AP_GROUPINFO("", 15, AC_Geometric_OutputMapper_Params, _moment_norm_y, 0.0f),
    AP_GROUPINFO("", 16, AC_Geometric_OutputMapper_Params, _moment_norm_z, 0.0f),
    AP_GROUPEND
};

AC_Geometric_OutputMapper_Params::AC_Geometric_OutputMapper_Params()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AC_Geometric_OutputMapper_Params::convert_legacy_params(uint16_t old_key)
{
    AP_Param::convert_class(old_key, this, legacy_var_info, 0, true);
}

Vector3f AC_Geometric_OutputMapper_Params::moment_norm() const
{
    return Vector3f{_moment_norm_x.get(), _moment_norm_y.get(), _moment_norm_z.get()};
}

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
