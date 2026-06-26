#include "AC_GeometricControl.h"

#include <AP_HAL/AP_HAL.h>

#define AC_GEOMETRIC_POS_KX_XY_DEFAULT 1.0f
#define AC_GEOMETRIC_POS_KX_Z_DEFAULT 1.0f
#define AC_GEOMETRIC_POS_KI_XY_DEFAULT 0.0f
#define AC_GEOMETRIC_POS_KI_Z_DEFAULT 0.0f
#define AC_GEOMETRIC_POS_KV_XY_DEFAULT 2.0f
#define AC_GEOMETRIC_POS_KV_Z_DEFAULT 2.0f
#define AC_GEOMETRIC_ATT_KR_X_DEFAULT 4.0f
#define AC_GEOMETRIC_ATT_KR_Y_DEFAULT 4.0f
#define AC_GEOMETRIC_ATT_KR_Z_DEFAULT 2.0f
#define AC_GEOMETRIC_ATT_KO_X_DEFAULT 0.2f
#define AC_GEOMETRIC_ATT_KO_Y_DEFAULT 0.2f
#define AC_GEOMETRIC_ATT_KO_Z_DEFAULT 0.2f
#define AC_GEOMETRIC_HOVER_THROTTLE_DEFAULT 0.5f
#define AC_GEOMETRIC_MOMENT_NORM_X_DEFAULT 4.0f
#define AC_GEOMETRIC_MOMENT_NORM_Y_DEFAULT 4.0f
#define AC_GEOMETRIC_MOMENT_NORM_Z_DEFAULT 2.0f
#define AC_GEOMETRIC_OUTPUT_ENABLED_DEFAULT 0

const AP_Param::GroupInfo AC_GeometricControl::var_info[] = {
    // @Param: POS_KX_XY
    // @DisplayName: Geometric position horizontal Kx
    // @Description: Lee SE(3) position error gain k_x for the horizontal axes. This affects geometric observer outputs and, when geometric motor output is enabled, the active geometric position channel.
    // @Range: 0 20
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("POS_KX_XY", 1, AC_GeometricControl, _pos_kx_xy, AC_GEOMETRIC_POS_KX_XY_DEFAULT),

    // @Param: POS_KX_Z
    // @DisplayName: Geometric position vertical Kx
    // @Description: Lee SE(3) position error gain k_x for the vertical axis. This affects geometric observer outputs and, when geometric motor output is enabled, the active geometric position channel.
    // @Range: 0 20
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("POS_KX_Z", 2, AC_GeometricControl, _pos_kx_z, AC_GEOMETRIC_POS_KX_Z_DEFAULT),

    // @Param: POS_KI_XY
    // @DisplayName: Geometric position horizontal Ki
    // @Description: Integral position error gain for the horizontal axes. This is an implementation extension to the Lee position channel. It affects geometric observer outputs and, when geometric motor output is enabled, the active geometric position channel.
    // @Range: 0 5
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("POS_KI_XY", 3, AC_GeometricControl, _pos_ki_xy, AC_GEOMETRIC_POS_KI_XY_DEFAULT),

    // @Param: POS_KI_Z
    // @DisplayName: Geometric position vertical Ki
    // @Description: Integral position error gain for the vertical axis. This is an implementation extension to the Lee position channel. It affects geometric observer outputs and, when geometric motor output is enabled, the active geometric position channel.
    // @Range: 0 5
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("POS_KI_Z", 4, AC_GeometricControl, _pos_ki_z, AC_GEOMETRIC_POS_KI_Z_DEFAULT),

    // @Param: POS_KV_XY
    // @DisplayName: Geometric velocity horizontal Kv
    // @Description: Lee SE(3) velocity error gain k_v for the horizontal axes. This affects geometric observer outputs and, when geometric motor output is enabled, the active geometric position channel.
    // @Range: 0 20
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("POS_KV_XY", 5, AC_GeometricControl, _pos_kv_xy, AC_GEOMETRIC_POS_KV_XY_DEFAULT),

    // @Param: POS_KV_Z
    // @DisplayName: Geometric velocity vertical Kv
    // @Description: Lee SE(3) velocity error gain k_v for the vertical axis. This affects geometric observer outputs and, when geometric motor output is enabled, the active geometric position channel.
    // @Range: 0 20
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("POS_KV_Z", 6, AC_GeometricControl, _pos_kv_z, AC_GEOMETRIC_POS_KV_Z_DEFAULT),

    // @Param: ATT_KR_X
    // @DisplayName: Geometric attitude roll KR
    // @Description: Lee SO(3) attitude error gain k_R for the body X axis. This affects the geometric observer moment proxy and, when geometric motor output is enabled, the active geometric roll output.
    // @Range: 0 20
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("ATT_KR_X", 7, AC_GeometricControl, _att_kr_x, AC_GEOMETRIC_ATT_KR_X_DEFAULT),

    // @Param: ATT_KR_Y
    // @DisplayName: Geometric attitude pitch KR
    // @Description: Lee SO(3) attitude error gain k_R for the body Y axis. This affects the geometric observer moment proxy and, when geometric motor output is enabled, the active geometric pitch output.
    // @Range: 0 20
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("ATT_KR_Y", 8, AC_GeometricControl, _att_kr_y, AC_GEOMETRIC_ATT_KR_Y_DEFAULT),

    // @Param: ATT_KR_Z
    // @DisplayName: Geometric attitude yaw KR
    // @Description: Lee SO(3) attitude error gain k_R for the body Z axis. This affects the geometric observer moment proxy and, when geometric motor output is enabled, the active geometric yaw output.
    // @Range: 0 20
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("ATT_KR_Z", 9, AC_GeometricControl, _att_kr_z, AC_GEOMETRIC_ATT_KR_Z_DEFAULT),

    // @Param: ATT_KO_X
    // @DisplayName: Geometric angular velocity roll KOmega
    // @Description: Lee SO(3) angular velocity error gain k_Omega for the body X axis. This affects the geometric observer moment proxy and, when geometric motor output is enabled, the active geometric roll output.
    // @Range: 0 5
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("ATT_KO_X", 10, AC_GeometricControl, _att_ko_x, AC_GEOMETRIC_ATT_KO_X_DEFAULT),

    // @Param: ATT_KO_Y
    // @DisplayName: Geometric angular velocity pitch KOmega
    // @Description: Lee SO(3) angular velocity error gain k_Omega for the body Y axis. This affects the geometric observer moment proxy and, when geometric motor output is enabled, the active geometric pitch output.
    // @Range: 0 5
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("ATT_KO_Y", 11, AC_GeometricControl, _att_ko_y, AC_GEOMETRIC_ATT_KO_Y_DEFAULT),

    // @Param: ATT_KO_Z
    // @DisplayName: Geometric angular velocity yaw KOmega
    // @Description: Lee SO(3) angular velocity error gain k_Omega for the body Z axis. This affects the geometric observer moment proxy and, when geometric motor output is enabled, the active geometric yaw output.
    // @Range: 0 5
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("ATT_KO_Z", 12, AC_GeometricControl, _att_ko_z, AC_GEOMETRIC_ATT_KO_Z_DEFAULT),

    // @Param: HOV_THR
    // @DisplayName: Geometric hover throttle
    // @Description: Hover throttle reference used to normalize the geometric thrust output. This affects geometric observer logging and, when geometric motor output is enabled, the active normalized throttle output.
    // @Range: 0.05 0.95
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("HOV_THR", 13, AC_GeometricControl, _hover_throttle_norm, AC_GEOMETRIC_HOVER_THROTTLE_DEFAULT),

    // @Param: MOM_NORM_X
    // @DisplayName: Geometric roll moment normalization
    // @Description: Body X moment proxy magnitude that maps to full normalized roll actuator output. This affects geometric observer logging and, when geometric motor output is enabled, the active roll output scale.
    // @Range: 0.01 100
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("MOM_NORM_X", 14, AC_GeometricControl, _mom_norm_x, AC_GEOMETRIC_MOMENT_NORM_X_DEFAULT),

    // @Param: MOM_NORM_Y
    // @DisplayName: Geometric pitch moment normalization
    // @Description: Body Y moment proxy magnitude that maps to full normalized pitch actuator output. This affects geometric observer logging and, when geometric motor output is enabled, the active pitch output scale.
    // @Range: 0.01 100
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("MOM_NORM_Y", 15, AC_GeometricControl, _mom_norm_y, AC_GEOMETRIC_MOMENT_NORM_Y_DEFAULT),

    // @Param: MOM_NORM_Z
    // @DisplayName: Geometric yaw moment normalization
    // @Description: Body Z moment proxy magnitude that maps to full normalized yaw actuator output. This affects geometric observer logging and, when geometric motor output is enabled, the active yaw output scale.
    // @Range: 0.01 100
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("MOM_NORM_Z", 16, AC_GeometricControl, _mom_norm_z, AC_GEOMETRIC_MOMENT_NORM_Z_DEFAULT),

    // @Param: OUT_EN
    // @DisplayName: Geometric motor output enable
    // @Description: Enables the geometric controller to write normalized roll, pitch, yaw and throttle outputs to AP_Motors when the vehicle-specific mode also allows it. For Copter Guided mode, GUID_OPTIONS bit 8 must also be set. Leave disabled until the geometric controller has been validated in simulation for the vehicle and parameter set.
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO("OUT_EN", 17, AC_GeometricControl, _output_enabled, AC_GEOMETRIC_OUTPUT_ENABLED_DEFAULT),

    AP_GROUPEND
};

AC_GeometricControl::AC_GeometricControl()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AC_GeometricControl::reset()
{
    _position_pid.reset();
    _output = {};
    _last_update_ms = 0;
}

void AC_GeometricControl::set_enabled(bool enabled)
{
    if (_enabled && !enabled) {
        reset();
    }
    _enabled = enabled;
}

uint32_t AC_GeometricControl::output_age_ms(uint32_t now_ms) const
{
    if (_last_update_ms == 0) {
        return UINT32_MAX;
    }
    return now_ms - _last_update_ms;
}

bool AC_GeometricControl::output_is_fresh(uint32_t now_ms, uint32_t max_age_ms) const
{
    return output_age_ms(now_ms) <= max_age_ms;
}

void AC_GeometricControl::update_gains_from_params()
{
    AC_Geometric_Position_Gains position_gains {};
    position_gains.p = Vector3f{_pos_kx_xy.get(), _pos_kx_xy.get(), _pos_kx_z.get()};
    position_gains.i = Vector3f{_pos_ki_xy.get(), _pos_ki_xy.get(), _pos_ki_z.get()};
    position_gains.d = Vector3f{_pos_kv_xy.get(), _pos_kv_xy.get(), _pos_kv_z.get()};
    _position_pid.set_gains(position_gains);

    AC_Geometric_Attitude_Gains attitude_gains {};
    attitude_gains.attitude_p = Vector3f{_att_kr_x.get(), _att_kr_y.get(), _att_kr_z.get()};
    attitude_gains.omega_p = Vector3f{_att_ko_x.get(), _att_ko_y.get(), _att_ko_z.get()};
    _attitude_pd.set_gains(attitude_gains);
}

void AC_GeometricControl::update(const AC_Geometric_State& state,
                                 const AC_Geometric_Target& target,
                                 float dt)
{
    if (!_enabled) {
        return;
    }

    update_gains_from_params();

    // The position channel owns the SE(3) coupling: it will eventually build
    // thrust and desired attitude from position/velocity/heading targets.
    _position_pid.update(state, target, dt, _output.position);

    // Feed the position-generated desired attitude into the SO(3) attitude channel.
    AC_Geometric_Target attitude_target = target;
    attitude_target.attitude_body_to_ned = _output.position.attitude_body_to_ned;
    attitude_target.omega_body_rads = _output.position.omega_body_rads;
    attitude_target.omega_dot_body_radss = _output.position.omega_dot_body_radss;

    _attitude_pd.update(state, attitude_target, dt, _output.attitude);

    const Vector3f moment_norm {
        _mom_norm_x.get(),
        _mom_norm_y.get(),
        _mom_norm_z.get()
    };
    _output_mapper.update(_output.position, _output.attitude, _hover_throttle_norm.get(), moment_norm, _output.mapped);
    _last_update_ms = AP_HAL::millis();
}
