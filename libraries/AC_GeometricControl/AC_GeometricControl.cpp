#include "AC_GeometricControl.h"

#include <AP_HAL/AP_HAL.h>

#define AC_GEOMETRIC_POS_KX_XY_DEFAULT 1.0f
#define AC_GEOMETRIC_POS_KX_Z_DEFAULT 1.0f
#define AC_GEOMETRIC_POS_KI_XY_DEFAULT 0.0f
#define AC_GEOMETRIC_POS_KI_Z_DEFAULT 0.0f
#define AC_GEOMETRIC_POS_KV_XY_DEFAULT 2.0f
#define AC_GEOMETRIC_POS_KV_Z_DEFAULT 2.0f
#define AC_GEOMETRIC_POS_IMAX_XY_DEFAULT 1.0f
#define AC_GEOMETRIC_POS_IMAX_Z_DEFAULT 1.0f
#define AC_GEOMETRIC_POS_INT_C_DEFAULT 1.0f
#define AC_GEOMETRIC_ATT_KR_X_DEFAULT 4.0f
#define AC_GEOMETRIC_ATT_KR_Y_DEFAULT 4.0f
#define AC_GEOMETRIC_ATT_KR_Z_DEFAULT 2.0f
#define AC_GEOMETRIC_ATT_KO_X_DEFAULT 0.2f
#define AC_GEOMETRIC_ATT_KO_Y_DEFAULT 0.2f
#define AC_GEOMETRIC_ATT_KO_Z_DEFAULT 0.4f
#define AC_GEOMETRIC_ATT_KI_X_DEFAULT 0.0f
#define AC_GEOMETRIC_ATT_KI_Y_DEFAULT 0.0f
#define AC_GEOMETRIC_ATT_KI_Z_DEFAULT 0.0f
#define AC_GEOMETRIC_ATT_IMAX_X_DEFAULT 0.0f
#define AC_GEOMETRIC_ATT_IMAX_Y_DEFAULT 0.0f
#define AC_GEOMETRIC_ATT_IMAX_Z_DEFAULT 0.0f
#define AC_GEOMETRIC_ATT_INT_C_DEFAULT 1.0f
#define AC_GEOMETRIC_ATT_J_X_DEFAULT 0.011f
#define AC_GEOMETRIC_ATT_J_Y_DEFAULT 0.020f
#define AC_GEOMETRIC_ATT_J_Z_DEFAULT 0.023f
#define AC_GEOMETRIC_HOVER_THROTTLE_DEFAULT 0.0f
#define AC_GEOMETRIC_MOMENT_NORM_X_DEFAULT 4.0f
#define AC_GEOMETRIC_MOMENT_NORM_Y_DEFAULT 4.0f
#define AC_GEOMETRIC_MOMENT_NORM_Z_DEFAULT 2.0f
#define AC_GEOMETRIC_OUTPUT_ENABLED_DEFAULT 1
#define AC_GEOMETRIC_FILTER_DISABLED 0.0f
#define AC_GEOMETRIC_OMEGA_C_FILTER_DEFAULT 5.0f
#define AC_GEOMETRIC_OMEGA_DOT_C_FILTER_DEFAULT 2.0f
#define AC_GEOMETRIC_SHAPE_ENABLED_DEFAULT 1
#define AC_GEOMETRIC_SHAPE_VEL_XY_DEFAULT 1.0f
#define AC_GEOMETRIC_SHAPE_ACCEL_XY_DEFAULT 0.5f
#define AC_GEOMETRIC_SHAPE_VEL_UP_DEFAULT 2.5f
#define AC_GEOMETRIC_SHAPE_VEL_DOWN_DEFAULT 1.5f
#define AC_GEOMETRIC_SHAPE_ACCEL_Z_DEFAULT 1.0f
#define AC_GEOMETRIC_SHAPE_YAW_ENABLED_DEFAULT 0
#define AC_GEOMETRIC_SHAPE_YAW_RATE_DEFAULT 1.0f
#define AC_GEOMETRIC_SHAPE_YAW_ACCEL_DEFAULT 1.0f

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
    // @Description: Geometric position integral gain k_i for the horizontal axes. This multiplies e_XI = integral(e_v + POS_INT_C*e_x). It affects geometric observer outputs and, when geometric motor output is enabled, the active geometric position channel.
    // @Range: 0 5
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("POS_KI_XY", 3, AC_GeometricControl, _pos_ki_xy, AC_GEOMETRIC_POS_KI_XY_DEFAULT),

    // @Param: POS_KI_Z
    // @DisplayName: Geometric position vertical Ki
    // @Description: Geometric position integral gain k_i for the vertical axis. This multiplies e_XI = integral(e_v + POS_INT_C*e_x). It affects geometric observer outputs and, when geometric motor output is enabled, the active geometric position channel.
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
    // @Description: Hover throttle reference used to normalize the geometric thrust output. Zero uses the vehicle hover throttle estimate from MOT_THST_HOVER. Non-zero values override that vehicle hover throttle reference. This affects geometric observer logging and, when geometric motor output is enabled, the active normalized throttle output.
    // @Range: 0 0.95
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

    // @Param: POS_FLTE
    // @DisplayName: Geometric position error filter
    // @Description: Optional first-order low-pass cutoff applied to Lee position error before the geometric position PID channel. A value of zero disables this filter and preserves the raw state-error path.
    // @Range: 0 50
    // @Units: Hz
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("POS_FLTE", 18, AC_GeometricControl, _pos_error_filt_hz, AC_GEOMETRIC_FILTER_DISABLED),

    // @Param: VEL_FLTE
    // @DisplayName: Geometric velocity error filter
    // @Description: Optional first-order low-pass cutoff applied to Lee velocity error before the geometric position PID channel. A value of zero disables this filter and preserves the raw state-error path.
    // @Range: 0 50
    // @Units: Hz
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("VEL_FLTE", 19, AC_GeometricControl, _vel_error_filt_hz, AC_GEOMETRIC_FILTER_DISABLED),

    // @Param: OMG_FLTE
    // @DisplayName: Geometric angular velocity error filter
    // @Description: Optional first-order low-pass cutoff applied to Lee angular velocity error before the SO(3) attitude PD channel. A value of zero disables this filter and preserves the raw angular-rate error path.
    // @Range: 0 100
    // @Units: Hz
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("OMG_FLTE", 20, AC_GeometricControl, _omega_error_filt_hz, AC_GEOMETRIC_FILTER_DISABLED),

    // @Param: POS_IMAX_XY
    // @DisplayName: Geometric position horizontal integrator limit
    // @Description: Limit applied to the horizontal geometric position integral state e_XI before the Ki term is applied. The integral state has units of meters because it integrates velocity error plus POS_INT_C times position error. A value of zero disables horizontal integral accumulation.
    // @Range: 0 20
    // @Units: m
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("POS_IMAX_XY", 21, AC_GeometricControl, _pos_imax_xy, AC_GEOMETRIC_POS_IMAX_XY_DEFAULT),

    // @Param: POS_IMAX_Z
    // @DisplayName: Geometric position vertical integrator limit
    // @Description: Limit applied to the vertical geometric position integral state e_XI before the Ki term is applied. The integral state has units of meters because it integrates velocity error plus POS_INT_C times position error. A value of zero disables vertical integral accumulation.
    // @Range: 0 20
    // @Units: m
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("POS_IMAX_Z", 22, AC_GeometricControl, _pos_imax_z, AC_GEOMETRIC_POS_IMAX_Z_DEFAULT),

    // @Param: ATT_KI_Z
    // @DisplayName: Geometric yaw integral gain
    // @Description: Yaw geometric integral gain applied to the attitude-channel integral state e_Iz. This term is intended to reject slow yaw bias and drift.
    // @Range: 0 5
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("ATT_KI_Z", 23, AC_GeometricControl, _att_ki_z, AC_GEOMETRIC_ATT_KI_Z_DEFAULT),

    // @Param: ATT_IMAX_Z
    // @DisplayName: Geometric yaw integrator limit
    // @Description: Limit applied to the yaw attitude integral state before the ATT_KI_Z term is applied. A value of zero disables yaw integral accumulation.
    // @Range: 0 20
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("ATT_IMAX_Z", 24, AC_GeometricControl, _att_imax_z, AC_GEOMETRIC_ATT_IMAX_Z_DEFAULT),

    // @Param: ATT_INT_C
    // @DisplayName: Geometric attitude integral error weight
    // @Description: Attitude-error weight used in the geometric integral state e_I = integral(e_Omega + c*e_R). This has no effect on axes with zero attitude integral gain.
    // @Range: 0 10
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("ATT_INT_C", 25, AC_GeometricControl, _att_int_c, AC_GEOMETRIC_ATT_INT_C_DEFAULT),

    // @Param: ATT_KI_X
    // @DisplayName: Geometric roll integral gain
    // @Description: Roll geometric integral gain applied to the attitude-channel integral state e_Ix. This defaults to zero so roll remains PD unless explicitly enabled.
    // @Range: 0 5
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("ATT_KI_X", 26, AC_GeometricControl, _att_ki_x, AC_GEOMETRIC_ATT_KI_X_DEFAULT),

    // @Param: ATT_KI_Y
    // @DisplayName: Geometric pitch integral gain
    // @Description: Pitch geometric integral gain applied to the attitude-channel integral state e_Iy. This defaults to zero so pitch remains PD unless explicitly enabled.
    // @Range: 0 5
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("ATT_KI_Y", 27, AC_GeometricControl, _att_ki_y, AC_GEOMETRIC_ATT_KI_Y_DEFAULT),

    // @Param: ATT_IMAX_X
    // @DisplayName: Geometric roll integrator limit
    // @Description: Limit applied to the roll attitude integral state before the ATT_KI_X term is applied. This defaults to zero so roll integral accumulation is disabled unless explicitly enabled.
    // @Range: 0 20
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("ATT_IMAX_X", 28, AC_GeometricControl, _att_imax_x, AC_GEOMETRIC_ATT_IMAX_X_DEFAULT),

    // @Param: ATT_IMAX_Y
    // @DisplayName: Geometric pitch integrator limit
    // @Description: Limit applied to the pitch attitude integral state before the ATT_KI_Y term is applied. This defaults to zero so pitch integral accumulation is disabled unless explicitly enabled.
    // @Range: 0 20
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("ATT_IMAX_Y", 29, AC_GeometricControl, _att_imax_y, AC_GEOMETRIC_ATT_IMAX_Y_DEFAULT),

    // @Param: SHAPE_EN
    // @DisplayName: Geometric setpoint shaper enable
    // @Description: Enables the geometric setpoint shaper for position-derived Guided targets before they enter the Lee SE(3) position channel. The shaper limits target position changes using velocity and acceleration bounds.
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO("SHAPE_EN", 30, AC_GeometricControl, _shape_enabled, AC_GEOMETRIC_SHAPE_ENABLED_DEFAULT),

    // @Param: SHAPE_VXY
    // @DisplayName: Geometric shaper horizontal velocity
    // @Description: Maximum horizontal reference velocity generated by the geometric setpoint shaper.
    // @Range: 0 20
    // @Units: m/s
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("SHAPE_VXY", 31, AC_GeometricControl, _shape_vel_xy_max_ms, AC_GEOMETRIC_SHAPE_VEL_XY_DEFAULT),

    // @Param: SHAPE_AXY
    // @DisplayName: Geometric shaper horizontal acceleration
    // @Description: Maximum horizontal reference acceleration generated by the geometric setpoint shaper.
    // @Range: 0 20
    // @Units: m/s/s
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("SHAPE_AXY", 32, AC_GeometricControl, _shape_accel_xy_max_mss, AC_GEOMETRIC_SHAPE_ACCEL_XY_DEFAULT),

    // @Param: SHAPE_VUP
    // @DisplayName: Geometric shaper climb velocity
    // @Description: Maximum upward reference velocity generated by the geometric setpoint shaper. NED Z is positive down, so upward motion uses negative Z velocity internally.
    // @Range: 0 10
    // @Units: m/s
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("SHAPE_VUP", 33, AC_GeometricControl, _shape_vel_up_max_ms, AC_GEOMETRIC_SHAPE_VEL_UP_DEFAULT),

    // @Param: SHAPE_VDN
    // @DisplayName: Geometric shaper descent velocity
    // @Description: Maximum downward reference velocity generated by the geometric setpoint shaper. NED Z is positive down.
    // @Range: 0 10
    // @Units: m/s
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("SHAPE_VDN", 34, AC_GeometricControl, _shape_vel_down_max_ms, AC_GEOMETRIC_SHAPE_VEL_DOWN_DEFAULT),

    // @Param: SHAPE_AZ
    // @DisplayName: Geometric shaper vertical acceleration
    // @Description: Maximum vertical reference acceleration generated by the geometric setpoint shaper.
    // @Range: 0 10
    // @Units: m/s/s
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("SHAPE_AZ", 35, AC_GeometricControl, _shape_accel_z_max_mss, AC_GEOMETRIC_SHAPE_ACCEL_Z_DEFAULT),

    // @Param: SHAPE_YRAT
    // @DisplayName: Geometric shaper yaw rate
    // @Description: Maximum yaw reference rate generated by the geometric setpoint shaper. This limits explicit yaw shaping when SHAPE_YAW is enabled and always limits geometric yaw-follow derived from shaped trajectory velocity.
    // @Range: 0 6.28
    // @Units: rad/s
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("SHAPE_YRAT", 36, AC_GeometricControl, _shape_yaw_rate_max_rads, AC_GEOMETRIC_SHAPE_YAW_RATE_DEFAULT),

    // @Param: SHAPE_YACC
    // @DisplayName: Geometric shaper yaw acceleration
    // @Description: Maximum yaw reference acceleration generated by the geometric setpoint shaper. This limits explicit yaw shaping when SHAPE_YAW is enabled and always limits geometric yaw-follow derived from shaped trajectory velocity.
    // @Range: 0 20
    // @Units: rad/s/s
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("SHAPE_YACC", 37, AC_GeometricControl, _shape_yaw_accel_max_radss, AC_GEOMETRIC_SHAPE_YAW_ACCEL_DEFAULT),

    // @Param: SHAPE_YAW
    // @DisplayName: Geometric yaw shaper enable
    // @Description: Enables yaw reference shaping for explicit yaw commands inside the geometric setpoint shaper. Geometric yaw-follow for Guided position targets is generated from shaped trajectory velocity and acceleration separately.
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO("SHAPE_YAW", 38, AC_GeometricControl, _shape_yaw_enabled, AC_GEOMETRIC_SHAPE_YAW_ENABLED_DEFAULT),

    // @Param: ATT_J_X
    // @DisplayName: Geometric roll inertia
    // @Description: Diagonal body-X inertia term Jx used by the Lee SO(3) attitude moment formula. Default follows the Gao quadrotor model J = 10^-2 diag(1.1,2.0,2.3) kg*m*m. This is a model parameter for the geometric moment proxy, not an ArduPilot motor normalization scale.
    // @Range: 0.001 1
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("ATT_J_X", 39, AC_GeometricControl, _att_j_x, AC_GEOMETRIC_ATT_J_X_DEFAULT),

    // @Param: ATT_J_Y
    // @DisplayName: Geometric pitch inertia
    // @Description: Diagonal body-Y inertia term Jy used by the Lee SO(3) attitude moment formula. Default follows the Gao quadrotor model J = 10^-2 diag(1.1,2.0,2.3) kg*m*m. This is a model parameter for the geometric moment proxy, not an ArduPilot motor normalization scale.
    // @Range: 0.001 1
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("ATT_J_Y", 40, AC_GeometricControl, _att_j_y, AC_GEOMETRIC_ATT_J_Y_DEFAULT),

    // @Param: ATT_J_Z
    // @DisplayName: Geometric yaw inertia
    // @Description: Diagonal body-Z inertia term Jz used by the Lee SO(3) attitude moment formula. Default follows the Gao quadrotor model J = 10^-2 diag(1.1,2.0,2.3) kg*m*m. This is a model parameter for the geometric moment proxy, not an ArduPilot motor normalization scale.
    // @Range: 0.001 1
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("ATT_J_Z", 41, AC_GeometricControl, _att_j_z, AC_GEOMETRIC_ATT_J_Z_DEFAULT),

    // @Param: POS_INT_C
    // @DisplayName: Geometric position integral error weight
    // @Description: Position-error weight c_x used in the geometric PID position integral state e_XI = integral(e_v + c_x*e_x). This has no effect on axes with zero position integral gain.
    // @Range: 0 10
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("POS_INT_C", 42, AC_GeometricControl, _pos_int_c, AC_GEOMETRIC_POS_INT_C_DEFAULT),

    // @Param: OMG_C_FLT
    // @DisplayName: Geometric Omega_c filter
    // @Description: Optional first-order low-pass cutoff applied to the position-generated commanded angular velocity Omega_c before the SO(3) attitude channel. This attenuates finite-difference spikes from R_c changes. A value of zero disables this filter.
    // @Range: 0 50
    // @Units: Hz
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("OMG_C_FLT", 43, AC_GeometricControl, _omega_c_filt_hz, AC_GEOMETRIC_OMEGA_C_FILTER_DEFAULT),

    // @Param: DOMG_C_FLT
    // @DisplayName: Geometric dot Omega_c filter
    // @Description: Optional first-order low-pass cutoff applied to the position-generated commanded angular acceleration dot(Omega_c) before the SO(3) attitude feed-forward term. This attenuates second-difference spikes from R_c changes. A value of zero disables this filter.
    // @Range: 0 50
    // @Units: Hz
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("DOMG_C_FLT", 44, AC_GeometricControl, _omega_dot_c_filt_hz, AC_GEOMETRIC_OMEGA_DOT_C_FILTER_DEFAULT),

    AP_GROUPEND
};

AC_GeometricControl::AC_GeometricControl()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AC_GeometricControl::reset()
{
    _position_pid.reset();
    _attitude_pd.reset();
    _setpoint_shaper.reset();
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

void AC_GeometricControl::set_hover_throttle_reference(float hover_throttle_norm)
{
    _hover_throttle_reference_norm = constrain_float(hover_throttle_norm, 0.05f, 0.95f);
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
    position_gains.integral_error_p = Vector3f{_pos_int_c.get(), _pos_int_c.get(), _pos_int_c.get()};
    _position_pid.set_gains(position_gains);
    AC_Geometric_Position_Integral_Limits position_integral_limits {};
    position_integral_limits.integral_error_m = Vector3f{_pos_imax_xy.get(), _pos_imax_xy.get(), _pos_imax_z.get()};
    _position_pid.set_integral_limits(position_integral_limits);
    AC_Geometric_Position_Filter_Hz position_filter_hz {};
    position_filter_hz.position_error = _pos_error_filt_hz.get();
    position_filter_hz.velocity_error = _vel_error_filt_hz.get();
    position_filter_hz.omega_c = _omega_c_filt_hz.get();
    position_filter_hz.omega_dot_c = _omega_dot_c_filt_hz.get();
    _position_pid.set_filter_hz(position_filter_hz);

    AC_Geometric_Attitude_Gains attitude_gains {};
    attitude_gains.attitude_p = Vector3f{_att_kr_x.get(), _att_kr_y.get(), _att_kr_z.get()};
    attitude_gains.omega_p = Vector3f{_att_ko_x.get(), _att_ko_y.get(), _att_ko_z.get()};
    attitude_gains.attitude_i = Vector3f{_att_ki_x.get(), _att_ki_y.get(), _att_ki_z.get()};
    attitude_gains.integral_error_p = Vector3f{_att_int_c.get(), _att_int_c.get(), _att_int_c.get()};
    _attitude_pd.set_gains(attitude_gains);
    AC_Geometric_Attitude_Model attitude_model {};
    attitude_model.inertia = Vector3f{_att_j_x.get(), _att_j_y.get(), _att_j_z.get()};
    _attitude_pd.set_model(attitude_model);
    AC_Geometric_Attitude_Integral_Limits attitude_integral_limits {};
    attitude_integral_limits.integral_error = Vector3f{_att_imax_x.get(), _att_imax_y.get(), _att_imax_z.get()};
    _attitude_pd.set_integral_limits(attitude_integral_limits);
    AC_Geometric_Attitude_Filter_Hz attitude_filter_hz {};
    attitude_filter_hz.omega_error = _omega_error_filt_hz.get();
    _attitude_pd.set_filter_hz(attitude_filter_hz);
}

float AC_GeometricControl::hover_throttle_norm() const
{
    const float hover_throttle_override = _hover_throttle_norm.get();
    if (is_positive(hover_throttle_override)) {
        return constrain_float(hover_throttle_override, 0.05f, 0.95f);
    }
    return _hover_throttle_reference_norm;
}

void AC_GeometricControl::update(const AC_Geometric_State& state,
                                 const AC_Geometric_Target& target,
                                 float dt)
{
    if (!_enabled) {
        return;
    }

    update_gains_from_params();

    AC_Geometric_Target position_target = target;
    if (_shape_enabled && target.build_attitude_from_position && target.shape_position_target) {
        AC_Geometric_Setpoint_Shaper_Limits limits {};
        limits.vel_xy_max_ms = _shape_vel_xy_max_ms.get();
        limits.accel_xy_max_mss = _shape_accel_xy_max_mss.get();
        limits.vel_up_max_ms = _shape_vel_up_max_ms.get();
        limits.vel_down_max_ms = _shape_vel_down_max_ms.get();
        limits.accel_z_max_mss = _shape_accel_z_max_mss.get();
        limits.yaw_enabled = _shape_yaw_enabled.get() != 0;
        limits.yaw_rate_max_rads = _shape_yaw_rate_max_rads.get();
        limits.yaw_accel_max_radss = _shape_yaw_accel_max_radss.get();
        _setpoint_shaper.set_limits(limits);
        _setpoint_shaper.update(state, target, dt, position_target);
    } else {
        _setpoint_shaper.reset();
    }

    // The position channel owns the SE(3) coupling: it will eventually build
    // thrust and desired attitude from position/velocity/heading targets.
    _position_pid.update(state, position_target, dt, _output.position);

    // Feed the position-generated desired attitude into the SO(3) attitude channel.
    AC_Geometric_Target attitude_target = position_target;
    attitude_target.attitude_body_to_ned = _output.position.attitude_body_to_ned;
    attitude_target.omega_body_rads = _output.position.omega_body_rads;
    attitude_target.omega_dot_body_radss = _output.position.omega_dot_body_radss;

    _attitude_pd.update(state, attitude_target, dt, _output.attitude);

    const Vector3f moment_norm {
        _mom_norm_x.get(),
        _mom_norm_y.get(),
        _mom_norm_z.get()
    };
    _output_mapper.update(_output.position, _output.attitude, hover_throttle_norm(), moment_norm, _output.mapped);
    _last_update_ms = AP_HAL::millis();
}
