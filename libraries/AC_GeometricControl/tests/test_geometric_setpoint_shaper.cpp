#include <AP_gtest.h>

#include <AC_GeometricControl/AC_Geometric_SetpointShaper.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

Quaternion attitude_from_yaw(float yaw_rad)
{
    Quaternion attitude;
    attitude.from_euler(0.0f, 0.0f, yaw_rad);
    return attitude;
}

AC_Geometric_Setpoint_Shaper_Limits default_limits()
{
    AC_Geometric_Setpoint_Shaper_Limits limits {};
    limits.vel_xy_max_ms = 2.0f;
    limits.accel_xy_max_mss = 1.0f;
    limits.vel_up_max_ms = 1.0f;
    limits.vel_down_max_ms = 2.0f;
    limits.accel_z_max_mss = 0.5f;
    limits.yaw_rate_max_rads = 1.0f;
    limits.yaw_accel_max_radss = 0.5f;
    return limits;
}

AC_Geometric_State state_at_origin()
{
    AC_Geometric_State state {};
    state.attitude_body_to_ned = attitude_from_yaw(0.0f);
    return state;
}

AC_Geometric_Target target_from_position(const Vector3f& position_ned_m)
{
    AC_Geometric_Target target {};
    target.position_ned_m = position_ned_m;
    target.build_attitude_from_position = true;
    return target;
}

}

TEST(AC_Geometric_SetpointShaper, FarHorizontalTargetIsAccelerationLimited)
{
    AC_Geometric_SetpointShaper shaper;
    shaper.set_limits(default_limits());

    const AC_Geometric_State state = state_at_origin();
    AC_Geometric_Target raw_target = target_from_position(Vector3f{10.0f, 0.0f, 0.0f});
    AC_Geometric_Target shaped_target {};

    shaper.update(state, raw_target, 1.0f, shaped_target);

    EXPECT_NEAR(shaped_target.position_ned_m.x, 0.5f, 1.0e-6f);
    EXPECT_NEAR(shaped_target.velocity_ned_ms.x, 1.0f, 1.0e-6f);
    EXPECT_NEAR(shaped_target.accel_ned_mss.x, 1.0f, 1.0e-6f);
    EXPECT_NEAR(shaped_target.position_ned_m.y, 0.0f, 1.0e-6f);
    EXPECT_NEAR(shaped_target.velocity_ned_ms.y, 0.0f, 1.0e-6f);
    EXPECT_NEAR(shaped_target.accel_ned_mss.y, 0.0f, 1.0e-6f);
}

TEST(AC_Geometric_SetpointShaper, HorizontalVelocityIsLimited)
{
    AC_Geometric_SetpointShaper shaper;
    AC_Geometric_Setpoint_Shaper_Limits limits = default_limits();
    limits.accel_xy_max_mss = 10.0f;
    shaper.set_limits(limits);

    const AC_Geometric_State state = state_at_origin();
    const AC_Geometric_Target raw_target = target_from_position(Vector3f{100.0f, 0.0f, 0.0f});
    AC_Geometric_Target shaped_target {};

    for (uint8_t i = 0; i < 5; i++) {
        shaper.update(state, raw_target, 1.0f, shaped_target);
        EXPECT_LE(fabsf(shaped_target.velocity_ned_ms.x), limits.vel_xy_max_ms + 1.0e-6f);
    }

    EXPECT_NEAR(shaped_target.velocity_ned_ms.x, limits.vel_xy_max_ms, 1.0e-6f);
}

TEST(AC_Geometric_SetpointShaper, VerticalVelocityUsesNedUpAndDownLimits)
{
    AC_Geometric_Setpoint_Shaper_Limits limits = default_limits();

    {
        AC_Geometric_SetpointShaper shaper;
        shaper.set_limits(limits);

        const AC_Geometric_State state = state_at_origin();
        const AC_Geometric_Target raw_target = target_from_position(Vector3f{0.0f, 0.0f, -10.0f});
        AC_Geometric_Target shaped_target {};

        shaper.update(state, raw_target, 1.0f, shaped_target);

        EXPECT_NEAR(shaped_target.position_ned_m.z, -0.25f, 1.0e-6f);
        EXPECT_NEAR(shaped_target.velocity_ned_ms.z, -0.5f, 1.0e-6f);
        EXPECT_NEAR(shaped_target.accel_ned_mss.z, -0.5f, 1.0e-6f);
    }

    {
        AC_Geometric_SetpointShaper shaper;
        shaper.set_limits(limits);

        const AC_Geometric_State state = state_at_origin();
        const AC_Geometric_Target raw_target = target_from_position(Vector3f{0.0f, 0.0f, 10.0f});
        AC_Geometric_Target shaped_target {};

        shaper.update(state, raw_target, 1.0f, shaped_target);

        EXPECT_NEAR(shaped_target.position_ned_m.z, 0.25f, 1.0e-6f);
        EXPECT_NEAR(shaped_target.velocity_ned_ms.z, 0.5f, 1.0e-6f);
        EXPECT_NEAR(shaped_target.accel_ned_mss.z, 0.5f, 1.0e-6f);
    }
}

TEST(AC_Geometric_SetpointShaper, YawReferenceIsRateAndAccelerationLimited)
{
    AC_Geometric_SetpointShaper shaper;
    AC_Geometric_Setpoint_Shaper_Limits limits = default_limits();
    limits.yaw_enabled = true;
    shaper.set_limits(limits);

    const AC_Geometric_State state = state_at_origin();
    AC_Geometric_Target raw_target = target_from_position(Vector3f{0.0f, 0.0f, 0.0f});
    raw_target.yaw_rad = M_PI_2;
    AC_Geometric_Target shaped_target {};

    shaper.update(state, raw_target, 1.0f, shaped_target);

    EXPECT_NEAR(shaped_target.yaw_rad, 0.25f, 1.0e-6f);
    EXPECT_NEAR(shaped_target.yaw_rate_rads, 0.5f, 1.0e-6f);
    EXPECT_NEAR(shaped_target.omega_body_rads.z, 0.5f, 1.0e-6f);
}

TEST(AC_Geometric_SetpointShaper, YawReferencePassesThroughWhenYawShaperDisabled)
{
    AC_Geometric_SetpointShaper shaper;
    AC_Geometric_Setpoint_Shaper_Limits limits = default_limits();
    limits.yaw_enabled = false;
    shaper.set_limits(limits);

    const AC_Geometric_State state = state_at_origin();
    AC_Geometric_Target raw_target = target_from_position(Vector3f{0.0f, 0.0f, 0.0f});
    raw_target.yaw_rad = 1.0f;
    raw_target.yaw_rate_rads = 0.3f;
    raw_target.omega_body_rads.z = 0.3f;
    AC_Geometric_Target shaped_target {};

    shaper.update(state, raw_target, 1.0f, shaped_target);

    EXPECT_NEAR(shaped_target.yaw_rad, raw_target.yaw_rad, 1.0e-6f);
    EXPECT_NEAR(shaped_target.yaw_rate_rads, raw_target.yaw_rate_rads, 1.0e-6f);
    EXPECT_NEAR(shaped_target.omega_body_rads.z, raw_target.omega_body_rads.z, 1.0e-6f);
}

TEST(AC_Geometric_SetpointShaper, TrajectoryYawUsesShapedVelocityDirection)
{
    AC_Geometric_SetpointShaper shaper;
    AC_Geometric_Setpoint_Shaper_Limits limits = default_limits();
    limits.yaw_enabled = false;
    shaper.set_limits(limits);

    const AC_Geometric_State state = state_at_origin();
    AC_Geometric_Target raw_target = target_from_position(Vector3f{0.0f, 10.0f, 0.0f});
    raw_target.yaw_rad = -1.0f;
    raw_target.yaw_from_trajectory = true;
    AC_Geometric_Target shaped_target {};

    shaper.update(state, raw_target, 1.0f, shaped_target);

    EXPECT_NEAR(shaped_target.velocity_ned_ms.y, 1.0f, 1.0e-6f);
    EXPECT_NEAR(shaped_target.yaw_rad, 0.25f, 1.0e-6f);
    EXPECT_NEAR(shaped_target.yaw_rate_rads, 0.5f, 1.0e-6f);
    EXPECT_NEAR(shaped_target.omega_body_rads.z, 0.5f, 1.0e-6f);
}

TEST(AC_Geometric_SetpointShaper, TrajectoryYawHoldsLastReferenceAtLowSpeed)
{
    AC_Geometric_SetpointShaper shaper;
    shaper.set_limits(default_limits());

    AC_Geometric_State state = state_at_origin();
    state.attitude_body_to_ned = attitude_from_yaw(0.7f);
    state.omega_body_rads.z = 0.4f;
    AC_Geometric_Target raw_target = target_from_position(Vector3f{0.0f, 0.0f, 0.0f});
    raw_target.yaw_rad = -1.0f;
    raw_target.yaw_from_trajectory = true;
    AC_Geometric_Target shaped_target {};

    shaper.update(state, raw_target, 1.0f, shaped_target);

    EXPECT_NEAR(shaped_target.velocity_ned_ms.xy().length(), 0.0f, 1.0e-6f);
    EXPECT_NEAR(wrap_PI(shaped_target.yaw_rad - 0.7f), 0.0f, 1.0e-6f);
    EXPECT_NEAR(shaped_target.yaw_rate_rads, 0.0f, 1.0e-6f);
    EXPECT_NEAR(shaped_target.omega_body_rads.z, 0.0f, 1.0e-6f);
}

TEST(AC_Geometric_SetpointShaper, ResetInitializesFromCurrentState)
{
    AC_Geometric_SetpointShaper shaper;
    shaper.set_limits(default_limits());

    const AC_Geometric_State state = state_at_origin();
    AC_Geometric_Target raw_target = target_from_position(Vector3f{10.0f, 0.0f, 0.0f});
    AC_Geometric_Target shaped_target {};
    shaper.update(state, raw_target, 1.0f, shaped_target);

    AC_Geometric_State new_state {};
    new_state.position_ned_m = Vector3f{5.0f, -2.0f, 1.0f};
    new_state.attitude_body_to_ned = attitude_from_yaw(0.5f);

    AC_Geometric_Target new_target = target_from_position(new_state.position_ned_m);
    new_target.yaw_rad = 0.5f;

    shaper.reset();
    shaper.update(new_state, new_target, 1.0f, shaped_target);

    EXPECT_NEAR(shaped_target.position_ned_m.x, new_state.position_ned_m.x, 1.0e-6f);
    EXPECT_NEAR(shaped_target.position_ned_m.y, new_state.position_ned_m.y, 1.0e-6f);
    EXPECT_NEAR(shaped_target.position_ned_m.z, new_state.position_ned_m.z, 1.0e-6f);
    EXPECT_NEAR(shaped_target.velocity_ned_ms.length(), 0.0f, 1.0e-6f);
    EXPECT_NEAR(wrap_PI(shaped_target.yaw_rad - new_target.yaw_rad), 0.0f, 1.0e-6f);
}

AP_GTEST_MAIN()
