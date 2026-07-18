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

    for (uint16_t i = 0; i < 500; i++) {
        shaper.update(state, raw_target, 0.01f, shaped_target);
        EXPECT_LE(shaped_target.velocity_ned_ms.xy().length(), limits.vel_xy_max_ms + 0.05f);
    }

    EXPECT_NEAR(shaped_target.velocity_ned_ms.x, limits.vel_xy_max_ms, 0.05f);
}

TEST(AC_Geometric_SetpointShaper, HorizontalAccelerationIsJerkLimited)
{
    AC_Geometric_SetpointShaper shaper;
    const AC_Geometric_Setpoint_Shaper_Limits limits = default_limits();
    shaper.set_limits(limits);

    const AC_Geometric_State state = state_at_origin();
    const AC_Geometric_Target raw_target = target_from_position(Vector3f{10.0f, 0.0f, 0.0f});
    AC_Geometric_Target shaped_target {};

    shaper.update(state, raw_target, 0.1f, shaped_target);

    EXPECT_NEAR(shaped_target.accel_ned_mss.x, 0.2f, 1.0e-6f);
    EXPECT_NEAR(shaped_target.velocity_ned_ms.x, 0.02f, 1.0e-6f);
    EXPECT_NEAR(shaped_target.position_ned_m.x, 0.001f, 1.0e-6f);
}

TEST(AC_Geometric_SetpointShaper, HorizontalReferenceSettlesAtTarget)
{
    AC_Geometric_SetpointShaper shaper;
    shaper.set_limits(default_limits());

    const AC_Geometric_State state = state_at_origin();
    const AC_Geometric_Target raw_target = target_from_position(Vector3f{1.0f, 0.0f, 0.0f});
    AC_Geometric_Target shaped_target {};

    for (uint16_t i = 0; i < 500; i++) {
        shaper.update(state, raw_target, 0.01f, shaped_target);
    }

    EXPECT_NEAR(shaped_target.position_ned_m.x, raw_target.position_ned_m.x, 1.0e-6f);
    EXPECT_NEAR(shaped_target.position_ned_m.y, raw_target.position_ned_m.y, 1.0e-6f);
    EXPECT_NEAR(shaped_target.velocity_ned_ms.xy().length(), 0.0f, 1.0e-6f);
    EXPECT_NEAR(shaped_target.accel_ned_mss.xy().length(), 0.0f, 1.0e-6f);

    for (uint8_t i = 0; i < 50; i++) {
        shaper.update(state, raw_target, 0.01f, shaped_target);
        EXPECT_NEAR(shaped_target.position_ned_m.x, raw_target.position_ned_m.x, 1.0e-6f);
        EXPECT_NEAR(shaped_target.velocity_ned_ms.xy().length(), 0.0f, 1.0e-6f);
        EXPECT_NEAR(shaped_target.accel_ned_mss.xy().length(), 0.0f, 1.0e-6f);
    }
}

TEST(AC_Geometric_SetpointShaper, MovingHorizontalPvaIsNotZeroedBySettleShortcut)
{
    AC_Geometric_SetpointShaper shaper;
    AC_Geometric_Setpoint_Shaper_Limits limits = default_limits();
    limits.vel_xy_max_ms = 5.0f;
    limits.accel_xy_max_mss = 4.0f;
    shaper.set_limits(limits);

    const AC_Geometric_State state = state_at_origin();
    AC_Geometric_Target raw_target = target_from_position(Vector3f{});
    raw_target.velocity_ned_ms.x = 3.5f;
    AC_Geometric_Target shaped_target {};

    constexpr float dt = 1.0f / 85.0f;
    for (uint8_t i = 0; i < 85; i++) {
        // Each raw-position increment is below the XY settle distance.  A
        // position-only settle test would therefore snap every frame and
        // incorrectly erase the moving target's velocity feedforward.
        raw_target.position_ned_m.x += raw_target.velocity_ned_ms.x * dt;
        shaper.update(state, raw_target, dt, shaped_target);
    }

    EXPECT_GT(shaped_target.velocity_ned_ms.x, 1.0f);
    EXPECT_GT(shaped_target.position_ned_m.x, 0.1f);
}

TEST(AC_Geometric_SetpointShaper, HorizontalAccelerationFeedforwardPreventsFalseSettle)
{
    AC_Geometric_SetpointShaper shaper;
    shaper.set_limits(default_limits());

    const AC_Geometric_State state = state_at_origin();
    AC_Geometric_Target raw_target = target_from_position(Vector3f{});
    raw_target.accel_ned_mss.x = 0.5f;
    AC_Geometric_Target shaped_target {};

    shaper.update(state, raw_target, 0.01f, shaped_target);

    EXPECT_GT(shaped_target.accel_ned_mss.x, 0.0f);
    EXPECT_GT(shaped_target.velocity_ned_ms.x, 0.0f);
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

TEST(AC_Geometric_SetpointShaper, VerticalReferenceSettlesAtTarget)
{
    AC_Geometric_SetpointShaper shaper;
    shaper.set_limits(default_limits());

    const AC_Geometric_State state = state_at_origin();
    const AC_Geometric_Target raw_target = target_from_position(Vector3f{0.0f, 0.0f, -1.0f});
    AC_Geometric_Target shaped_target {};

    for (uint16_t i = 0; i < 500; i++) {
        shaper.update(state, raw_target, 0.01f, shaped_target);
    }

    EXPECT_NEAR(shaped_target.position_ned_m.z, raw_target.position_ned_m.z, 1.0e-6f);
    EXPECT_NEAR(shaped_target.velocity_ned_ms.z, 0.0f, 1.0e-6f);
    EXPECT_NEAR(shaped_target.accel_ned_mss.z, 0.0f, 1.0e-6f);

    for (uint8_t i = 0; i < 50; i++) {
        shaper.update(state, raw_target, 0.01f, shaped_target);
        EXPECT_NEAR(shaped_target.position_ned_m.z, raw_target.position_ned_m.z, 1.0e-6f);
        EXPECT_NEAR(shaped_target.velocity_ned_ms.z, 0.0f, 1.0e-6f);
        EXPECT_NEAR(shaped_target.accel_ned_mss.z, 0.0f, 1.0e-6f);
    }
}

TEST(AC_Geometric_SetpointShaper, MovingVerticalPvaIsNotZeroedBySettleShortcut)
{
    AC_Geometric_SetpointShaper shaper;
    AC_Geometric_Setpoint_Shaper_Limits limits = default_limits();
    limits.vel_up_max_ms = 2.0f;
    limits.accel_z_max_mss = 2.0f;
    shaper.set_limits(limits);

    const AC_Geometric_State state = state_at_origin();
    AC_Geometric_Target raw_target = target_from_position(Vector3f{});
    raw_target.velocity_ned_ms.z = -1.0f;
    AC_Geometric_Target shaped_target {};

    constexpr float dt = 1.0f / 85.0f;
    for (uint8_t i = 0; i < 85; i++) {
        // The per-frame climb increment is below the Z settle distance.
        raw_target.position_ned_m.z += raw_target.velocity_ned_ms.z * dt;
        shaper.update(state, raw_target, dt, shaped_target);
    }

    EXPECT_LT(shaped_target.velocity_ned_ms.z, -0.2f);
    EXPECT_LT(shaped_target.position_ned_m.z, -0.05f);
}

TEST(AC_Geometric_SetpointShaper, VerticalAccelerationFeedforwardPreventsFalseSettle)
{
    AC_Geometric_SetpointShaper shaper;
    shaper.set_limits(default_limits());

    const AC_Geometric_State state = state_at_origin();
    AC_Geometric_Target raw_target = target_from_position(Vector3f{});
    raw_target.accel_ned_mss.z = -0.25f;
    AC_Geometric_Target shaped_target {};

    shaper.update(state, raw_target, 0.01f, shaped_target);

    EXPECT_LT(shaped_target.accel_ned_mss.z, 0.0f);
    EXPECT_LT(shaped_target.velocity_ned_ms.z, 0.0f);
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
