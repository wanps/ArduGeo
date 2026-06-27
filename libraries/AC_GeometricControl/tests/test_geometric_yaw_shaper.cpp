#include <AP_gtest.h>

#include <AC_GeometricControl/AC_Geometric_YawShaper.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

Quaternion attitude_from_yaw(float yaw_rad)
{
    Quaternion attitude;
    attitude.from_euler(0.0f, 0.0f, yaw_rad);
    return attitude;
}

AC_Geometric_Yaw_Shaper_Limits default_limits()
{
    AC_Geometric_Yaw_Shaper_Limits limits {};
    limits.explicit_yaw_enabled = true;
    limits.yaw_rate_max_rads = 1.0f;
    limits.yaw_accel_max_radss = 0.5f;
    limits.trajectory_min_speed_ms = 0.1f;
    return limits;
}

AC_Geometric_State state_with_yaw(float yaw_rad)
{
    AC_Geometric_State state {};
    state.attitude_body_to_ned = attitude_from_yaw(yaw_rad);
    return state;
}

AC_Geometric_Target empty_position_target()
{
    AC_Geometric_Target target {};
    target.build_attitude_from_position = true;
    return target;
}

}

TEST(AC_Geometric_YawShaper, ExplicitYawIsRateAndAccelerationLimited)
{
    AC_Geometric_YawShaper shaper;
    shaper.set_limits(default_limits());

    const AC_Geometric_State state = state_with_yaw(0.0f);
    AC_Geometric_Target raw_target = empty_position_target();
    raw_target.yaw_rad = M_PI_2;
    AC_Geometric_Target shaped_target = raw_target;

    EXPECT_TRUE(shaper.update(state, raw_target, Vector3f{}, Vector3f{}, 1.0f, shaped_target));

    EXPECT_NEAR(shaped_target.yaw_rad, 0.25f, 1.0e-6f);
    EXPECT_NEAR(shaped_target.yaw_rate_rads, 0.5f, 1.0e-6f);
    EXPECT_NEAR(shaped_target.omega_body_rads.z, 0.5f, 1.0e-6f);
}

TEST(AC_Geometric_YawShaper, ExplicitYawUsesShortestReverseDirection)
{
    AC_Geometric_YawShaper shaper;
    shaper.set_limits(default_limits());

    const AC_Geometric_State state = state_with_yaw(0.0f);
    AC_Geometric_Target raw_target = empty_position_target();
    raw_target.yaw_rad = -M_PI_2;
    AC_Geometric_Target shaped_target = raw_target;

    EXPECT_TRUE(shaper.update(state, raw_target, Vector3f{}, Vector3f{}, 1.0f, shaped_target));

    EXPECT_NEAR(shaped_target.yaw_rad, -0.25f, 1.0e-6f);
    EXPECT_NEAR(shaped_target.yaw_rate_rads, -0.5f, 1.0e-6f);
    EXPECT_NEAR(shaped_target.omega_body_rads.z, -0.5f, 1.0e-6f);
}

TEST(AC_Geometric_YawShaper, ExplicitYawPassesThroughWhenDisabled)
{
    AC_Geometric_YawShaper shaper;
    AC_Geometric_Yaw_Shaper_Limits limits = default_limits();
    limits.explicit_yaw_enabled = false;
    shaper.set_limits(limits);

    const AC_Geometric_State state = state_with_yaw(0.0f);
    AC_Geometric_Target raw_target = empty_position_target();
    raw_target.yaw_rad = 1.0f;
    raw_target.yaw_rate_rads = 0.3f;
    AC_Geometric_Target shaped_target = raw_target;

    EXPECT_FALSE(shaper.update(state, raw_target, Vector3f{}, Vector3f{}, 1.0f, shaped_target));

    EXPECT_NEAR(shaped_target.yaw_rad, raw_target.yaw_rad, 1.0e-6f);
    EXPECT_NEAR(shaped_target.yaw_rate_rads, raw_target.yaw_rate_rads, 1.0e-6f);
    EXPECT_NEAR(shaped_target.omega_body_rads.z, raw_target.yaw_rate_rads, 1.0e-6f);
}

TEST(AC_Geometric_YawShaper, TrajectoryYawUsesShapedVelocityDirection)
{
    AC_Geometric_YawShaper shaper;
    AC_Geometric_Yaw_Shaper_Limits limits = default_limits();
    limits.explicit_yaw_enabled = false;
    shaper.set_limits(limits);

    const AC_Geometric_State state = state_with_yaw(0.0f);
    AC_Geometric_Target raw_target = empty_position_target();
    raw_target.yaw_from_trajectory = true;
    AC_Geometric_Target shaped_target = raw_target;

    EXPECT_TRUE(shaper.update(state,
                              raw_target,
                              Vector3f{0.0f, 1.0f, 0.0f},
                              Vector3f{},
                              1.0f,
                              shaped_target));

    EXPECT_NEAR(shaped_target.yaw_rad, 0.25f, 1.0e-6f);
    EXPECT_NEAR(shaped_target.yaw_rate_rads, 0.5f, 1.0e-6f);
    EXPECT_NEAR(shaped_target.omega_body_rads.z, 0.5f, 1.0e-6f);
}

TEST(AC_Geometric_YawShaper, TrajectoryYawHoldsLastReferenceAtLowSpeed)
{
    AC_Geometric_YawShaper shaper;
    shaper.set_limits(default_limits());

    AC_Geometric_State state = state_with_yaw(0.7f);
    state.omega_body_rads.z = 0.4f;
    AC_Geometric_Target raw_target = empty_position_target();
    raw_target.yaw_rad = -1.0f;
    raw_target.yaw_from_trajectory = true;
    AC_Geometric_Target shaped_target = raw_target;

    EXPECT_TRUE(shaper.update(state, raw_target, Vector3f{}, Vector3f{}, 1.0f, shaped_target));

    EXPECT_NEAR(wrap_PI(shaped_target.yaw_rad - 0.7f), 0.0f, 1.0e-6f);
    EXPECT_NEAR(shaped_target.yaw_rate_rads, 0.0f, 1.0e-6f);
    EXPECT_NEAR(shaped_target.omega_body_rads.z, 0.0f, 1.0e-6f);
}

AP_GTEST_MAIN()
