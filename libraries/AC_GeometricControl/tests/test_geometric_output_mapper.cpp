#include <AP_gtest.h>

#include <AC_GeometricControl/AC_Geometric_OutputMapper.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

Quaternion attitude_from_euler(float roll_rad, float pitch_rad, float yaw_rad)
{
    Quaternion attitude;
    attitude.from_euler(roll_rad, pitch_rad, yaw_rad);
    return attitude;
}

AC_Geometric_Mapped_Output run_mapper(const AC_Geometric_Position_Output& position,
                                      const AC_Geometric_Attitude_Output& attitude,
                                      float hover_throttle_norm)
{
    AC_Geometric_OutputMapper mapper;
    AC_Geometric_Mapped_Output output {};

    mapper.update(position, attitude, hover_throttle_norm, output);

    return output;
}

}

TEST(AC_Geometric_OutputMapper, HoverThrustMapsToHoverThrottle)
{
    AC_Geometric_Position_Output position {};
    position.thrust = GRAVITY_MSS;

    AC_Geometric_Attitude_Output attitude {};

    const AC_Geometric_Mapped_Output output = run_mapper(position, attitude, 0.45f);

    EXPECT_NEAR(output.throttle_norm_raw, 0.45f, 1.0e-6f);
    EXPECT_NEAR(output.throttle_norm, 0.45f, 1.0e-6f);
    EXPECT_FALSE(output.throttle_limited);
}

TEST(AC_Geometric_OutputMapper, ThrottleShadowOutputIsLimited)
{
    AC_Geometric_Position_Output position {};
    AC_Geometric_Attitude_Output attitude {};

    position.thrust = 3.0f * GRAVITY_MSS;
    AC_Geometric_Mapped_Output output = run_mapper(position, attitude, 0.5f);

    EXPECT_NEAR(output.throttle_norm_raw, 1.5f, 1.0e-6f);
    EXPECT_NEAR(output.throttle_norm, 1.0f, 1.0e-6f);
    EXPECT_TRUE(output.throttle_limited);

    position.thrust = -GRAVITY_MSS;
    output = run_mapper(position, attitude, 0.5f);

    EXPECT_NEAR(output.throttle_norm_raw, -0.5f, 1.0e-6f);
    EXPECT_NEAR(output.throttle_norm, 0.0f, 1.0e-6f);
    EXPECT_TRUE(output.throttle_limited);
}

TEST(AC_Geometric_OutputMapper, PassesThroughShadowAttitudeAndRate)
{
    AC_Geometric_Position_Output position {};
    position.attitude_body_to_ned = attitude_from_euler(0.1f, -0.2f, 0.3f);

    AC_Geometric_Attitude_Output attitude {};
    attitude.rate_target_body_rads = Vector3f{0.4f, -0.5f, 0.6f};

    const AC_Geometric_Mapped_Output output = run_mapper(position, attitude, 0.5f);

    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    output.attitude_body_to_ned.to_euler(roll_rad, pitch_rad, yaw_rad);

    EXPECT_NEAR(roll_rad, 0.1f, 1.0e-6f);
    EXPECT_NEAR(pitch_rad, -0.2f, 1.0e-6f);
    EXPECT_NEAR(yaw_rad, 0.3f, 1.0e-6f);
    EXPECT_NEAR(output.rate_target_body_rads.x, 0.4f, 1.0e-6f);
    EXPECT_NEAR(output.rate_target_body_rads.y, -0.5f, 1.0e-6f);
    EXPECT_NEAR(output.rate_target_body_rads.z, 0.6f, 1.0e-6f);
}

AP_GTEST_MAIN()
