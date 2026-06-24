#include <AP_gtest.h>

#include <AC_GeometricControl/AC_Geometric_Position_PID.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

Quaternion attitude_from_euler(float roll_rad, float pitch_rad, float yaw_rad)
{
    Quaternion attitude;
    attitude.from_euler(roll_rad, pitch_rad, yaw_rad);
    return attitude;
}

AC_Geometric_Position_Output run_position_pid(const AC_Geometric_Position_Gains& gains,
                                              const AC_Geometric_State& state,
                                              const AC_Geometric_Target& target)
{
    AC_Geometric_Position_PID controller;
    AC_Geometric_Position_Output output {};

    controller.set_gains(gains);
    controller.update(state, target, 0.01f, output);

    return output;
}

Vector3f normalized(Vector3f value)
{
    value.normalize();
    return value;
}

}

TEST(AC_Geometric_Position_PID, HoverBuildsLevelRc)
{
    AC_Geometric_State state {};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);

    AC_Geometric_Target target {};
    target.build_attitude_from_position = true;

    AC_Geometric_Position_Gains gains {};
    gains.p = Vector3f{1.0f, 1.0f, 1.0f};

    const AC_Geometric_Position_Output output = run_position_pid(gains, state, target);

    EXPECT_NEAR(output.specific_force_ned_mss.x, 0.0f, 1.0e-6f);
    EXPECT_NEAR(output.specific_force_ned_mss.y, 0.0f, 1.0e-6f);
    EXPECT_NEAR(output.specific_force_ned_mss.z, -GRAVITY_MSS, 1.0e-5f);
    EXPECT_NEAR(output.thrust, GRAVITY_MSS, 1.0e-5f);

    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    output.attitude_body_to_ned.to_euler(roll_rad, pitch_rad, yaw_rad);

    EXPECT_NEAR(roll_rad, 0.0f, 1.0e-6f);
    EXPECT_NEAR(pitch_rad, 0.0f, 1.0e-6f);
    EXPECT_NEAR(yaw_rad, 0.0f, 1.0e-6f);
}

TEST(AC_Geometric_Position_PID, NorthAndEastTargetsTiltRcWithNedSigns)
{
    AC_Geometric_State state {};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);

    AC_Geometric_Position_Gains gains {};
    gains.p = Vector3f{1.0f, 1.0f, 1.0f};

    {
        AC_Geometric_Target target {};
        target.position_ned_m.x = 1.0f;
        target.build_attitude_from_position = true;

        const AC_Geometric_Position_Output output = run_position_pid(gains, state, target);

        float roll_rad;
        float pitch_rad;
        float yaw_rad;
        output.attitude_body_to_ned.to_euler(roll_rad, pitch_rad, yaw_rad);

        EXPECT_GT(output.specific_force_ned_mss.x, 0.0f);
        EXPECT_NEAR(output.specific_force_ned_mss.y, 0.0f, 1.0e-6f);
        EXPECT_NEAR(roll_rad, 0.0f, 1.0e-6f);
        EXPECT_LT(pitch_rad, 0.0f);
    }

    {
        AC_Geometric_Target target {};
        target.position_ned_m.y = 1.0f;
        target.build_attitude_from_position = true;

        const AC_Geometric_Position_Output output = run_position_pid(gains, state, target);

        float roll_rad;
        float pitch_rad;
        float yaw_rad;
        output.attitude_body_to_ned.to_euler(roll_rad, pitch_rad, yaw_rad);

        EXPECT_NEAR(output.specific_force_ned_mss.x, 0.0f, 1.0e-6f);
        EXPECT_GT(output.specific_force_ned_mss.y, 0.0f);
        EXPECT_GT(roll_rad, 0.0f);
        EXPECT_NEAR(pitch_rad, 0.0f, 1.0e-6f);
    }
}

TEST(AC_Geometric_Position_PID, YawDoesNotChangeRcThrustDirection)
{
    AC_Geometric_State state {};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);

    AC_Geometric_Target target {};
    target.position_ned_m = Vector3f{1.0f, 0.5f, 0.0f};
    target.yaw_rad = 0.7f;
    target.build_attitude_from_position = true;

    AC_Geometric_Position_Gains gains {};
    gains.p = Vector3f{1.0f, 1.0f, 1.0f};

    const AC_Geometric_Position_Output output = run_position_pid(gains, state, target);

    Matrix3f attitude;
    output.attitude_body_to_ned.rotation_matrix(attitude);

    const Vector3f thrust_direction_ned = normalized(output.thrust_vector_ned);
    const Vector3f body_z_ned = attitude.colz();

    EXPECT_NEAR(thrust_direction_ned.x, -body_z_ned.x, 1.0e-5f);
    EXPECT_NEAR(thrust_direction_ned.y, -body_z_ned.y, 1.0e-5f);
    EXPECT_NEAR(thrust_direction_ned.z, -body_z_ned.z, 1.0e-5f);

    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    output.attitude_body_to_ned.to_euler(roll_rad, pitch_rad, yaw_rad);

    // Euler yaw can shift slightly once the thrust vector tilts; the critical
    // invariant above is that yaw composition does not change the thrust axis.
    EXPECT_NEAR(yaw_rad, target.yaw_rad, 2.0e-3f);
}

AP_GTEST_MAIN()
