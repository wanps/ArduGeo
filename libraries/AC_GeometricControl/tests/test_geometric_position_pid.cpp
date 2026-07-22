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

TEST(AC_Geometric_Position_PID, TiltedRcUsesLeeHeadingProjection)
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

    const Vector3f heading_ned{cosf(target.yaw_rad), sinf(target.yaw_rad), 0.0f};
    const Vector3f expected_body_x_ned = normalized(heading_ned - body_z_ned * (heading_ned * body_z_ned));
    const Vector3f body_x_ned = attitude.colx();

    EXPECT_NEAR(body_x_ned.x, expected_body_x_ned.x, 1.0e-5f);
    EXPECT_NEAR(body_x_ned.y, expected_body_x_ned.y, 1.0e-5f);
    EXPECT_NEAR(body_x_ned.z, expected_body_x_ned.z, 1.0e-5f);
}

TEST(AC_Geometric_Position_PID, LevelRcPreservesArduPilotYawInterface)
{
    AC_Geometric_State state {};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);

    AC_Geometric_Target target {};
    target.yaw_rad = -0.8f;
    target.build_attitude_from_position = true;

    AC_Geometric_Position_Gains gains {};
    const AC_Geometric_Position_Output output = run_position_pid(gains, state, target);

    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    output.attitude_body_to_ned.to_euler(roll_rad, pitch_rad, yaw_rad);

    EXPECT_NEAR(roll_rad, 0.0f, 1.0e-6f);
    EXPECT_NEAR(pitch_rad, 0.0f, 1.0e-6f);
    EXPECT_NEAR(wrap_PI(yaw_rad - target.yaw_rad), 0.0f, 1.0e-6f);
}

TEST(AC_Geometric_Position_PID, PositionGeneratedRcProducesBodyRateFeedForward)
{
    AC_Geometric_Position_PID controller;
    AC_Geometric_Position_Gains gains {};
    gains.p = Vector3f{1.0f, 1.0f, 1.0f};
    controller.set_gains(gains);

    AC_Geometric_State state {};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);

    AC_Geometric_Target target {};
    target.build_attitude_from_position = true;

    AC_Geometric_Position_Output output {};
    controller.update(state, target, 0.1f, output);

    target.yaw_rad = 0.1f;
    controller.update(state, target, 0.1f, output);

    EXPECT_NEAR(output.omega_body_rads.x, 0.0f, 1.0e-6f);
    EXPECT_NEAR(output.omega_body_rads.y, 0.0f, 1.0e-6f);
    EXPECT_NEAR(output.omega_body_rads.z, 1.0f, 2.0e-3f);
}

TEST(AC_Geometric_Position_PID, OptionalOmegaTargetFiltersSmoothRcDifferentiation)
{
    AC_Geometric_Position_PID controller;

    AC_Geometric_Position_Gains gains {};
    gains.p = Vector3f{1.0f, 1.0f, 1.0f};
    controller.set_gains(gains);

    AC_Geometric_Position_Filter_Hz filter_hz {};
    filter_hz.omega_c = 1.0f;
    filter_hz.omega_dot_c = 1.0f;
    controller.set_filter_hz(filter_hz);

    AC_Geometric_State state {};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);

    AC_Geometric_Target target {};
    target.build_attitude_from_position = true;

    AC_Geometric_Position_Output output {};
    controller.update(state, target, 0.1f, output);

    target.yaw_rad = 0.1f;
    controller.update(state, target, 0.1f, output);

    EXPECT_GT(output.omega_body_rads.z, 0.0f);
    EXPECT_LT(output.omega_body_rads.z, 1.0f);
    EXPECT_GT(output.omega_dot_body_radss.z, 0.0f);
    EXPECT_LT(output.omega_dot_body_radss.z, 10.0f);
}

TEST(AC_Geometric_Position_PID, OptionalErrorFiltersSmoothStateErrorSteps)
{
    AC_Geometric_Position_PID controller;
    AC_Geometric_Position_Gains gains {};
    gains.p = Vector3f{1.0f, 1.0f, 1.0f};
    gains.d = Vector3f{1.0f, 1.0f, 1.0f};
    controller.set_gains(gains);

    AC_Geometric_Position_Filter_Hz filter_hz {};
    filter_hz.position_error = 5.0f;
    filter_hz.velocity_error = 5.0f;
    controller.set_filter_hz(filter_hz);

    AC_Geometric_State state {};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);

    AC_Geometric_Target target {};
    target.build_attitude_from_position = true;

    AC_Geometric_Position_Output output {};
    controller.update(state, target, 0.01f, output);

    state.position_ned_m = Vector3f{1.0f, -1.0f, 0.5f};
    state.velocity_ned_ms = Vector3f{2.0f, -2.0f, 1.0f};
    controller.update(state, target, 0.01f, output);

    EXPECT_GT(output.position_error_m.x, 0.0f);
    EXPECT_LT(output.position_error_m.x, state.position_ned_m.x);
    EXPECT_LT(output.position_error_m.y, 0.0f);
    EXPECT_GT(output.position_error_m.y, state.position_ned_m.y);
    EXPECT_GT(output.velocity_error_ms.x, 0.0f);
    EXPECT_LT(output.velocity_error_ms.x, state.velocity_ned_ms.x);
    EXPECT_LT(output.velocity_error_ms.y, 0.0f);
    EXPECT_GT(output.velocity_error_ms.y, state.velocity_ned_ms.y);

    EXPECT_NEAR(output.specific_force_ned_mss.x,
                -output.position_error_m.x - output.velocity_error_ms.x,
                1.0e-6f);
    EXPECT_NEAR(output.specific_force_ned_mss.y,
                -output.position_error_m.y - output.velocity_error_ms.y,
                1.0e-6f);
}

TEST(AC_Geometric_Position_PID, IntegralStateIsConstrainedByImax)
{
    AC_Geometric_Position_PID controller;
    AC_Geometric_Position_Gains gains {};
    gains.i = Vector3f{1.0f, 1.0f, 1.0f};
    gains.integral_error_p = Vector3f{1.0f, 1.0f, 1.0f};
    controller.set_gains(gains);

    AC_Geometric_Position_Integral_Limits integral_limits {};
    integral_limits.integral_error_m = Vector3f{0.5f, 0.25f, 0.125f};
    controller.set_integral_limits(integral_limits);

    AC_Geometric_State state {};
    state.position_ned_m = Vector3f{10.0f, -10.0f, 10.0f};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);

    AC_Geometric_Target target {};
    target.build_attitude_from_position = true;

    AC_Geometric_Position_Output output {};
    for (uint8_t i = 0; i < 10; i++) {
        controller.update(state, target, 1.0f, output);
    }

    EXPECT_NEAR(output.specific_force_ned_mss.x, -0.5f, 1.0e-6f);
    EXPECT_NEAR(output.specific_force_ned_mss.y, 0.25f, 1.0e-6f);
    EXPECT_NEAR(output.specific_force_ned_mss.z, -GRAVITY_MSS - 0.125f, 1.0e-5f);
}

TEST(AC_Geometric_Position_PID, IntegralStateUsesVelocityPlusPositionWeight)
{
    AC_Geometric_Position_PID controller;
    AC_Geometric_Position_Gains gains {};
    gains.i = Vector3f{1.0f, 1.0f, 1.0f};
    gains.integral_error_p = Vector3f{0.5f, 0.5f, 0.5f};
    controller.set_gains(gains);

    AC_Geometric_Position_Integral_Limits integral_limits {};
    integral_limits.integral_error_m = Vector3f{10.0f, 10.0f, 10.0f};
    controller.set_integral_limits(integral_limits);

    AC_Geometric_State state {};
    state.position_ned_m = Vector3f{2.0f, -4.0f, 6.0f};
    state.velocity_ned_ms = Vector3f{1.0f, 2.0f, -3.0f};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);

    AC_Geometric_Target target {};
    target.build_attitude_from_position = true;

    AC_Geometric_Position_Output output {};
    controller.update(state, target, 1.0f, output);

    EXPECT_NEAR(output.integral_error_m.x, 2.0f, 1.0e-6f);
    EXPECT_NEAR(output.integral_error_m.y, 0.0f, 1.0e-6f);
    EXPECT_NEAR(output.integral_error_m.z, 0.0f, 1.0e-6f);
    EXPECT_NEAR(output.specific_force_ned_mss.x, -2.0f, 1.0e-6f);
    EXPECT_NEAR(output.specific_force_ned_mss.y, 0.0f, 1.0e-6f);
    EXPECT_NEAR(output.specific_force_ned_mss.z, -GRAVITY_MSS, 1.0e-5f);
}

TEST(AC_Geometric_Position_PID, NonUpwardForceUsesLevelYawAndZeroCollectiveDirection)
{
    AC_Geometric_State state {};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);

    AC_Geometric_Target target {};
    // The unconstrained request points below the NED horizontal plane.  Its
    // lateral part must not turn a zero/negative collective request into a
    // 90-plus-degree attitude target.
    target.accel_ned_mss = Vector3f{2.0f, -1.0f, GRAVITY_MSS + 0.5f};
    target.yaw_rad = 0.6f;
    target.build_attitude_from_position = true;

    AC_Geometric_Position_Gains gains {};
    const AC_Geometric_Position_Output output = run_position_pid(gains, state, target);

    // Preserve the raw force for diagnostics, but apply the physical zero
    // collective lower bound and use level-yaw for R_c.
    EXPECT_NEAR(output.specific_force_ned_mss.x, 2.0f, 1.0e-6f);
    EXPECT_NEAR(output.specific_force_ned_mss.y, -1.0f, 1.0e-6f);
    EXPECT_NEAR(output.specific_force_ned_mss.z, 0.5f, 1.0e-5f);
    EXPECT_NEAR(output.thrust_vector_ned.length(), 0.0f, 1.0e-6f);
    EXPECT_NEAR(output.thrust, 0.0f, 1.0e-6f);

    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    output.attitude_body_to_ned.to_euler(roll_rad, pitch_rad, yaw_rad);
    EXPECT_NEAR(roll_rad, 0.0f, 1.0e-6f);
    EXPECT_NEAR(pitch_rad, 0.0f, 1.0e-6f);
    EXPECT_NEAR(yaw_rad, target.yaw_rad, 1.0e-6f);
}

TEST(AC_Geometric_Position_PID, ZeroVerticalForceCannotLeakLateralForceIntoCollective)
{
    AC_Geometric_State state {};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.35f, 0.0f);

    AC_Geometric_Target target {};
    target.accel_ned_mss = Vector3f{-4.0f, 0.0f, GRAVITY_MSS};
    target.yaw_rad = 0.2f;
    target.build_attitude_from_position = true;

    AC_Geometric_Position_Gains gains {};
    const AC_Geometric_Position_Output output = run_position_pid(gains, state, target);

    Matrix3f actual_attitude;
    state.attitude_body_to_ned.rotation_matrix(actual_attitude);
    const float leaked_collective_without_lower_bound =
        -(output.specific_force_ned_mss * actual_attitude.colz());

    EXPECT_GT(leaked_collective_without_lower_bound, 1.0f);
    EXPECT_NEAR(output.specific_force_ned_mss.x, -4.0f, 1.0e-6f);
    EXPECT_NEAR(output.specific_force_ned_mss.z, 0.0f, 1.0e-6f);
    EXPECT_NEAR(output.thrust, 0.0f, 1.0e-6f);
    EXPECT_NEAR(output.thrust_vector_ned.length(), 0.0f, 1.0e-6f);
}

TEST(AC_Geometric_Position_PID, NearZeroUpwardForceCannotLeakLateralForceIntoCollective)
{
    AC_Geometric_State state {};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.35f, 0.0f);

    AC_Geometric_Target target {};
    target.accel_ned_mss = Vector3f{-4.0f, 0.0f, GRAVITY_MSS - 0.1f};
    target.yaw_rad = -0.2f;
    target.build_attitude_from_position = true;

    AC_Geometric_Position_Gains gains {};
    const AC_Geometric_Position_Output output = run_position_pid(gains, state, target);

    Matrix3f actual_attitude;
    state.attitude_body_to_ned.rotation_matrix(actual_attitude);
    const float leaked_collective_without_regularized_force =
        -(output.specific_force_ned_mss * actual_attitude.colz());
    const float expected_vertical_collective =
        -output.specific_force_ned_mss.z * actual_attitude.colz().z;

    EXPECT_GT(leaked_collective_without_regularized_force, 1.0f);
    EXPECT_NEAR(output.specific_force_ned_mss.x, -4.0f, 1.0e-6f);
    EXPECT_NEAR(output.specific_force_ned_mss.z, -0.1f, 1.0e-5f);
    EXPECT_NEAR(output.thrust_vector_ned.length(), 0.0f, 1.0e-6f);
    EXPECT_GT(output.thrust, 0.0f);
    EXPECT_NEAR(output.thrust, expected_vertical_collective, 1.0e-6f);
}

TEST(AC_Geometric_Position_PID, GroundedDescentCannotWindUpIntoInvertedTarget)
{
    AC_Geometric_Position_PID controller;

    AC_Geometric_Position_Gains gains {};
    // Match the default-style gains from the touchdown failure: D initially
    // leaves ample upward force, then I moves it toward zero over a few seconds.
    gains.d.z = 3.0f;
    gains.i.z = 5.0f;
    controller.set_gains(gains);

    AC_Geometric_Position_Integral_Limits integral_limits {};
    integral_limits.integral_error_m.z = 100.0f;
    controller.set_integral_limits(integral_limits);

    AC_Geometric_State state {};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);

    AC_Geometric_Target target {};
    // Include a small lateral residual: without the feasibility boundary this
    // removes the antipodal-axis degeneracy and exposes the near-180-degree
    // roll/pitch target seen at touchdown.
    target.accel_ned_mss.x = 0.1f;
    target.velocity_ned_ms.z = 0.7f;
    target.yaw_rad = -0.4f;
    target.build_attitude_from_position = true;

    AC_Geometric_Position_Output output {};
    float minimum_body_z_down = 1.0f;
    float integral_after_three_seconds = 0.0f;
    bool reached_level_yaw_regularisation = false;
    // Exercise longer than the Loiter landing timeout that exposed the fault.
    for (uint16_t i = 0; i < 6000; i++) {
        controller.update(state, target, 0.01f, output);
        Matrix3f attitude;
        output.attitude_body_to_ned.rotation_matrix(attitude);
        minimum_body_z_down = MIN(minimum_body_z_down, attitude.colz().z);
        reached_level_yaw_regularisation |= is_zero(output.thrust_vector_ned.length_squared());
        if (i == 299) {
            integral_after_three_seconds = output.integral_error_m.z;
        }
    }

    const float specific_force_without_integral_z_mss = gains.d.z * target.velocity_ned_ms.z - GRAVITY_MSS;
    const float expected_zero_force_integral_z_m = specific_force_without_integral_z_mss / gains.i.z;
    EXPECT_TRUE(reached_level_yaw_regularisation);
    EXPECT_NEAR(output.specific_force_ned_mss.z, 0.0f, 1.0e-5f);
    EXPECT_NEAR(output.thrust_vector_ned.length(), 0.0f, 1.0e-6f);
    EXPECT_NEAR(output.thrust, 0.0f, 1.0e-5f);
    EXPECT_NEAR(output.integral_error_m.z, expected_zero_force_integral_z_m, 1.0e-5f);
    EXPECT_NEAR(integral_after_three_seconds, output.integral_error_m.z, 1.0e-6f);
    EXPECT_GT(minimum_body_z_down, cosf(radians(15.0f)));

    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    output.attitude_body_to_ned.to_euler(roll_rad, pitch_rad, yaw_rad);
    EXPECT_NEAR(roll_rad, 0.0f, 1.0e-6f);
    EXPECT_NEAR(pitch_rad, 0.0f, 1.0e-6f);
    EXPECT_NEAR(yaw_rad, target.yaw_rad, 1.0e-6f);

    // Reverse the vertical error. The force increment now points back into the
    // attainable domain, so conditional anti-windup must release immediately.
    target.velocity_ned_ms.z = -0.7f;
    for (uint8_t i = 0; i < 100; i++) {
        controller.update(state, target, 0.01f, output);
    }
    EXPECT_GT(output.integral_error_m.z, expected_zero_force_integral_z_m);
    EXPECT_LT(output.specific_force_ned_mss.z, -GRAVITY_MSS * 0.05f);
    EXPECT_GT(output.thrust_vector_ned.length(), 0.0f);
}

TEST(AC_Geometric_Position_PID, UpwardTiltForceIsNotProjected)
{
    AC_Geometric_State state {};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);

    AC_Geometric_Target target {};
    target.accel_ned_mss = Vector3f{2.0f, -1.0f, 0.0f};
    target.yaw_rad = 0.3f;
    target.build_attitude_from_position = true;

    AC_Geometric_Position_Gains gains {};
    const AC_Geometric_Position_Output output = run_position_pid(gains, state, target);

    EXPECT_NEAR(output.thrust_vector_ned.x, output.specific_force_ned_mss.x, 1.0e-6f);
    EXPECT_NEAR(output.thrust_vector_ned.y, output.specific_force_ned_mss.y, 1.0e-6f);
    EXPECT_NEAR(output.thrust_vector_ned.z, output.specific_force_ned_mss.z, 1.0e-6f);

    Matrix3f attitude;
    output.attitude_body_to_ned.rotation_matrix(attitude);
    const Vector3f thrust_direction_ned = normalized(output.thrust_vector_ned);
    EXPECT_NEAR(thrust_direction_ned.x, -attitude.colz().x, 1.0e-5f);
    EXPECT_NEAR(thrust_direction_ned.y, -attitude.colz().y, 1.0e-5f);
    EXPECT_NEAR(thrust_direction_ned.z, -attitude.colz().z, 1.0e-5f);
    EXPECT_GT(attitude.colz().z, 0.0f);
}

TEST(AC_Geometric_Position_PID, NearZeroAttitudeRegularizationHasHysteresisAndRateReset)
{
    AC_Geometric_Position_PID controller;
    AC_Geometric_Position_Gains gains {};
    controller.set_gains(gains);

    AC_Geometric_State state {};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);

    AC_Geometric_Target target {};
    target.accel_ned_mss.x = 0.1f;
    target.yaw_rad = 0.3f;
    target.build_attitude_from_position = true;

    AC_Geometric_Position_Output output {};
    // Establish a normal upward-force attitude and a valid finite-difference
    // history before crossing the regularisation boundary.
    controller.update(state, target, 0.01f, output);
    controller.update(state, target, 0.01f, output);
    EXPECT_GT(output.thrust_vector_ned.length(), 0.0f);

    // Four percent of hover force enters the level-yaw domain. The upward
    // collective remains available; only its ill-conditioned direction is
    // regularised. The transition uses the caller fallback, not delta-R/dt.
    target.accel_ned_mss.z = GRAVITY_MSS * 0.96f;
    target.omega_body_rads = Vector3f{0.12f, -0.08f, 0.9f};
    target.yaw_rate_rads = 0.2f;
    controller.update(state, target, 0.01f, output);
    EXPECT_NEAR(output.thrust, GRAVITY_MSS * 0.04f, 1.0e-5f);
    EXPECT_NEAR(output.thrust_vector_ned.length(), 0.0f, 1.0e-6f);
    EXPECT_NEAR(output.omega_body_rads.x, target.omega_body_rads.x, 1.0e-6f);
    EXPECT_NEAR(output.omega_body_rads.y, target.omega_body_rads.y, 1.0e-6f);
    EXPECT_NEAR(output.omega_body_rads.z, target.yaw_rate_rads, 1.0e-6f);
    EXPECT_NEAR(output.omega_dot_body_radss.length(), 0.0f, 1.0e-6f);

    // Six percent lies between the 5% entry and 7% release thresholds, so it
    // must remain regularised. The post-transition derivative reset suppresses
    // a one-frame omega-dot impulse from the fallback seed.
    target.accel_ned_mss.z = GRAVITY_MSS * 0.94f;
    controller.update(state, target, 0.01f, output);
    EXPECT_NEAR(output.thrust_vector_ned.length(), 0.0f, 1.0e-6f);
    EXPECT_NEAR(output.omega_dot_body_radss.length(), 0.0f, 1.0e-6f);

    // Eight percent releases regularisation. This attitude switch also uses
    // fallback rates and resets the derivative path.
    target.accel_ned_mss.z = GRAVITY_MSS * 0.92f;
    target.omega_body_rads = Vector3f{-0.11f, 0.07f, -0.8f};
    target.yaw_rate_rads = -0.15f;
    controller.update(state, target, 0.01f, output);
    EXPECT_GT(output.thrust_vector_ned.length(), 0.0f);
    EXPECT_NEAR(output.omega_body_rads.x, target.omega_body_rads.x, 1.0e-6f);
    EXPECT_NEAR(output.omega_body_rads.y, target.omega_body_rads.y, 1.0e-6f);
    EXPECT_NEAR(output.omega_body_rads.z, target.yaw_rate_rads, 1.0e-6f);
    EXPECT_NEAR(output.omega_dot_body_radss.length(), 0.0f, 1.0e-6f);

    controller.update(state, target, 0.01f, output);
    EXPECT_NEAR(output.omega_dot_body_radss.length(), 0.0f, 1.0e-6f);

    // On the released side, 6% is retained by hysteresis; falling to 4%
    // re-enters level-yaw regularisation.
    target.accel_ned_mss.z = GRAVITY_MSS * 0.94f;
    controller.update(state, target, 0.01f, output);
    EXPECT_GT(output.thrust_vector_ned.length(), 0.0f);
    target.accel_ned_mss.z = GRAVITY_MSS * 0.96f;
    controller.update(state, target, 0.01f, output);
    EXPECT_NEAR(output.thrust_vector_ned.length(), 0.0f, 1.0e-6f);
}

AP_GTEST_MAIN()
