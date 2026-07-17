#include <AP_gtest.h>

#include <AC_GeometricControl/AC_Geometric_Attitude_PID.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

Quaternion attitude_from_euler(float roll_rad, float pitch_rad, float yaw_rad)
{
    Quaternion attitude;
    attitude.from_euler(roll_rad, pitch_rad, yaw_rad);
    return attitude;
}

void set_unit_inertia(AC_Geometric_Attitude_PID& controller)
{
    AC_Geometric_Attitude_Model model {};
    model.inertia = Vector3f{1.0f, 1.0f, 1.0f};
    controller.set_model(model);
}

AC_Geometric_Attitude_Output run_attitude_pid(const AC_Geometric_Attitude_Gains& gains,
                                              const AC_Geometric_State& state,
                                              const AC_Geometric_Target& target)
{
    AC_Geometric_Attitude_PID controller;
    AC_Geometric_Attitude_Output output {};

    set_unit_inertia(controller);
    controller.set_gains(gains);
    controller.update(state, target, 0.01f, output);

    return output;
}

Vector3f rotate_target_body_to_current_body(const Quaternion& attitude_body_to_ned,
                                            const Quaternion& attitude_target_to_ned,
                                            const Vector3f& vector_target_body)
{
    Matrix3f attitude;
    Matrix3f attitude_target;

    attitude_body_to_ned.rotation_matrix(attitude);
    attitude_target_to_ned.rotation_matrix(attitude_target);

    return attitude.mul_transpose(attitude_target * vector_target_body);
}

}

TEST(AC_Geometric_Attitude_PID, DefaultModelUsesGaoReferenceInertia)
{
    AC_Geometric_Attitude_PID controller;

    const AC_Geometric_Attitude_Model& model = controller.get_model();

    EXPECT_NEAR(model.inertia.x, 0.011f, 1.0e-6f);
    EXPECT_NEAR(model.inertia.y, 0.020f, 1.0e-6f);
    EXPECT_NEAR(model.inertia.z, 0.023f, 1.0e-6f);
}

TEST(AC_Geometric_Attitude_PID, PositiveTargetAnglesProduceNegativeLeeError)
{
    const float angle_rad = 0.1f;

    AC_Geometric_State state {};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);

    AC_Geometric_Attitude_Gains gains {};
    gains.attitude_p = Vector3f{2.0f, 3.0f, 4.0f};

    {
        AC_Geometric_Target target {};
        target.attitude_body_to_ned = attitude_from_euler(angle_rad, 0.0f, 0.0f);

        const AC_Geometric_Attitude_Output output = run_attitude_pid(gains, state, target);

        EXPECT_NEAR(output.attitude_error.x, -sinf(angle_rad), 1.0e-5f);
        EXPECT_NEAR(output.attitude_error.y, 0.0f, 1.0e-6f);
        EXPECT_NEAR(output.attitude_error.z, 0.0f, 1.0e-6f);
        EXPECT_NEAR(output.moment.x, gains.attitude_p.x * sinf(angle_rad), 1.0e-5f);
    }

    {
        AC_Geometric_Target target {};
        target.attitude_body_to_ned = attitude_from_euler(0.0f, angle_rad, 0.0f);

        const AC_Geometric_Attitude_Output output = run_attitude_pid(gains, state, target);

        EXPECT_NEAR(output.attitude_error.x, 0.0f, 1.0e-6f);
        EXPECT_NEAR(output.attitude_error.y, -sinf(angle_rad), 1.0e-5f);
        EXPECT_NEAR(output.attitude_error.z, 0.0f, 1.0e-6f);
        EXPECT_NEAR(output.moment.y, gains.attitude_p.y * sinf(angle_rad), 1.0e-5f);
    }

    {
        AC_Geometric_Target target {};
        target.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, angle_rad);

        const AC_Geometric_Attitude_Output output = run_attitude_pid(gains, state, target);

        EXPECT_NEAR(output.attitude_error.x, 0.0f, 1.0e-6f);
        EXPECT_NEAR(output.attitude_error.y, 0.0f, 1.0e-6f);
        EXPECT_NEAR(output.attitude_error.z, -sinf(angle_rad), 1.0e-5f);
        EXPECT_NEAR(output.moment.z, gains.attitude_p.z * sinf(angle_rad), 1.0e-5f);
    }
}

TEST(AC_Geometric_Attitude_PID, ReportsGlobalSO3ConfigurationError)
{
    AC_Geometric_State state {};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);

    const AC_Geometric_Attitude_Gains gains {};
    const float angles_rad[] {
        0.0f,
        radians(60.0f),
        radians(120.0f),
        radians(180.0f),
    };

    for (const float angle_rad : angles_rad) {
        AC_Geometric_Target target {};
        target.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, angle_rad);

        const AC_Geometric_Attitude_Output output = run_attitude_pid(gains, state, target);

        EXPECT_NEAR(output.attitude_configuration_error,
                    1.0f - cosf(angle_rad),
                    1.0e-5f);
        EXPECT_NEAR(output.attitude_error_angle_rad, angle_rad, 1.0e-4f);
        EXPECT_NEAR(output.attitude_error.length(), fabsf(sinf(angle_rad)), 1.0e-5f);
    }
}

TEST(AC_Geometric_Attitude_PID, AngularVelocityErrorProducesDampingMoment)
{
    AC_Geometric_State state {};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);
    state.omega_body_rads = Vector3f{0.4f, -0.3f, 0.2f};

    AC_Geometric_Target target {};
    target.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);

    AC_Geometric_Attitude_Gains gains {};
    gains.omega_p = Vector3f{2.0f, 3.0f, 4.0f};

    const AC_Geometric_Attitude_Output output = run_attitude_pid(gains, state, target);

    EXPECT_NEAR(output.omega_error_rads.x, 0.4f, 1.0e-6f);
    EXPECT_NEAR(output.omega_error_rads.y, -0.3f, 1.0e-6f);
    EXPECT_NEAR(output.omega_error_rads.z, 0.2f, 1.0e-6f);

    EXPECT_NEAR(output.moment.x, -0.8f, 1.0e-6f);
    EXPECT_NEAR(output.moment.y, 0.9f, 1.0e-6f);
    EXPECT_NEAR(output.moment.z, -0.8f, 1.0e-6f);
}

TEST(AC_Geometric_Attitude_PID, AngularVelocityErrorUsesLeeRelativeAttitude)
{
    AC_Geometric_State state {};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.2f);
    state.omega_body_rads = Vector3f{0.2f, -0.1f, 0.3f};

    AC_Geometric_Target target {};
    target.attitude_body_to_ned = attitude_from_euler(0.1f, -0.2f, 0.7f);
    target.omega_body_rads = Vector3f{0.5f, 0.1f, -0.2f};

    AC_Geometric_Attitude_Gains gains {};
    gains.omega_p = Vector3f{1.0f, 1.0f, 1.0f};

    const AC_Geometric_Attitude_Output output = run_attitude_pid(gains, state, target);
    const Vector3f omega_target_current_body =
        rotate_target_body_to_current_body(state.attitude_body_to_ned,
                                           target.attitude_body_to_ned,
                                           target.omega_body_rads);
    const Vector3f expected_error = state.omega_body_rads - omega_target_current_body;

    EXPECT_NEAR(output.omega_error_rads.x, expected_error.x, 1.0e-6f);
    EXPECT_NEAR(output.omega_error_rads.y, expected_error.y, 1.0e-6f);
    EXPECT_NEAR(output.omega_error_rads.z, expected_error.z, 1.0e-6f);
}

TEST(AC_Geometric_Attitude_PID, LeeFeedForwardUsesTransportAndOmegaDot)
{
    AC_Geometric_State state {};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);
    state.omega_body_rads = Vector3f{0.0f, 0.0f, 1.0f};

    AC_Geometric_Target target {};
    target.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);
    target.omega_body_rads = Vector3f{1.0f, 0.0f, 0.0f};
    target.omega_dot_body_radss = Vector3f{0.2f, 0.3f, 0.4f};

    AC_Geometric_Attitude_Gains gains {};

    const AC_Geometric_Attitude_Output output = run_attitude_pid(gains, state, target);

    EXPECT_NEAR(output.moment.x, 0.2f, 1.0e-6f);
    EXPECT_NEAR(output.moment.y, -0.7f, 1.0e-6f);
    EXPECT_NEAR(output.moment.z, 0.4f, 1.0e-6f);
}

TEST(AC_Geometric_Attitude_PID, LeeFeedForwardUsesDiagonalInertia)
{
    AC_Geometric_Attitude_PID controller;

    AC_Geometric_Attitude_Model model {};
    model.inertia = Vector3f{2.0f, 3.0f, 4.0f};
    controller.set_model(model);

    AC_Geometric_State state {};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);
    state.omega_body_rads = Vector3f{0.0f, 0.0f, 1.0f};

    AC_Geometric_Target target {};
    target.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);
    target.omega_body_rads = Vector3f{1.0f, 0.0f, 0.0f};
    target.omega_dot_body_radss = Vector3f{0.2f, 0.3f, 0.4f};

    AC_Geometric_Attitude_Output output {};
    controller.update(state, target, 0.01f, output);

    EXPECT_NEAR(output.moment.x, 0.4f, 1.0e-6f);
    EXPECT_NEAR(output.moment.y, -2.1f, 1.0e-6f);
    EXPECT_NEAR(output.moment.z, 1.6f, 1.0e-6f);
}

TEST(AC_Geometric_Attitude_PID, AttitudeGainScalesMoment)
{
    const float angle_rad = 0.2f;

    AC_Geometric_State state {};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);

    AC_Geometric_Target target {};
    target.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, angle_rad);

    AC_Geometric_Attitude_Gains low_gains {};
    low_gains.attitude_p.z = 2.0f;
    const AC_Geometric_Attitude_Output low_output = run_attitude_pid(low_gains, state, target);

    AC_Geometric_Attitude_Gains high_gains {};
    high_gains.attitude_p.z = 4.0f;
    const AC_Geometric_Attitude_Output high_output = run_attitude_pid(high_gains, state, target);

    EXPECT_NEAR(low_output.moment.z, low_gains.attitude_p.z * sinf(angle_rad), 1.0e-5f);
    EXPECT_NEAR(high_output.moment.z, high_gains.attitude_p.z * sinf(angle_rad), 1.0e-5f);
    EXPECT_NEAR(high_output.moment.z, 2.0f * low_output.moment.z, 1.0e-5f);
}

TEST(AC_Geometric_Attitude_PID, OptionalOmegaFilterSmoothsRateErrorStep)
{
    AC_Geometric_Attitude_PID controller;
    set_unit_inertia(controller);

    AC_Geometric_Attitude_Gains gains {};
    gains.omega_p = Vector3f{1.0f, 1.0f, 1.0f};
    controller.set_gains(gains);

    AC_Geometric_Attitude_Filter_Hz filter_hz {};
    filter_hz.omega_error = 20.0f;
    controller.set_filter_hz(filter_hz);

    AC_Geometric_State state {};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);

    AC_Geometric_Target target {};
    target.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);

    AC_Geometric_Attitude_Output output {};
    controller.update(state, target, 0.01f, output);

    state.omega_body_rads = Vector3f{1.0f, -1.0f, 0.5f};
    controller.update(state, target, 0.01f, output);

    EXPECT_GT(output.omega_error_rads.x, 0.0f);
    EXPECT_LT(output.omega_error_rads.x, state.omega_body_rads.x);
    EXPECT_LT(output.omega_error_rads.y, 0.0f);
    EXPECT_GT(output.omega_error_rads.y, state.omega_body_rads.y);
    EXPECT_GT(output.omega_error_rads.z, 0.0f);
    EXPECT_LT(output.omega_error_rads.z, state.omega_body_rads.z);
    EXPECT_NEAR(output.moment.x, -output.omega_error_rads.x, 1.0e-6f);
    EXPECT_NEAR(output.moment.y, -output.omega_error_rads.y, 1.0e-6f);
    EXPECT_NEAR(output.moment.z, -output.omega_error_rads.z, 1.0e-6f);
}

TEST(AC_Geometric_Attitude_PID, YawIntegralIsConstrainedAndYawOnlyByDefault)
{
    AC_Geometric_Attitude_PID controller;
    set_unit_inertia(controller);

    AC_Geometric_Attitude_Gains gains {};
    gains.attitude_i = Vector3f{0.0f, 0.0f, 2.0f};
    gains.integral_error_p = Vector3f{};
    controller.set_gains(gains);

    AC_Geometric_Attitude_Integral_Limits integral_limits {};
    integral_limits.integral_error = Vector3f{0.3f, 0.3f, 0.3f};
    controller.set_integral_limits(integral_limits);

    AC_Geometric_State state {};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);
    state.omega_body_rads = Vector3f{0.5f, -0.4f, 0.5f};

    AC_Geometric_Target target {};
    target.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);

    AC_Geometric_Attitude_Output output {};
    for (uint8_t i = 0; i < 3; i++) {
        controller.update(state, target, 1.0f, output);
    }

    EXPECT_NEAR(output.integral_error.x, 0.0f, 1.0e-6f);
    EXPECT_NEAR(output.integral_error.y, 0.0f, 1.0e-6f);
    EXPECT_NEAR(output.integral_error.z, 0.3f, 1.0e-6f);
    EXPECT_NEAR(output.moment.x, 0.0f, 1.0e-6f);
    EXPECT_NEAR(output.moment.y, 0.0f, 1.0e-6f);
    EXPECT_NEAR(output.moment.z, -0.6f, 1.0e-6f);
}

TEST(AC_Geometric_Attitude_PID, RollPitchIntegralCanBeEnabledExplicitly)
{
    AC_Geometric_Attitude_PID controller;
    set_unit_inertia(controller);

    AC_Geometric_Attitude_Gains gains {};
    gains.attitude_i = Vector3f{1.0f, 1.0f, 0.0f};
    gains.integral_error_p = Vector3f{};
    controller.set_gains(gains);

    AC_Geometric_Attitude_Integral_Limits integral_limits {};
    integral_limits.integral_error = Vector3f{0.2f, 0.25f, 0.3f};
    controller.set_integral_limits(integral_limits);

    AC_Geometric_State state {};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);
    state.omega_body_rads = Vector3f{1.0f, -1.0f, 0.5f};

    AC_Geometric_Target target {};
    target.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);

    AC_Geometric_Attitude_Output output {};
    controller.update(state, target, 1.0f, output);

    EXPECT_NEAR(output.integral_error.x, 0.2f, 1.0e-6f);
    EXPECT_NEAR(output.integral_error.y, -0.25f, 1.0e-6f);
    EXPECT_NEAR(output.integral_error.z, 0.0f, 1.0e-6f);
    EXPECT_NEAR(output.moment.x, -0.2f, 1.0e-6f);
    EXPECT_NEAR(output.moment.y, 0.25f, 1.0e-6f);
    EXPECT_NEAR(output.moment.z, 0.0f, 1.0e-6f);
}

AP_GTEST_MAIN()
