#include <AP_gtest.h>

#include <AC_GeometricControl/AC_Geometric_Attitude_PD.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

Quaternion attitude_from_euler(float roll_rad, float pitch_rad, float yaw_rad)
{
    Quaternion attitude;
    attitude.from_euler(roll_rad, pitch_rad, yaw_rad);
    return attitude;
}

AC_Geometric_Attitude_Output run_attitude_pd(const AC_Geometric_Attitude_Gains& gains,
                                             const AC_Geometric_State& state,
                                             const AC_Geometric_Target& target)
{
    AC_Geometric_Attitude_PD controller;
    AC_Geometric_Attitude_Output output {};

    controller.set_gains(gains);
    controller.update(state, target, 0.01f, output);

    return output;
}

}

TEST(AC_Geometric_Attitude_PD, PositiveTargetAnglesProduceNegativeLeeError)
{
    const float angle_rad = 0.1f;

    AC_Geometric_State state {};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);

    AC_Geometric_Attitude_Gains gains {};
    gains.attitude_p = Vector3f{2.0f, 3.0f, 4.0f};

    {
        AC_Geometric_Target target {};
        target.attitude_body_to_ned = attitude_from_euler(angle_rad, 0.0f, 0.0f);

        const AC_Geometric_Attitude_Output output = run_attitude_pd(gains, state, target);

        EXPECT_NEAR(output.attitude_error.x, -sinf(angle_rad), 1.0e-5f);
        EXPECT_NEAR(output.attitude_error.y, 0.0f, 1.0e-6f);
        EXPECT_NEAR(output.attitude_error.z, 0.0f, 1.0e-6f);
        EXPECT_NEAR(output.moment.x, gains.attitude_p.x * sinf(angle_rad), 1.0e-5f);
    }

    {
        AC_Geometric_Target target {};
        target.attitude_body_to_ned = attitude_from_euler(0.0f, angle_rad, 0.0f);

        const AC_Geometric_Attitude_Output output = run_attitude_pd(gains, state, target);

        EXPECT_NEAR(output.attitude_error.x, 0.0f, 1.0e-6f);
        EXPECT_NEAR(output.attitude_error.y, -sinf(angle_rad), 1.0e-5f);
        EXPECT_NEAR(output.attitude_error.z, 0.0f, 1.0e-6f);
        EXPECT_NEAR(output.moment.y, gains.attitude_p.y * sinf(angle_rad), 1.0e-5f);
    }

    {
        AC_Geometric_Target target {};
        target.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, angle_rad);

        const AC_Geometric_Attitude_Output output = run_attitude_pd(gains, state, target);

        EXPECT_NEAR(output.attitude_error.x, 0.0f, 1.0e-6f);
        EXPECT_NEAR(output.attitude_error.y, 0.0f, 1.0e-6f);
        EXPECT_NEAR(output.attitude_error.z, -sinf(angle_rad), 1.0e-5f);
        EXPECT_NEAR(output.moment.z, gains.attitude_p.z * sinf(angle_rad), 1.0e-5f);
    }
}

TEST(AC_Geometric_Attitude_PD, AngularVelocityErrorProducesDampingMoment)
{
    AC_Geometric_State state {};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);
    state.omega_body_rads = Vector3f{0.4f, -0.3f, 0.2f};

    AC_Geometric_Target target {};
    target.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);
    target.omega_body_rads = Vector3f{0.1f, 0.2f, -0.1f};

    AC_Geometric_Attitude_Gains gains {};
    gains.omega_p = Vector3f{2.0f, 3.0f, 4.0f};

    const AC_Geometric_Attitude_Output output = run_attitude_pd(gains, state, target);

    EXPECT_NEAR(output.omega_error_rads.x, 0.3f, 1.0e-6f);
    EXPECT_NEAR(output.omega_error_rads.y, -0.5f, 1.0e-6f);
    EXPECT_NEAR(output.omega_error_rads.z, 0.3f, 1.0e-6f);

    EXPECT_NEAR(output.moment.x, -0.6f, 1.0e-6f);
    EXPECT_NEAR(output.moment.y, 1.5f, 1.0e-6f);
    EXPECT_NEAR(output.moment.z, -1.2f, 1.0e-6f);
}

TEST(AC_Geometric_Attitude_PD, AttitudeGainScalesMoment)
{
    const float angle_rad = 0.2f;

    AC_Geometric_State state {};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);

    AC_Geometric_Target target {};
    target.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, angle_rad);

    AC_Geometric_Attitude_Gains low_gains {};
    low_gains.attitude_p.z = 2.0f;
    const AC_Geometric_Attitude_Output low_output = run_attitude_pd(low_gains, state, target);

    AC_Geometric_Attitude_Gains high_gains {};
    high_gains.attitude_p.z = 4.0f;
    const AC_Geometric_Attitude_Output high_output = run_attitude_pd(high_gains, state, target);

    EXPECT_NEAR(low_output.moment.z, low_gains.attitude_p.z * sinf(angle_rad), 1.0e-5f);
    EXPECT_NEAR(high_output.moment.z, high_gains.attitude_p.z * sinf(angle_rad), 1.0e-5f);
    EXPECT_NEAR(high_output.moment.z, 2.0f * low_output.moment.z, 1.0e-5f);
}

TEST(AC_Geometric_Attitude_PD, OptionalOmegaFilterSmoothsRateErrorStep)
{
    AC_Geometric_Attitude_PD controller;

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

AP_GTEST_MAIN()
