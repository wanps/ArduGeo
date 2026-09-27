#include <AP_gtest.h>

#include <vector>

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

TEST(AC_Geometric_Attitude_PID, ParameterDefaultsMatchRuntimeBaseline)
{
    AC_Geometric_Attitude_PID_Params params;
    const AC_Geometric_Attitude_Gains gains = params.gains();
    const AC_Geometric_Attitude_Model model = params.model();
    const AC_Geometric_Attitude_Filter_Hz filters = params.filter_hz();
    const AC_Geometric_Attitude_Integral_Limits limits = params.integral_limits();

    EXPECT_EQ(gains.attitude_p, Vector3f(4.0f, 4.0f, 2.0f));
    EXPECT_EQ(gains.omega_p, Vector3f(0.2f, 0.2f, 0.4f));
    EXPECT_EQ(gains.attitude_i, Vector3f(0.0f, 0.0f, 0.1f));
    EXPECT_EQ(gains.integral_error_p, Vector3f(0.5f, 0.5f, 0.5f));
    EXPECT_EQ(model.inertia, Vector3f(0.010f, 0.020f, 0.020f));
    EXPECT_EQ(limits.integral_error, Vector3f(0.0f, 0.0f, 1.0f));
    EXPECT_FLOAT_EQ(filters.omega_error, 0.0f);
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


// The lead term is the geometric counterpart of the Native rate-controller
// derivative term. These lock down the three properties that make it safe to
// enable: it is inert at zero gain, it cannot kick on the first frame after a
// reset, and it opposes a growing angular-rate error.
namespace {

// Drives the controller repeatedly so the derivative state is exercised, and
// returns the output of the final update.
AC_Geometric_Attitude_Output run_steps(const AC_Geometric_Attitude_Gains& gains,
                                       const AC_Geometric_Attitude_Filter_Hz& filters,
                                       const std::vector<Vector3f>& omega_sequence,
                                       float dt)
{
    AC_Geometric_Attitude_PID controller;
    AC_Geometric_Attitude_Output output {};
    set_unit_inertia(controller);
    controller.set_gains(gains);
    controller.set_filter_hz(filters);

    AC_Geometric_State state {};
    AC_Geometric_Target target {};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);
    target.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);

    for (const Vector3f& omega : omega_sequence) {
        state.omega_body_rads = omega;
        controller.update(state, target, dt, output);
    }
    return output;
}

AC_Geometric_Attitude_Gains lead_gains(float kd)
{
    AC_Geometric_Attitude_Gains gains {};
    gains.attitude_p = Vector3f{1.0f, 1.0f, 1.0f};
    gains.omega_p = Vector3f{0.1f, 0.1f, 0.1f};
    gains.omega_d = Vector3f{kd, kd, kd};
    return gains;
}

}  // namespace

TEST(AC_Geometric_Attitude_PID, LeadTermIsInertAtZeroGain)
{
    AC_Geometric_Attitude_Filter_Hz filters {};   // both filters disabled
    const std::vector<Vector3f> ramp {
        Vector3f{0.00f, 0.0f, 0.0f},
        Vector3f{0.05f, 0.0f, 0.0f},
        Vector3f{0.10f, 0.0f, 0.0f},
        Vector3f{0.15f, 0.0f, 0.0f},
    };

    const AC_Geometric_Attitude_Output without = run_steps(lead_gains(0.0f), filters, ramp, 0.01f);
    // A zero gain must leave the moment bit-identical to the pre-lead law, which
    // for this state is just the rate term.
    EXPECT_FLOAT_EQ(-0.1f * without.omega_error_rads.x, without.moment.x);
    // The derivative is still reported, so the diagnostic stays usable, but it
    // contributes nothing.
    EXPECT_GT(without.omega_error_derivative_radss.x, 0.0f);
}

TEST(AC_Geometric_Attitude_PID, LeadTermDoesNotKickOnFirstUpdate)
{
    AC_Geometric_Attitude_Filter_Hz filters {};
    // A large rate error present on the very first update would differentiate to
    // a huge value if the state were not seeded.
    const std::vector<Vector3f> step { Vector3f{0.5f, 0.0f, 0.0f} };

    const AC_Geometric_Attitude_Output output = run_steps(lead_gains(0.02f), filters, step, 0.01f);
    EXPECT_FLOAT_EQ(0.0f, output.omega_error_derivative_radss.x);
    EXPECT_FLOAT_EQ(-0.1f * output.omega_error_rads.x, output.moment.x);
}

TEST(AC_Geometric_Attitude_PID, LeadTermOpposesGrowingRateError)
{
    AC_Geometric_Attitude_Filter_Hz filters {};
    const std::vector<Vector3f> ramp {
        Vector3f{0.00f, 0.0f, 0.0f},
        Vector3f{0.05f, 0.0f, 0.0f},
        Vector3f{0.10f, 0.0f, 0.0f},
    };
    const float kd = 0.02f;

    const AC_Geometric_Attitude_Output with = run_steps(lead_gains(kd), filters, ramp, 0.01f);
    const AC_Geometric_Attitude_Output without = run_steps(lead_gains(0.0f), filters, ramp, 0.01f);

    // e_Omega is growing positive, so the lead term must push the moment further
    // negative than the law without it.
    EXPECT_LT(with.moment.x, without.moment.x);
    EXPECT_FLOAT_EQ(without.moment.x - kd * with.omega_error_derivative_radss.x,
                    with.moment.x);
}

TEST(AC_Geometric_Attitude_PID, LeadTermDerivativeFilterAttenuates)
{
    const std::vector<Vector3f> ramp {
        Vector3f{0.00f, 0.0f, 0.0f},
        Vector3f{0.05f, 0.0f, 0.0f},
        Vector3f{0.10f, 0.0f, 0.0f},
        Vector3f{0.15f, 0.0f, 0.0f},
    };

    AC_Geometric_Attitude_Filter_Hz unfiltered {};
    AC_Geometric_Attitude_Filter_Hz filtered {};
    filtered.omega_error_derivative = 5.0f;

    const AC_Geometric_Attitude_Output raw = run_steps(lead_gains(0.02f), unfiltered, ramp, 0.01f);
    const AC_Geometric_Attitude_Output smooth = run_steps(lead_gains(0.02f), filtered, ramp, 0.01f);

    // A ramp gives a constant raw derivative; a low-pass started from zero has
    // not reached it yet, so the filtered value must be smaller and positive.
    EXPECT_GT(raw.omega_error_derivative_radss.x, smooth.omega_error_derivative_radss.x);
    EXPECT_GT(smooth.omega_error_derivative_radss.x, 0.0f);
}

TEST(AC_Geometric_Attitude_PID, LeadTermResetClearsDerivativeState)
{
    AC_Geometric_Attitude_Filter_Hz filters {};
    AC_Geometric_Attitude_PID controller;
    AC_Geometric_Attitude_Output output {};
    set_unit_inertia(controller);
    controller.set_gains(lead_gains(0.02f));
    controller.set_filter_hz(filters);

    AC_Geometric_State state {};
    AC_Geometric_Target target {};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);
    target.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);

    for (int i = 0; i < 4; i++) {
        state.omega_body_rads = Vector3f{0.05f * i, 0.0f, 0.0f};
        controller.update(state, target, 0.01f, output);
    }
    EXPECT_GT(output.omega_error_derivative_radss.x, 0.0f);

    // After a reset the next update must behave like a first update again.
    controller.reset();
    state.omega_body_rads = Vector3f{0.5f, 0.0f, 0.0f};
    controller.update(state, target, 0.01f, output);
    EXPECT_FLOAT_EQ(0.0f, output.omega_error_derivative_radss.x);
}


TEST(AC_Geometric_Attitude_PID, LeadTermDoesNotDifferentiateTheReference)
{
    // A reference rate that steps between updates must not appear in the lead
    // term: the reference contribution is analytic, so a step in Omega_ref with
    // a constant measured rate leaves the derivative at zero apart from the
    // transport term, which is zero here because Omega is zero.
    AC_Geometric_Attitude_PID controller;
    AC_Geometric_Attitude_Output output {};
    set_unit_inertia(controller);
    controller.set_gains(lead_gains(0.02f));
    controller.set_filter_hz(AC_Geometric_Attitude_Filter_Hz {});

    AC_Geometric_State state {};
    AC_Geometric_Target target {};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);
    target.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);
    state.omega_body_rads.zero();

    controller.update(state, target, 0.01f, output);   // seeds the state
    // Step the reference rate hard. dot(Omega_ref) stays zero, so the lead term
    // must stay zero; differencing e_Omega would instead produce -50 rad/s^2.
    target.omega_body_rads = Vector3f{0.5f, 0.0f, 0.0f};
    controller.update(state, target, 0.01f, output);

    EXPECT_FLOAT_EQ(0.0f, output.omega_error_derivative_radss.x);
    // The rate term still sees the step: e_Omega = Omega - Omega_ref = -0.5.
    EXPECT_FLOAT_EQ(-0.5f, output.omega_error_rads.x);
}

TEST(AC_Geometric_Attitude_PID, LeadTermFollowsAnalyticReferenceAcceleration)
{
    // A commanded angular acceleration is a real input to the lead term and must
    // pass through with the opposite sign to a measured one.
    AC_Geometric_Attitude_PID controller;
    AC_Geometric_Attitude_Output output {};
    set_unit_inertia(controller);
    controller.set_gains(lead_gains(0.02f));
    controller.set_filter_hz(AC_Geometric_Attitude_Filter_Hz {});

    AC_Geometric_State state {};
    AC_Geometric_Target target {};
    state.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);
    target.attitude_body_to_ned = attitude_from_euler(0.0f, 0.0f, 0.0f);

    controller.update(state, target, 0.01f, output);
    target.omega_dot_body_radss = Vector3f{2.0f, 0.0f, 0.0f};
    controller.update(state, target, 0.01f, output);

    EXPECT_FLOAT_EQ(-2.0f, output.omega_error_derivative_radss.x);
}

AP_GTEST_MAIN()
