#include <AP_gtest.h>

#include <AC_GeometricControl/AC_Geometric_L1_Observer.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

constexpr float DT = 0.0025f;          // 400 Hz main loop
constexpr float TOLERANCE = 1.0e-6f;

AC_Geometric_L1_Observer_Config config_with(float predictor_gain,
                                            float filter_hz = 0.0f,
                                            float sigma_max = 0.0f)
{
    AC_Geometric_L1_Observer_Config config {};
    config.predictor_gain = predictor_gain;
    config.filter_hz = filter_hz;
    config.sigma_max = sigma_max;
    return config;
}

AC_Geometric_State state_with(const Vector3f& velocity_ned_ms,
                              float roll_rad = 0.0f,
                              float pitch_rad = 0.0f,
                              float yaw_rad = 0.0f)
{
    AC_Geometric_State state {};
    state.velocity_ned_ms = velocity_ned_ms;
    state.attitude_body_to_ned.from_euler(roll_rad, pitch_rad, yaw_rad);
    return state;
}

AC_Geometric_Position_Output position_with(float thrust)
{
    AC_Geometric_Position_Output position {};
    position.thrust = thrust;
    return position;
}

// Drive the observer against a plant whose true acceleration is the nominal
// one plus a constant the model does not know about.  Level attitude with
// thrust exactly cancelling gravity makes the nominal acceleration zero, so
// the disturbance is the whole of the true acceleration and the expected
// estimate is unambiguous.
Vector3f run_with_constant_disturbance(AC_Geometric_L1_Observer& observer,
                                       const Vector3f& disturbance_mss,
                                       uint16_t steps)
{
    Vector3f velocity_ned_ms;
    for (uint16_t i = 0; i < steps; i++) {
        observer.update(state_with(velocity_ned_ms), position_with(GRAVITY_MSS), DT);
        velocity_ned_ms += disturbance_mss * DT;
    }
    return observer.get_output().sigma_filtered_ned_mss;
}

} // namespace

TEST(GeometricL1Observer, FirstSampleCannotReportAnError)
{
    AC_Geometric_L1_Observer observer;
    observer.set_config(config_with(5.0f));

    // A resumed observer seeds from the measurement, so however fast the
    // vehicle is already moving the first prediction error must be zero.
    observer.update(state_with(Vector3f{7.0f, -3.0f, 2.0f}), position_with(GRAVITY_MSS), DT);

    EXPECT_NEAR(observer.get_output().velocity_error_ned_ms.length(), 0.0f, TOLERANCE);
    EXPECT_NEAR(observer.get_output().sigma_hat_ned_mss.length(), 0.0f, TOLERANCE);
}

TEST(GeometricL1Observer, UndisturbedHoverEstimatesNothing)
{
    AC_Geometric_L1_Observer observer;
    observer.set_config(config_with(5.0f));

    const Vector3f sigma = run_with_constant_disturbance(observer, Vector3f{}, 400);

    EXPECT_NEAR(sigma.length(), 0.0f, 1.0e-4f);
    EXPECT_TRUE(observer.get_output().valid);
}

TEST(GeometricL1Observer, ConstantUnmodelledAccelerationIsRecovered)
{
    AC_Geometric_L1_Observer observer;
    observer.set_config(config_with(5.0f, 5.0f));

    const Vector3f disturbance{0.8f, -0.4f, 0.25f};
    const Vector3f sigma = run_with_constant_disturbance(observer, disturbance, 4000);

    EXPECT_NEAR(sigma.x, disturbance.x, 0.02f);
    EXPECT_NEAR(sigma.y, disturbance.y, 0.02f);
    EXPECT_NEAR(sigma.z, disturbance.z, 0.02f);
}

TEST(GeometricL1Observer, EstimateIsHeldInNedNotBody)
{
    // The same NED disturbance seen from two different headings must produce
    // the same estimate; if the estimate were body-referenced, yawing would
    // rotate it.
    const Vector3f disturbance{0.6f, 0.0f, 0.0f};

    AC_Geometric_L1_Observer level;
    level.set_config(config_with(5.0f, 5.0f));
    Vector3f velocity_ned_ms;
    for (uint16_t i = 0; i < 4000; i++) {
        level.update(state_with(velocity_ned_ms, 0.0f, 0.0f, 0.0f),
                     position_with(GRAVITY_MSS), DT);
        velocity_ned_ms += disturbance * DT;
    }

    AC_Geometric_L1_Observer yawed;
    yawed.set_config(config_with(5.0f, 5.0f));
    velocity_ned_ms.zero();
    for (uint16_t i = 0; i < 4000; i++) {
        yawed.update(state_with(velocity_ned_ms, 0.0f, 0.0f, radians(90.0f)),
                     position_with(GRAVITY_MSS), DT);
        velocity_ned_ms += disturbance * DT;
    }

    // Guard against the comparison passing because both estimates are zero.
    EXPECT_GT(level.get_output().sigma_filtered_ned_mss.x, 0.5f);
    EXPECT_NEAR(level.get_output().sigma_filtered_ned_mss.x,
                yawed.get_output().sigma_filtered_ned_mss.x, 1.0e-3f);
    EXPECT_NEAR(yawed.get_output().sigma_filtered_ned_mss.y, 0.0f, 1.0e-2f);
}

TEST(GeometricL1Observer, TiltedThrustStillCancelsGravity)
{
    // Convention-independent check on the nominal model: holding altitude at a
    // tilt needs thrust g/cos(tilt), and that must leave no vertical nominal
    // acceleration regardless of how the rotation is composed.
    AC_Geometric_L1_Observer observer;
    observer.set_config(config_with(5.0f));

    for (const float tilt_deg : {0.0f, 10.0f, 25.0f}) {
        const float tilt = radians(tilt_deg);
        observer.reset();
        observer.update(state_with(Vector3f{}, 0.0f, tilt, radians(30.0f)),
                        position_with(GRAVITY_MSS / cosf(tilt)), DT);
        EXPECT_NEAR(observer.get_output().nominal_accel_ned_mss.z, 0.0f, 1.0e-4f)
            << "tilt " << tilt_deg << " deg";
    }
}

TEST(GeometricL1Observer, NonFiniteInputInvalidatesAndReseeds)
{
    AC_Geometric_L1_Observer observer;
    observer.set_config(config_with(5.0f));
    run_with_constant_disturbance(observer, Vector3f{0.5f, 0.0f, 0.0f}, 200);
    const uint32_t resets_before = observer.get_output().resets;

    AC_Geometric_State bad = state_with(Vector3f{});
    bad.velocity_ned_ms.x = nanf("");
    observer.update(bad, position_with(GRAVITY_MSS), DT);

    EXPECT_FALSE(observer.get_output().valid);
    EXPECT_GT(observer.get_output().resets, resets_before);

    // Recovering must start from the measurement, not from the stale predictor.
    observer.update(state_with(Vector3f{4.0f, 0.0f, 0.0f}), position_with(GRAVITY_MSS), DT);
    EXPECT_NEAR(observer.get_output().velocity_error_ned_ms.length(), 0.0f, TOLERANCE);
}

TEST(GeometricL1Observer, NonPositiveTimestepIsRejected)
{
    AC_Geometric_L1_Observer observer;
    observer.set_config(config_with(5.0f));

    observer.update(state_with(Vector3f{}), position_with(GRAVITY_MSS), 0.0f);
    EXPECT_FALSE(observer.get_output().valid);

    observer.update(state_with(Vector3f{}), position_with(GRAVITY_MSS), -DT);
    EXPECT_FALSE(observer.get_output().valid);
}

TEST(GeometricL1Observer, ZeroPredictorGainIsInert)
{
    // The documented disable path. A zero gain must not divide by zero and
    // must not produce an estimate.
    AC_Geometric_L1_Observer observer;
    observer.set_config(config_with(0.0f));

    for (uint16_t i = 0; i < 100; i++) {
        observer.update(state_with(Vector3f{float(i) * 0.01f, 0.0f, 0.0f}),
                        position_with(GRAVITY_MSS), DT);
    }

    EXPECT_FALSE(observer.get_output().valid);
    EXPECT_NEAR(observer.get_output().sigma_hat_ned_mss.length(), 0.0f, TOLERANCE);
}

TEST(GeometricL1Observer, EstimateLimitClampsAndReportsSaturation)
{
    AC_Geometric_L1_Observer observer;
    observer.set_config(config_with(5.0f, 0.0f, 0.1f));

    run_with_constant_disturbance(observer, Vector3f{5.0f, 0.0f, 0.0f}, 200);

    EXPECT_LE(observer.get_output().sigma_hat_ned_mss.length(), 0.1f + TOLERANCE);
    EXPECT_TRUE(observer.get_output().saturated);
}

TEST(GeometricL1Observer, FilterAttenuatesWithoutChangingTheRawEstimate)
{
    AC_Geometric_L1_Observer observer;
    observer.set_config(config_with(5.0f, 0.5f));

    // One step of a disturbance that the raw law reacts to immediately; a
    // half-hertz filter cannot have followed it yet.
    observer.update(state_with(Vector3f{}), position_with(GRAVITY_MSS), DT);
    observer.update(state_with(Vector3f{0.0f, 0.0f, 0.0f}), position_with(GRAVITY_MSS), DT);
    for (uint16_t i = 0; i < 40; i++) {
        observer.update(state_with(Vector3f{float(i + 1) * 0.05f, 0.0f, 0.0f}),
                        position_with(GRAVITY_MSS), DT);
    }

    const AC_Geometric_L1_Observer_Output& out = observer.get_output();
    // A zero filtered value would satisfy the inequality without the filter
    // doing anything, so require both signals to be live.
    EXPECT_GT(fabsf(out.sigma_hat_ned_mss.x), 0.0f);
    EXPECT_GT(fabsf(out.sigma_filtered_ned_mss.x), 0.0f);
    EXPECT_LT(fabsf(out.sigma_filtered_ned_mss.x), fabsf(out.sigma_hat_ned_mss.x));
}

TEST(GeometricL1Observer, ResetClearsTheEstimateButKeepsTheResetCount)
{
    AC_Geometric_L1_Observer observer;
    observer.set_config(config_with(5.0f, 5.0f));
    run_with_constant_disturbance(observer, Vector3f{1.0f, 0.0f, 0.0f}, 2000);
    EXPECT_GT(observer.get_output().sigma_filtered_ned_mss.length(), 0.1f);

    observer.update(state_with(Vector3f{}), position_with(GRAVITY_MSS), -1.0f);
    const uint32_t resets = observer.get_output().resets;
    EXPECT_GT(resets, 0u);

    observer.reset();
    EXPECT_NEAR(observer.get_output().sigma_filtered_ned_mss.length(), 0.0f, TOLERANCE);
    EXPECT_EQ(observer.get_output().resets, resets);
}

AP_GTEST_MAIN()
