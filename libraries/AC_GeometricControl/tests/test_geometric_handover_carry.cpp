#include <AP_gtest.h>

#include <AC_GeometricControl/AC_Geometric_HandoverCarry.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

constexpr float DT = 0.0025f;          // 400 Hz main loop
constexpr float TOLERANCE = 1.0e-6f;

AC_Geometric_HandoverCarry_Config config_with(float time_constant_s)
{
    return AC_Geometric_HandoverCarry_Config { time_constant_s };
}

// What the two loops were commanding either side of a logged handover, in
// normalized actuator units. Asymmetric and non-zero so a sign error shows.
const Vector3f NATIVE { -0.0222f, 0.0096f, 0.0f };
const Vector3f GEOMETRIC { 0.0457f, 0.0545f, 0.0f };

}  // namespace

TEST(HandoverCarry, DisabledLeavesTheCommandAlone)
{
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(0.0f));

    EXPECT_TRUE(carry.seed(NATIVE, GEOMETRIC).is_zero());
    EXPECT_FALSE(carry.active());
    EXPECT_TRUE(carry.update(DT).is_zero());
}

TEST(HandoverCarry, NegativeTimeConstantIsTreatedAsDisabled)
{
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(-1.0f));

    EXPECT_TRUE(carry.seed(NATIVE, GEOMETRIC).is_zero());
}

TEST(HandoverCarry, SeedIsTheCommandStepItCancels)
{
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(1.0f));

    const Vector3f& offset = carry.seed(NATIVE, GEOMETRIC);
    // Adding the offset to what the geometric controller computed reproduces
    // what the motors were already getting, so the command does not step.
    EXPECT_NEAR(NATIVE.x, GEOMETRIC.x + offset.x, TOLERANCE);
    EXPECT_NEAR(NATIVE.y, GEOMETRIC.y + offset.y, TOLERANCE);
    EXPECT_NEAR(NATIVE.z, GEOMETRIC.z + offset.z, TOLERANCE);
    EXPECT_TRUE(carry.active());
}

TEST(HandoverCarry, AgreeingControllersSeedNothing)
{
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(1.0f));

    EXPECT_TRUE(carry.seed(NATIVE, NATIVE).is_zero());
    EXPECT_FALSE(carry.active());
}

TEST(HandoverCarry, SaturatedGeometricCommandIsBoundedToTheCommandRange)
{
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(1.0f));

    // A law demanding three times full scale. Unbounded this seeds -2.96 and
    // the washout holds the command down for seconds while the controller is
    // saturated, which hides the saturation instead of smoothing a handover.
    const Vector3f saturated { 3.0f, -3.0f, 0.0f };
    const Vector3f& offset = carry.seed(NATIVE, saturated);

    EXPECT_NEAR(NATIVE.x - 1.0f, offset.x, TOLERANCE);
    EXPECT_NEAR(NATIVE.y + 1.0f, offset.y, TOLERANCE);
}

TEST(HandoverCarry, OutOfRangeNativeIsAlsoBounded)
{
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(1.0f));

    // AP_Motors documents set_roll as -1..+1 but does not enforce it.
    const Vector3f out_of_range { 4.0f, -4.0f, 0.0f };
    const Vector3f& offset = carry.seed(out_of_range, GEOMETRIC);

    EXPECT_NEAR(1.0f - GEOMETRIC.x, offset.x, TOLERANCE);
    EXPECT_NEAR(-1.0f - GEOMETRIC.y, offset.y, TOLERANCE);
}

TEST(HandoverCarry, OffsetNeverExceedsTwiceTheCommandRange)
{
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(1.0f));

    const Vector3f& offset = carry.seed(Vector3f{100.0f, -100.0f, 50.0f},
                                        Vector3f{-100.0f, 100.0f, -50.0f});
    EXPECT_NEAR(2.0f, offset.x, TOLERANCE);
    EXPECT_NEAR(-2.0f, offset.y, TOLERANCE);
    EXPECT_NEAR(2.0f, offset.z, TOLERANCE);
}

TEST(HandoverCarry, OneTimeConstantLeavesAboutAThird)
{
    AC_Geometric_HandoverCarry carry;
    const float tc = 0.5f;
    carry.set_config(config_with(tc));
    const Vector3f seeded = carry.seed(NATIVE, GEOMETRIC);

    float elapsed = 0.0f;
    while (elapsed < tc) {
        carry.update(DT);
        elapsed += DT;
    }
    // exp(-1) of what it started at, within the step quantisation.
    EXPECT_NEAR(seeded.x * expf(-1.0f), carry.offset().x, 1.0e-3f);
    EXPECT_NEAR(seeded.y * expf(-1.0f), carry.offset().y, 1.0e-3f);
}

TEST(HandoverCarry, AShortConstantIsGoneWithinATenthOfASecond)
{
    // The guidance is 0.1 to 0.2 s, because what the washout decays toward --
    // the persistent difference between the two controllers -- is already near
    // zero, and a long constant holds stale fast content open loop instead.
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(0.02f));
    const float seeded = carry.seed(NATIVE, GEOMETRIC).length();

    for (int i = 0; i < 40; i++) {         // 0.1 s at 400 Hz
        carry.update(DT);
    }
    EXPECT_LT(carry.offset().length(), 0.01f * seeded);
}

TEST(HandoverCarry, DecaysToNothingAndStaysThere)
{
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(0.2f));
    carry.seed(NATIVE, GEOMETRIC);

    for (int i = 0; i < 4000; i++) {       // 10 s at 400 Hz
        carry.update(DT);
    }
    EXPECT_TRUE(carry.offset().is_zero());
    EXPECT_FALSE(carry.active());
}

TEST(HandoverCarry, DecayIsMonotonic)
{
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(1.0f));
    float previous = carry.seed(NATIVE, GEOMETRIC).length();

    for (int i = 0; i < 200; i++) {
        const float now = carry.update(DT).length();
        EXPECT_LE(now, previous);
        previous = now;
    }
}

TEST(HandoverCarry, NonFiniteInputsSeedNothing)
{
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(1.0f));

    const Vector3f nan_input { NAN, 0.0f, 0.0f };
    const Vector3f inf_input { INFINITY, 0.0f, 0.0f };
    EXPECT_TRUE(carry.seed(nan_input, GEOMETRIC).is_zero());
    EXPECT_TRUE(carry.seed(NATIVE, nan_input).is_zero());
    EXPECT_TRUE(carry.seed(inf_input, GEOMETRIC).is_zero());
    EXPECT_TRUE(carry.seed(NATIVE, inf_input).is_zero());
}

TEST(HandoverCarry, BadTimestepClearsRatherThanHolding)
{
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(1.0f));

    carry.seed(NATIVE, GEOMETRIC);
    EXPECT_TRUE(carry.update(0.0f).is_zero());

    carry.seed(NATIVE, GEOMETRIC);
    EXPECT_TRUE(carry.update(-DT).is_zero());

    carry.seed(NATIVE, GEOMETRIC);
    EXPECT_TRUE(carry.update(NAN).is_zero());
}

TEST(HandoverCarry, ResetClears)
{
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(1.0f));

    carry.seed(NATIVE, GEOMETRIC);
    EXPECT_TRUE(carry.active());
    carry.reset();
    EXPECT_FALSE(carry.active());
    EXPECT_TRUE(carry.offset().is_zero());
}

TEST(HandoverCarry, DisablingMidDecayStopsTheCarry)
{
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(1.0f));
    carry.seed(NATIVE, GEOMETRIC);
    carry.update(DT);
    EXPECT_TRUE(carry.active());

    carry.set_config(config_with(0.0f));
    EXPECT_TRUE(carry.update(DT).is_zero());
}

AP_GTEST_MAIN()
