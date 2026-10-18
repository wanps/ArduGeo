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

TEST(HandoverCarry, ASaturatedLawIsRefusedRatherThanSmoothed)
{
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(0.1f));

    // A law demanding three times full scale. Reproducing the last native
    // command would hold the output near what native wanted while this law is
    // asking for more than full authority, which is the wrong way to be wrong.
    EXPECT_TRUE(carry.seed(NATIVE, Vector3f{3.0f, -3.0f, 0.0f}).is_zero());
    EXPECT_FALSE(carry.active());
}

TEST(HandoverCarry, OneSaturatedAxisRefusesTheWholeSeed)
{
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(0.1f));

    // Conservative on purpose. A law at the rail on one axis is not in a state
    // where smoothing the others is obviously right.
    EXPECT_TRUE(carry.seed(NATIVE, Vector3f{0.2f, 1.5f, 0.1f}).is_zero());
    EXPECT_TRUE(carry.seed(NATIVE, Vector3f{0.2f, 0.1f, -1.01f}).is_zero());
}

TEST(HandoverCarry, ACommandExactlyAtTheRailIsStillSeeded)
{
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(0.1f));

    // The refusal is for commands the mapper had to limit, not for one that
    // happens to land on the boundary.
    EXPECT_FALSE(carry.seed(NATIVE, Vector3f{1.0f, -1.0f, 0.0f}).is_zero());
}

TEST(HandoverCarry, OutOfRangeNativeIsClampedBeforeSubtracting)
{
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(0.1f));

    // AP_Motors documents set_roll as -1..+1 but does not enforce it. The
    // difference against an in-range law is then 0.95 and -1.05, both past
    // the offset bound.
    const Vector3f& offset = carry.seed(Vector3f{4.0f, -4.0f, 0.0f}, GEOMETRIC);

    EXPECT_NEAR(0.5f, offset.x, TOLERANCE);
    EXPECT_NEAR(-0.5f, offset.y, TOLERANCE);
}

TEST(HandoverCarry, TheOffsetIsBoundedToTheScaleOfTheJump)
{
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(0.1f));

    // The offset is applied to later frames' commands too, so it is bounded by
    // the size of the thing being smoothed -- 0.5, against a worst measured
    // jump of 0.208 -- not by what the command range would permit.
    const Vector3f& offset = carry.seed(Vector3f{1.0f, -1.0f, 1.0f},
                                        Vector3f{-0.9f, 0.9f, -0.2f});
    EXPECT_NEAR(0.5f, offset.x, TOLERANCE);
    EXPECT_NEAR(-0.5f, offset.y, TOLERANCE);
    EXPECT_NEAR(0.5f, offset.z, TOLERANCE);
}

TEST(HandoverCarry, EachAxisIsBoundedIndependently)
{
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(0.1f));

    const Vector3f& offset = carry.seed(Vector3f{1.0f, 0.1f, -1.0f},
                                        Vector3f{-0.9f, 0.0f, -0.8f});
    EXPECT_NEAR(0.5f, offset.x, TOLERANCE);      // bound binds
    EXPECT_NEAR(0.1f, offset.y, TOLERANCE);      // does not
    EXPECT_NEAR(-0.2f, offset.z, TOLERANCE);     // does not
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

TEST(HandoverCarry, TheRecommendedConstantIsSpentWithinAThirdOfASecond)
{
    // 0.1 s is the suggested starting value, chosen against a measured
    // transient envelope of 50 to 100 ms. Pin what it actually does rather
    // than a constant nobody is told to use: a third of the seed is still
    // applied at 0.1 s, and the correction is spent by 0.3 s.
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(0.1f));
    const float seeded = carry.seed(NATIVE, GEOMETRIC).length();

    for (int i = 0; i < 40; i++) {         // 0.1 s at 400 Hz
        carry.update(DT);
    }
    EXPECT_NEAR(seeded * expf(-1.0f), carry.offset().length(), 0.1f * seeded);

    for (int i = 0; i < 80; i++) {         // to 0.3 s
        carry.update(DT);
    }
    EXPECT_LT(carry.offset().length(), 0.06f * seeded);
}

// ---------------------------------------------------------------------------
// apply(): turning an offset into the command the motors receive. Both
// defects found in review lived here, in code no test reached.

TEST(HandoverCarryApply, TheFirstFrameReproducesTheNativeCommand)
{
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(0.1f));
    const Vector3f raw { 0.30f, -0.20f, 0.05f };
    const Vector3f offset = carry.seed(NATIVE, raw);

    const AC_Geometric_HandoverCarry::Applied applied =
        AC_Geometric_HandoverCarry::apply(raw, offset, false);

    EXPECT_NEAR(NATIVE.x, applied.rpy_norm.x, TOLERANCE);
    EXPECT_NEAR(NATIVE.y, applied.rpy_norm.y, TOLERANCE);
    EXPECT_NEAR(NATIVE.z, applied.rpy_norm.z, TOLERANCE);
    EXPECT_FALSE(applied.limited);
}

TEST(HandoverCarryApply, ASaturatedLawGetsNoCarryAndKeepsItsAuthority)
{
    // The case that broke in review: the offset was computed against the
    // limited command and added to the unlimited one, so a law demanding 1.5
    // against a native -0.5 produced 0.0 -- neither continuous nor the law's
    // intent -- and reported the rail as clean. It is now refused outright,
    // so the law keeps the authority it is asking for.
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(0.1f));
    const Vector3f raw { 1.5f, 0.0f, 0.0f };
    const Vector3f offset = carry.seed(Vector3f{-0.5f, 0.0f, 0.0f}, raw);
    EXPECT_TRUE(offset.is_zero());

    const AC_Geometric_HandoverCarry::Applied applied =
        AC_Geometric_HandoverCarry::apply(raw, offset, true);
    EXPECT_NEAR(1.0f, applied.rpy_norm.x, TOLERANCE);
    EXPECT_TRUE(applied.limited);
}

TEST(HandoverCarryApply, ADecayFrameCannotBePushedBeyondTheRail)
{
    // The offset was sized against the previous frame's command and is applied
    // to this one. Whatever that mis-sizing does, the result is a realizable
    // command and the rail is reported.
    const AC_Geometric_HandoverCarry::Applied applied =
        AC_Geometric_HandoverCarry::apply(Vector3f{0.8f, -0.8f, 0.0f},
                                          Vector3f{0.5f, -0.5f, 0.0f}, false);
    EXPECT_NEAR(1.0f, applied.rpy_norm.x, TOLERANCE);
    EXPECT_NEAR(-1.0f, applied.rpy_norm.y, TOLERANCE);
    EXPECT_TRUE(applied.limited);
}

TEST(HandoverCarryApply, TheMappersLimitFlagSurvives)
{
    // The law asked for more than the mapper could give. That stays true even
    // when the carry brings the command back inside the range.
    const AC_Geometric_HandoverCarry::Applied applied =
        AC_Geometric_HandoverCarry::apply(Vector3f{2.0f, 0.0f, 0.0f},
                                          Vector3f{-1.5f, 0.0f, 0.0f}, true);
    EXPECT_NEAR(0.5f, applied.rpy_norm.x, TOLERANCE);
    EXPECT_TRUE(applied.limited);
}

TEST(HandoverCarryApply, ACarryThatPushesPastTheRailIsReported)
{
    // The mapper was happy; the carry is what reaches the limit. The gate that
    // watches for a handover made worse has to see this.
    const AC_Geometric_HandoverCarry::Applied applied =
        AC_Geometric_HandoverCarry::apply(Vector3f{0.9f, 0.0f, 0.0f},
                                          Vector3f{0.4f, 0.0f, 0.0f}, false);
    EXPECT_NEAR(1.0f, applied.rpy_norm.x, TOLERANCE);
    EXPECT_TRUE(applied.limited);
}

TEST(HandoverCarryApply, AnInRangeCarryReportsNothing)
{
    const AC_Geometric_HandoverCarry::Applied applied =
        AC_Geometric_HandoverCarry::apply(Vector3f{0.2f, -0.3f, 0.1f},
                                          Vector3f{0.1f, 0.1f, -0.05f}, false);
    EXPECT_NEAR(0.3f, applied.rpy_norm.x, TOLERANCE);
    EXPECT_NEAR(-0.2f, applied.rpy_norm.y, TOLERANCE);
    EXPECT_NEAR(0.05f, applied.rpy_norm.z, TOLERANCE);
    EXPECT_FALSE(applied.limited);
}

TEST(HandoverCarryApply, EveryAxisIsCheckedForTheRail)
{
    for (int axis = 0; axis < 3; axis++) {
        Vector3f raw, offset;
        raw[axis] = 0.9f;
        offset[axis] = 0.5f;
        const AC_Geometric_HandoverCarry::Applied applied =
            AC_Geometric_HandoverCarry::apply(raw, offset, false);
        EXPECT_TRUE(applied.limited) << "轴 " << axis;
        EXPECT_NEAR(1.0f, applied.rpy_norm[axis], TOLERANCE) << "轴 " << axis;
    }
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

// ---------------------------------------------------------------------------
// The parameter clamp. Separated from config() because the parameter itself
// cannot be set from a unit test without the full AP_Param machinery, and this
// is the half that has the failure modes.

TEST(HandoverCarryParams, AValueInRangePassesThrough)
{
    EXPECT_FLOAT_EQ(0.1f, AC_Geometric_HandoverCarry_Params::clamped_time_constant(0.1f));
    EXPECT_FLOAT_EQ(0.5f, AC_Geometric_HandoverCarry_Params::clamped_time_constant(0.5f));
}

TEST(HandoverCarryParams, AnOverlongConstantIsClamped)
{
    // The declared range is metadata the ground station honours and nothing
    // else did. A 30 s constant would hold an open-loop offset for half a
    // minute.
    EXPECT_FLOAT_EQ(0.5f, AC_Geometric_HandoverCarry_Params::clamped_time_constant(30.0f));
    EXPECT_FLOAT_EQ(0.5f, AC_Geometric_HandoverCarry_Params::clamped_time_constant(2.0f));
}

TEST(HandoverCarryParams, NegativeStaysNegativeInEffect)
{
    EXPECT_FLOAT_EQ(0.0f, AC_Geometric_HandoverCarry_Params::clamped_time_constant(-1.0f));
}

TEST(HandoverCarryParams, NonFiniteDisablesRatherThanEnabling)
{
    // constrain_float returns the midpoint of its range for NaN, so clamping
    // alone would turn a corrupt parameter into an enabled carry at 0.25 s.
    EXPECT_FLOAT_EQ(0.0f, AC_Geometric_HandoverCarry_Params::clamped_time_constant(NAN));
    EXPECT_FLOAT_EQ(0.0f, AC_Geometric_HandoverCarry_Params::clamped_time_constant(INFINITY));
    EXPECT_FLOAT_EQ(0.0f, AC_Geometric_HandoverCarry_Params::clamped_time_constant(-INFINITY));
}

AP_GTEST_MAIN()
