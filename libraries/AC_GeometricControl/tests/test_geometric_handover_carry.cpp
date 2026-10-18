#include <AP_gtest.h>

#include <AC_GeometricControl/AC_Geometric_HandoverCarry.h>
#include <AC_GeometricControl/AC_GeometricControl.h>
#include <AP_Param/AP_Param.h>

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
    carry.set_config(config_with(0.5f));

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
    carry.set_config(config_with(0.5f));

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

TEST(HandoverCarry, ACommandExactlyAtTheRailIsNotTreatedAsSaturated)
{
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(0.1f));

    // The refusal is for commands the mapper had to limit, not for one that
    // happens to land on the boundary. Native is near the same rail so the
    // difference stays inside MAX_OFFSET and the other refusal cannot fire.
    const Vector3f& offset = carry.seed(Vector3f{0.9f, -0.9f, 0.0f},
                                        Vector3f{1.0f, -1.0f, 0.0f});
    EXPECT_NEAR(-0.1f, offset.x, TOLERANCE);
    EXPECT_NEAR(0.1f, offset.y, TOLERANCE);
}

TEST(HandoverCarry, OneUlpPastTheRailIsSaturated)
{
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(0.1f));

    // FLT_EPSILON is one ulp at 1.0, so there is no gap between "at the rail"
    // and "past it".
    EXPECT_TRUE(carry.seed(Vector3f{0.9f, 0.0f, 0.0f},
                           Vector3f{nextafterf(1.0f, 2.0f), 0.0f, 0.0f}).is_zero());
}

TEST(HandoverCarry, OutOfRangeNativeIsClampedBeforeSubtracting)
{
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(0.1f));

    // AP_Motors documents set_roll as -1..+1 but does not enforce it. Clamped
    // to 1.0 the difference against a law at 0.6 is 0.4, inside the bound;
    // unclamped it would be 3.4 and the seed would be refused instead.
    const Vector3f& offset = carry.seed(Vector3f{4.0f, -4.0f, 0.0f},
                                        Vector3f{0.6f, -0.6f, 0.0f});

    EXPECT_NEAR(0.4f, offset.x, TOLERANCE);
    EXPECT_NEAR(-0.4f, offset.y, TOLERANCE);
}

TEST(HandoverCarry, ADisagreementLargerThanTheBoundIsRefusedNotTruncated)
{
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(0.1f));

    // Truncating would take both halves of the bargain and deliver neither:
    // the part of the step above the bound survives uncancelled, and 0.5 of
    // command the law never asked for is still held through the decay.
    EXPECT_TRUE(carry.seed(Vector3f{1.0f, -1.0f, 1.0f},
                           Vector3f{-0.9f, 0.9f, -0.2f}).is_zero());
    EXPECT_FALSE(carry.active());
}

TEST(HandoverCarry, OneOversizedAxisRefusesTheWholeSeed)
{
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(0.1f));

    // Roll differs by 1.9, the others by 0.1 and 0.2. Refusing all three
    // falls back to the behaviour both logged flights flew.
    EXPECT_TRUE(carry.seed(Vector3f{1.0f, 0.1f, -1.0f},
                           Vector3f{-0.9f, 0.0f, -0.8f}).is_zero());
}

TEST(HandoverCarry, ADisagreementAtTheBoundIsStillSeeded)
{
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(0.1f));

    // The refusal is strictly greater than, so the bound itself is usable.
    const Vector3f& offset = carry.seed(Vector3f{0.5f, 0.0f, 0.0f}, Vector3f{});
    EXPECT_NEAR(0.5f, offset.x, TOLERANCE);
}

TEST(HandoverCarry, OneUlpPastTheBoundIsRefused)
{
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(0.1f));

    // Pinned at ulp resolution like the saturation rail, so the boundary
    // cannot drift without a test noticing.
    EXPECT_TRUE(carry.seed(Vector3f{nextafterf(0.5f, 1.0f), 0.0f, 0.0f},
                           Vector3f{}).is_zero());
    EXPECT_FALSE(carry.seed(Vector3f{nextafterf(0.5f, 0.0f), 0.0f, 0.0f},
                            Vector3f{}).is_zero());
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
    carry.set_config(config_with(0.5f));
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
    carry.set_config(config_with(0.5f));

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
    carry.set_config(config_with(0.5f));

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
    carry.set_config(config_with(0.5f));

    carry.seed(NATIVE, GEOMETRIC);
    EXPECT_TRUE(carry.active());
    carry.reset();
    EXPECT_FALSE(carry.active());
    EXPECT_TRUE(carry.offset().is_zero());
}

TEST(HandoverCarry, DisablingMidDecayStopsTheCarry)
{
    AC_Geometric_HandoverCarry carry;
    carry.set_config(config_with(0.5f));
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

// ---------------------------------------------------------------------------
// The wiring. Which field is handed to seed() and which field is written back
// is what broke twice, and the class tests cannot see it.

namespace {

// Sets GEO_HAND_TC through the real parameter path. The sub-group is declared
// with an empty name (AP_SUBGROUPINFO(..., "", 18, ...)), so it is located by
// the table it points at rather than by name.
bool set_handover_tc(AC_GeometricControl& control, float value)
{
    for (const AP_Param::GroupInfo* gi = AC_GeometricControl::var_info;
         gi->type != AP_PARAM_NONE; gi++) {
        if (gi->type == AP_PARAM_GROUP &&
            gi->group_info == AC_Geometric_HandoverCarry_Params::var_info) {
            return AP_Param::set_object_value(
                reinterpret_cast<const uint8_t*>(&control) + gi->offset,
                gi->group_info, "HAND_TC", value);
        }
    }
    return false;
}

}  // namespace

TEST(HandoverCarryWiring, TheCommandMovesAndTheLawsIntentDoesNot)
{
    AC_GeometricControl control;
    ASSERT_TRUE(set_handover_tc(control, 0.1f));

    // rpy_norm_raw starts at zero, so the offset is the native command itself.
    const Vector3f native { 0.10f, -0.20f, 0.30f };
    control.begin_output_ownership(native);

    const AC_Geometric_Mapped_Output& mapped = control.get_output().mapped;
    EXPECT_NEAR(native.x, mapped.rpy_norm.x, TOLERANCE);
    EXPECT_NEAR(native.y, mapped.rpy_norm.y, TOLERANCE);
    EXPECT_NEAR(native.z, mapped.rpy_norm.z, TOLERANCE);
    // The law asked for nothing, and still says so. Writing the carry here is
    // what made a saturated law look unsaturated to the output-rail gate.
    EXPECT_TRUE(mapped.rpy_norm_raw.is_zero());
}

TEST(HandoverCarryWiring, TheConfigIsReadBeforeTheFirstSeed)
{
    // Without set_config() in begin_output_ownership the carry would use a
    // default-constructed config on the first engage after boot and silently
    // never fire.
    AC_GeometricControl control;
    ASSERT_TRUE(set_handover_tc(control, 0.1f));
    control.begin_output_ownership(Vector3f{0.10f, -0.20f, 0.30f});
    EXPECT_FALSE(control.handover_carry_offset().is_zero());
}

TEST(HandoverCarryWiring, ASecondEngageSeedsFromTheLawsIntentNotTheCarriedCommand)
{
    // The two fields are equal on a first engage, so every other test here is
    // blind to swapping them -- which is the round-3 defect, both halves of
    // it. After one engage they differ: rpy_norm carries the first offset
    // while rpy_norm_raw is still the law's intent, which is what the second
    // seed must be measured against.
    AC_GeometricControl control;
    ASSERT_TRUE(set_handover_tc(control, 0.1f));

    control.begin_output_ownership(Vector3f{0.10f, 0.0f, 0.0f});
    control.end_output_ownership();
    control.begin_output_ownership(Vector3f{0.30f, 0.0f, 0.0f});

    // Seeding from rpy_norm instead would give 0.20; applying to rpy_norm
    // instead would give a command of 0.40.
    EXPECT_NEAR(0.30f, control.handover_carry_offset().x, TOLERANCE);
    EXPECT_NEAR(0.30f, control.get_output().mapped.rpy_norm.x, TOLERANCE);
    EXPECT_TRUE(control.get_output().mapped.rpy_norm_raw.is_zero());
}

TEST(HandoverCarryWiring, EndingOwnershipClearsTheOffset)
{
    AC_GeometricControl control;
    ASSERT_TRUE(set_handover_tc(control, 0.1f));
    control.begin_output_ownership(Vector3f{0.10f, -0.20f, 0.30f});
    ASSERT_FALSE(control.handover_carry_offset().is_zero());

    control.end_output_ownership();
    EXPECT_TRUE(control.handover_carry_offset().is_zero());
}

TEST(HandoverCarryWiring, ResetAlsoClearsTheOffset)
{
    // The other door into the same invariant. set_enabled(false) arrives here.
    AC_GeometricControl control;
    ASSERT_TRUE(set_handover_tc(control, 0.1f));
    control.begin_output_ownership(Vector3f{0.10f, -0.20f, 0.30f});
    ASSERT_FALSE(control.handover_carry_offset().is_zero());

    control.reset();
    EXPECT_TRUE(control.handover_carry_offset().is_zero());
}

TEST(HandoverCarryWiring, AnOverlongParameterIsClampedThroughTheRealPath)
{
    AC_GeometricControl control;
    ASSERT_TRUE(set_handover_tc(control, 30.0f));
    EXPECT_FLOAT_EQ(0.5f, control.handover_carry_time_constant_s());
}

TEST(HandoverCarryWiring, TheDefaultLeavesTheCommandAlone)
{
    AC_GeometricControl control;
    control.begin_output_ownership(Vector3f{0.10f, -0.20f, 0.30f});

    const AC_Geometric_Mapped_Output& mapped = control.get_output().mapped;
    EXPECT_TRUE(mapped.rpy_norm.is_zero());
    EXPECT_TRUE(mapped.rpy_norm_raw.is_zero());
    EXPECT_FALSE(mapped.rpy_limited);
    EXPECT_TRUE(control.handover_carry_offset().is_zero());
}


// Driving the real cascade, because the rail flag the output-rail gate reads
// can only diverge from the mapper's own on a decay frame, and reaching one
// needs a command the mapper actually produced.
namespace {

// A roll attitude error of theta yields rpy_norm_raw.x = -sin(theta), which is
// how these tests place the command where they need it.
void step_with_roll_error(AC_GeometricControl& control, float roll_error_deg)
{
    AC_Geometric_State state {};
    state.attitude_body_to_ned.from_euler(radians(roll_error_deg), 0.0f, 0.0f);
    AC_Geometric_Target target {};
    target.attitude_body_to_ned.from_euler(0.0f, 0.0f, 0.0f);
    target.build_attitude_from_position = false;
    control.update(state, target, 0.0025f);
}

}  // namespace

TEST(HandoverCarryWiring, ACarryThatRailsADecayFrameIsReportedToTheGate)
{
    AC_GeometricControl control;
    control.set_enabled(true);
    control.set_hover_throttle_reference(0.2f);
    ASSERT_TRUE(set_handover_tc(control, 0.1f));

    // Engage against a law commanding nothing, so the offset is the native
    // command itself.
    step_with_roll_error(control, 0.0f);
    ASSERT_TRUE(control.get_output().mapped.rpy_norm_raw.is_zero());
    control.begin_output_ownership(Vector3f{-0.5f, 0.0f, 0.0f});
    ASSERT_NEAR(-0.5f, control.handover_carry_offset().x, TOLERANCE);

    // One frame later the law wants -0.707, which the mapper passes through
    // untouched. The carry still holds nearly -0.5 on top of it, and the sum
    // is past the rail.
    step_with_roll_error(control, 45.0f);

    const AC_Geometric_Mapped_Output& mapped = control.get_output().mapped;
    EXPECT_NEAR(-0.7071f, mapped.rpy_norm_raw.x, 1.0e-3f);   // the law's intent
    EXPECT_NEAR(-1.0f, mapped.rpy_norm.x, TOLERANCE);        // what is written
    // The mapper saw nothing out of range; only the carry reached the rail.
    // Without the write-back in apply_handover_carry this reads false and the
    // gate is blind to a handover that railed an axis.
    EXPECT_TRUE(mapped.rpy_limited);
}

TEST(HandoverCarryWiring, AnInRangeDecayFrameIsNotReportedAsRailed)
{
    AC_GeometricControl control;
    control.set_enabled(true);
    control.set_hover_throttle_reference(0.2f);
    ASSERT_TRUE(set_handover_tc(control, 0.1f));

    step_with_roll_error(control, 0.0f);
    control.begin_output_ownership(Vector3f{-0.2f, 0.0f, 0.0f});
    step_with_roll_error(control, 20.0f);                    // law wants -0.342

    const AC_Geometric_Mapped_Output& mapped = control.get_output().mapped;
    EXPECT_NEAR(-0.542f, mapped.rpy_norm.x, 1.0e-2f);
    EXPECT_FALSE(mapped.rpy_limited);
}

AP_GTEST_MAIN()
