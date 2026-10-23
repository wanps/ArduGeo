#include <AP_gtest.h>

#include <AC_GeometricControl/AC_Geometric_L1_Augment.h>
#include <AC_GeometricControl/AC_Geometric_Position_PID.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

constexpr float DT = 0.0025f;
const Vector3f SIGMA { 0.7f, -0.4f, -1.5f };   // horizontal parts must be ignored (S3a)

AC_Geometric_L1_Augment_Config enabled_config()
{
    AC_Geometric_L1_Augment_Config c {};
    c.enabled = true;
    c.ramp_in_s = 1.0f;
    c.ramp_out_s = 0.5f;
    c.pause_throttle = 0.12f;
    return c;
}

AC_Geometric_L1_Augment_Inputs healthy()
{
    AC_Geometric_L1_Augment_Inputs in {};
    in.observer_valid = true;
    in.observer_admissible = true;
    in.observer_resets = 3;
    in.sigma_filtered_ned_mss = SIGMA;
    in.owns_output = true;
    in.throttle = 0.30f;
    return in;
}

void run(AC_Geometric_L1_Augment& a, const AC_Geometric_L1_Augment_Inputs& in, float seconds)
{
    const int n = int(seconds / DT + 0.5f);
    for (int i = 0; i < n; i++) {
        a.update(in, DT);
    }
}

AC_Geometric_L1_Augment engaged()
{
    AC_Geometric_L1_Augment a;
    a.set_config(enabled_config());
    run(a, healthy(), 1.2f);
    return a;
}

}  // namespace

TEST(L1Augment, OffByDefaultAndInjectsNothing)
{
    AC_Geometric_L1_Augment a;          // default config: disabled
    run(a, healthy(), 3.0f);
    EXPECT_EQ(AC_Geometric_L1_State::OFF, a.state());
    EXPECT_TRUE(a.injection_ned_mss().is_zero());
}

TEST(L1Augment, NotAdmittedStaysOff)
{
    AC_Geometric_L1_Augment a;
    a.set_config(enabled_config());
    AC_Geometric_L1_Augment_Inputs in = healthy();
    in.observer_admissible = false;
    run(a, in, 3.0f);
    EXPECT_EQ(AC_Geometric_L1_State::OFF, a.state());
}

TEST(L1Augment, DoesNotEngageWithoutOwnership)
{
    AC_Geometric_L1_Augment a;
    a.set_config(enabled_config());
    AC_Geometric_L1_Augment_Inputs in = healthy();
    in.owns_output = false;
    run(a, in, 3.0f);
    EXPECT_EQ(AC_Geometric_L1_State::OFF, a.state());
}

TEST(L1Augment, RampsInOverTheConfiguredTimeVerticalOnly)
{
    AC_Geometric_L1_Augment a;
    a.set_config(enabled_config());
    run(a, healthy(), 0.5f);
    EXPECT_EQ(AC_Geometric_L1_State::RAMP_IN, a.state());
    EXPECT_NEAR(0.5f, a.lambda(), 0.01f);
    run(a, healthy(), 0.6f);
    EXPECT_EQ(AC_Geometric_L1_State::ENGAGED, a.state());
    const Vector3f inj = a.injection_ned_mss();
    // Cancels the estimate: the vertical estimate is -1.5, so +1.5 is added.
    EXPECT_FLOAT_EQ(-SIGMA.z, inj.z);
    // S3a: the horizontal estimate never enters.
    EXPECT_FLOAT_EQ(0.0f, inj.x);
    EXPECT_FLOAT_EQ(0.0f, inj.y);
}

TEST(L1Augment, AdmissionIsCheckedOnlyAtEngage)
{
    // 8.4: no continuous gate. Losing admissibility after engaging is not a
    // reason to leave; only a fault is.
    AC_Geometric_L1_Augment a = engaged();
    AC_Geometric_L1_Augment_Inputs in = healthy();
    in.observer_admissible = false;
    run(a, in, 2.0f);
    EXPECT_EQ(AC_Geometric_L1_State::ENGAGED, a.state());
}

TEST(L1Augment, CleanExitKeepsTheResultantContinuous)
{
    // On every frame of the ramp-out, what leaves the feed-forward must enter
    // the integrator. Track -lambda*sigma + sum(handover): it must stay at
    // the engaged value to rounding.
    AC_Geometric_L1_Augment a = engaged();
    const float before = a.injection_ned_mss().z;
    AC_Geometric_L1_Augment_Config off = enabled_config();
    off.enabled = false;
    a.set_config(off);
    float handed = 0.0f;
    float worst = 0.0f;
    for (int i = 0; i < 400; i++) {
        a.update(healthy(), DT);
        handed += a.handover_ned_mss().z;
        // the integrator's contribution to A is -handed (A has -K_I*e_I)
        const float resultant = a.injection_ned_mss().z - handed;
        worst = MAX(worst, fabsf(resultant - before));
    }
    EXPECT_EQ(AC_Geometric_L1_State::OFF, a.state());
    EXPECT_LT(worst, 1.0e-4f);
    EXPECT_NEAR(SIGMA.z, handed, 1.0e-4f);
    EXPECT_EQ(AC_Geometric_L1_Reason::DISABLED, a.reason());
}

TEST(L1Augment, OverrunLatchesAndDiscards)
{
    AC_Geometric_L1_Augment a = engaged();
    AC_Geometric_L1_Augment_Inputs in = healthy();
    in.observer_over_limit = true;
    in.sigma_filtered_ned_mss = Vector3f{0, 0, -7.0f};
    a.update(in, DT);
    EXPECT_EQ(AC_Geometric_L1_State::LATCHED, a.state());
    EXPECT_EQ(AC_Geometric_L1_Reason::FAULT_OVERRUN, a.reason());
    EXPECT_TRUE(a.handover_ned_mss().is_zero()) << "a faulted estimate is not handed over";
    // The ramp-out carries the last healthy value, not the overrun one.
    EXPECT_LE(fabsf(a.injection_ned_mss().z), fabsf(SIGMA.z) + 1e-6f);
    run(a, in, AC_Geometric_L1_Augment::FAULT_RAMP_S + 0.05f);
    EXPECT_TRUE(a.injection_ned_mss().is_zero());
}

TEST(L1Augment, ALatchHoldsForTheFlight)
{
    AC_Geometric_L1_Augment a = engaged();
    AC_Geometric_L1_Augment_Inputs bad = healthy();
    bad.observer_valid = false;
    a.update(bad, DT);
    ASSERT_EQ(AC_Geometric_L1_State::LATCHED, a.state());
    EXPECT_EQ(AC_Geometric_L1_Reason::FAULT_INVALID, a.reason());
    run(a, healthy(), 10.0f);
    EXPECT_EQ(AC_Geometric_L1_State::LATCHED, a.state()) << "recovery must not re-engage";
    a.drop();
    EXPECT_EQ(AC_Geometric_L1_State::LATCHED, a.state()) << "an ownership change must not clear it";
    a.clear_latch();
    EXPECT_EQ(AC_Geometric_L1_State::OFF, a.state());
}

TEST(L1Augment, AnObserverResetIsAFault)
{
    AC_Geometric_L1_Augment a = engaged();
    AC_Geometric_L1_Augment_Inputs in = healthy();
    in.observer_resets = 4;
    a.update(in, DT);
    EXPECT_EQ(AC_Geometric_L1_Reason::FAULT_RESET, a.reason());
}

TEST(L1Augment, ANonFiniteEstimateIsAFaultAndNeverInjected)
{
    AC_Geometric_L1_Augment a = engaged();
    AC_Geometric_L1_Augment_Inputs in = healthy();
    in.sigma_filtered_ned_mss = Vector3f{0, 0, NAN};
    a.update(in, DT);
    EXPECT_EQ(AC_Geometric_L1_Reason::FAULT_NONFINITE, a.reason());
    EXPECT_FALSE(a.injection_ned_mss().is_nan());
}

TEST(L1Augment, LowCollectivePausesWithoutLatching)
{
    // Decision B: a hard throttle cut is an ordinary pilot action. Even with
    // the observer over its limit, as it is then, this is a pause.
    AC_Geometric_L1_Augment a = engaged();
    AC_Geometric_L1_Augment_Inputs in = healthy();
    in.throttle = 0.07f;
    in.observer_over_limit = true;
    a.update(in, DT);
    EXPECT_EQ(AC_Geometric_L1_State::PAUSED, a.state());
    EXPECT_EQ(AC_Geometric_L1_Reason::PAUSE_THROTTLE, a.reason());
    // A pause hands over like a clean exit: the integrator carries the
    // deficit through it, as it would with L1 off.
    float handed = a.handover_ned_mss().z;
    run(a, in, 0.6f);
    EXPECT_EQ(AC_Geometric_L1_State::OFF, a.state());
    // (run() only reports the last frame; the per-frame sum is checked in
    // APauseIsForceNeutral.)
    (void)handed;
    // Collective back: it re-enters through admission, not by itself.
    AC_Geometric_L1_Augment_Inputs back = healthy();
    back.observer_admissible = false;
    run(a, back, 1.0f);
    EXPECT_EQ(AC_Geometric_L1_State::OFF, a.state());
    run(a, healthy(), 0.1f);
    EXPECT_EQ(AC_Geometric_L1_State::RAMP_IN, a.state());
}

TEST(L1Augment, PossibleLandingPauses)
{
    AC_Geometric_L1_Augment a = engaged();
    AC_Geometric_L1_Augment_Inputs in = healthy();
    in.land_complete_maybe = true;
    a.update(in, DT);
    EXPECT_EQ(AC_Geometric_L1_Reason::PAUSE_LANDED, a.reason());
}

TEST(L1Augment, LosingOwnershipDropsAtOnce)
{
    AC_Geometric_L1_Augment a = engaged();
    AC_Geometric_L1_Augment_Inputs in = healthy();
    in.owns_output = false;
    a.update(in, DT);
    EXPECT_EQ(AC_Geometric_L1_State::OFF, a.state());
    EXPECT_TRUE(a.injection_ned_mss().is_zero());
    EXPECT_TRUE(a.handover_ned_mss().is_zero());
}

TEST(L1Augment, ParametersDefaultOffWithTheApprovedRamps)
{
    AC_Geometric_L1_Augment_Params p;
    const AC_Geometric_L1_Augment_Config c = p.config();
    EXPECT_FALSE(c.enabled);
    EXPECT_FLOAT_EQ(1.0f, c.ramp_in_s);
    EXPECT_FLOAT_EQ(0.5f, c.ramp_out_s);
    EXPECT_FLOAT_EQ(0.12f, c.pause_throttle);
}

TEST(PositionPidHandover, AddsAccelerationThroughTheIntegralGain)
{
    AC_Geometric_Position_PID pid;
    AC_Geometric_Position_Gains g {};
    g.i = Vector3f{1.0f, 1.0f, 5.0f};
    pid.set_gains(g);
    AC_Geometric_Position_Integral_Limits lim {};
    lim.integral_error_m = Vector3f{10.0f, 10.0f, 10.0f};
    pid.set_integral_limits(lim);

    // After the handover, -K_I*e_I must have moved by -accel.
    AC_Geometric_State st {};
    st.attitude_body_to_ned.initialise();
    AC_Geometric_Target tg {};
    tg.attitude_body_to_ned.initialise();
    AC_Geometric_Position_Output before {}, after {};
    pid.update(st, tg, 0.0f, before);
    pid.add_integral_acceleration(Vector3f{0.0f, 0.0f, -1.5f});
    pid.update(st, tg, 0.0f, after);
    EXPECT_NEAR(-1.5f / 5.0f, after.integral_error_m.z - before.integral_error_m.z, 1e-6f);
}

TEST(PositionPidHandover, AnAxisWithoutIntegralGainIgnoresIt)
{
    AC_Geometric_Position_PID pid;
    AC_Geometric_Position_Gains g {};
    g.i = Vector3f{0.0f, 0.0f, 0.0f};
    pid.set_gains(g);
    AC_Geometric_State st {};
    st.attitude_body_to_ned.initialise();
    AC_Geometric_Target tg {};
    tg.attitude_body_to_ned.initialise();
    AC_Geometric_Position_Output out {};
    pid.add_integral_acceleration(Vector3f{0.0f, 0.0f, -1.5f});
    pid.update(st, tg, 0.0f, out);
    EXPECT_FLOAT_EQ(0.0f, out.integral_error_m.z);
}

TEST(L1Augment, RampInIsForceNeutral)
{
    // B1: during the ramp-in, what enters the feed-forward leaves the integrator.
    AC_Geometric_L1_Augment a;
    a.set_config(enabled_config());
    float taken = 0.0f, worst = 0.0f;
    for (int i = 0; i < 600; i++) {
        a.update(healthy(), DT);
        taken += a.handover_ned_mss().z;      // integrator contribution to A is -taken
        worst = MAX(worst, fabsf(a.injection_ned_mss().z - taken));
    }
    EXPECT_EQ(AC_Geometric_L1_State::ENGAGED, a.state());
    EXPECT_LT(worst, 1.0e-4f);
    EXPECT_NEAR(-SIGMA.z, taken, 1.0e-4f);
}

TEST(L1Augment, SaturationHoldsInsteadOfFaulting)
{
    AC_Geometric_L1_Augment a = engaged();
    const float lam = a.lambda();
    const Vector3f inj = a.injection_ned_mss();
    AC_Geometric_L1_Augment_Inputs in = healthy();
    in.thrust_limited = true;
    in.observer_over_limit = true;
    in.sigma_filtered_ned_mss = Vector3f{0, 0, -9.0f};
    run(a, in, 2.0f);
    EXPECT_EQ(AC_Geometric_L1_State::ENGAGED, a.state());
    EXPECT_FLOAT_EQ(lam, a.lambda());
    EXPECT_TRUE(inj == a.injection_ned_mss()) << "the estimate is held, not followed";
}

TEST(L1Augment, ADisableWinsOverAnInvalidObserver)
{
    // Switching the observer off arrives as disabled plus invalid on the same
    // frame; it must be a clean exit with handover, not a latch.
    AC_Geometric_L1_Augment a = engaged();
    AC_Geometric_L1_Augment_Config off = enabled_config();
    off.enabled = false;
    a.set_config(off);
    AC_Geometric_L1_Augment_Inputs in = healthy();
    in.observer_valid = false;
    a.update(in, DT);
    EXPECT_EQ(AC_Geometric_L1_State::RAMP_OUT, a.state());
    EXPECT_FALSE(a.handover_ned_mss().is_zero());
}

TEST(L1Augment, AFaultDuringRampInLatchesFromPartialLambda)
{
    AC_Geometric_L1_Augment a;
    a.set_config(enabled_config());
    run(a, healthy(), 0.3f);
    ASSERT_EQ(AC_Geometric_L1_State::RAMP_IN, a.state());
    const float partial = a.lambda();
    AC_Geometric_L1_Augment_Inputs bad = healthy();
    bad.observer_resets = 99;
    a.update(bad, DT);
    EXPECT_EQ(AC_Geometric_L1_State::LATCHED, a.state());
    EXPECT_LT(a.lambda(), partial);
    EXPECT_TRUE(a.handover_ned_mss().is_zero());
}

TEST(L1Augment, ADropOnlyRecordsAReasonWhenLeavingAnActiveState)
{
    AC_Geometric_L1_Augment a = engaged();
    AC_Geometric_L1_Augment_Inputs in = healthy();
    in.land_complete_maybe = true;
    run(a, in, 1.0f);
    ASSERT_EQ(AC_Geometric_L1_State::OFF, a.state());
    ASSERT_EQ(AC_Geometric_L1_Reason::PAUSE_LANDED, a.reason());
    a.drop(AC_Geometric_L1_Reason::RESET);       // Loiter's ground-safe reset
    EXPECT_EQ(AC_Geometric_L1_Reason::PAUSE_LANDED, a.reason());
    AC_Geometric_L1_Augment b = engaged();
    b.drop(AC_Geometric_L1_Reason::RESET);
    EXPECT_EQ(AC_Geometric_L1_Reason::RESET, b.reason());
}

TEST(L1Augment, ParameterBoundsAndNaN)
{
    AC_Geometric_L1_Augment_Params p;
    const AP_Param::GroupInfo* g = AC_Geometric_L1_Augment_Params::var_info;
    ASSERT_TRUE(AP_Param::set_object_value(&p, g, "L1_RIN", 0.0f));
    EXPECT_FLOAT_EQ(1.0f, p.config().ramp_in_s);
    ASSERT_TRUE(AP_Param::set_object_value(&p, g, "L1_RIN", NAN));
    EXPECT_FLOAT_EQ(1.0f, p.config().ramp_in_s);
    ASSERT_TRUE(AP_Param::set_object_value(&p, g, "L1_RIN", 10.0f));
    EXPECT_FLOAT_EQ(5.0f, p.config().ramp_in_s);
    ASSERT_TRUE(AP_Param::set_object_value(&p, g, "L1_ROUT", -1.0f));
    EXPECT_FLOAT_EQ(0.5f, p.config().ramp_out_s);
    ASSERT_TRUE(AP_Param::set_object_value(&p, g, "L1_PTHR", -0.2f));
    EXPECT_FLOAT_EQ(0.0f, p.config().pause_throttle);
    ASSERT_TRUE(AP_Param::set_object_value(&p, g, "L1_PTHR", NAN));
    EXPECT_FLOAT_EQ(0.12f, p.config().pause_throttle);
}

TEST(PositionPidHandover, ReportsWhatWasActuallyApplied)
{
    AC_Geometric_Position_PID pid;
    AC_Geometric_Position_Gains g {};
    g.i = Vector3f{0.0f, 1.0f, 5.0f};
    pid.set_gains(g);
    AC_Geometric_Position_Integral_Limits lim {};
    lim.integral_error_m = Vector3f{10.0f, 10.0f, 0.1f};
    pid.set_integral_limits(lim);
    const Vector3f applied = pid.add_integral_acceleration(Vector3f{2.0f, 0.5f, 3.0f});
    EXPECT_FLOAT_EQ(0.0f, applied.x) << "no K_I, nothing taken";
    EXPECT_FLOAT_EQ(0.5f, applied.y);
    EXPECT_NEAR(0.5f, applied.z, 1e-6f) << "clipped at IMAX 0.1 m * K_I 5";
}

TEST(L1Augment, APauseIsForceNeutral)
{
    AC_Geometric_L1_Augment a = engaged();
    AC_Geometric_L1_Augment_Inputs in = healthy();
    in.throttle = 0.07f;
    float handed = 0.0f, worst = 0.0f;
    const float start = a.injection_ned_mss().z;
    for (int i = 0; i < 400; i++) {
        a.update(in, DT);
        handed += a.handover_ned_mss().z;
        worst = MAX(worst, fabsf(a.injection_ned_mss().z - handed - start));
    }
    EXPECT_EQ(AC_Geometric_L1_State::OFF, a.state());
    EXPECT_LT(worst, 1.0e-4f);
    EXPECT_NEAR(SIGMA.z, handed, 1.0e-4f);
}

TEST(L1Augment, ASaturationReleaseSettlesThenWalksBack)
{
    // Review F1/F3: the estimate is still polluted for a while after the
    // limit clears. Neither latch on it nor step to it.
    AC_Geometric_L1_Augment a = engaged();
    const float before = a.injection_ned_mss().z;
    AC_Geometric_L1_Augment_Inputs lim = healthy();
    lim.thrust_limited = true;
    lim.observer_over_limit = true;
    lim.sigma_filtered_ned_mss = Vector3f{0, 0, -9.0f};
    run(a, lim, 1.0f);
    AC_Geometric_L1_Augment_Inputs released = lim;
    released.thrust_limited = false;          // the flag clears, the estimate has not
    run(a, released, 0.45f);
    EXPECT_EQ(AC_Geometric_L1_State::ENGAGED, a.state()) << "no latch inside the settle window";
    EXPECT_FLOAT_EQ(before, a.injection_ned_mss().z);
    // Settled, and the disturbance really did change while saturated: the
    // held value walks to the new estimate at the rejoin rate, never steps.
    AC_Geometric_L1_Augment_Inputs moved = healthy();
    moved.sigma_filtered_ned_mss.z = SIGMA.z - 1.0f;
    float last = a.injection_ned_mss().z, worst = 0.0f;
    for (int i = 0; i < 400; i++) {
        a.update(moved, DT);
        worst = MAX(worst, fabsf(a.injection_ned_mss().z - last));
        last = a.injection_ned_mss().z;
    }
    EXPECT_FALSE(a.state() == AC_Geometric_L1_State::LATCHED);
    EXPECT_LE(worst, AC_Geometric_L1_Augment::REJOIN_RATE_MSS_PER_S * DT + 1.0e-5f);
    EXPECT_FLOAT_EQ(-moved.sigma_filtered_ned_mss.z, a.injection_ned_mss().z);
}

TEST(L1Augment, NoAdmissionInsideTheSettleWindow)
{
    AC_Geometric_L1_Augment a;
    a.set_config(enabled_config());
    AC_Geometric_L1_Augment_Inputs lim = healthy();
    lim.thrust_limited = true;
    run(a, lim, 0.2f);
    run(a, healthy(), 0.4f);
    EXPECT_EQ(AC_Geometric_L1_State::OFF, a.state());
    run(a, healthy(), 0.2f);
    EXPECT_EQ(AC_Geometric_L1_State::RAMP_IN, a.state());
}

TEST(L1Augment, AHoldEndsInACleanExitAfterHoldMax)
{
    // #13: a hold has a ceiling, and leaving through it is a clean exit.
    AC_Geometric_L1_Augment a = engaged();
    AC_Geometric_L1_Augment_Inputs lim = healthy();
    lim.thrust_limited = true;
    const float start = a.injection_ned_mss().z;
    const int n_max = int(AC_Geometric_L1_Augment::HOLD_MAX_S / DT);
    float handed = 0.0f, worst = 0.0f;
    bool left_early = false;
    for (int i = 0; i < n_max + 400; i++) {
        a.update(lim, DT);
        if (i < n_max - 2 && a.state() != AC_Geometric_L1_State::ENGAGED) {
            left_early = true;
        }
        handed += a.handover_ned_mss().z;
        worst = MAX(worst, fabsf(a.injection_ned_mss().z - handed - start));
    }
    EXPECT_FALSE(left_early);
    EXPECT_EQ(AC_Geometric_L1_State::OFF, a.state());
    EXPECT_EQ(AC_Geometric_L1_Reason::HOLD_TIMEOUT, a.reason());
    EXPECT_LT(worst, 1.0e-4f) << "the exit hands over; force stays continuous";
    EXPECT_NEAR(SIGMA.z, handed, 1.0e-4f);
}

TEST(L1Augment, AFlickeringLimitStillTimesOutAndBlocksReentryUntilQuiet)
{
    // The case the ceiling exists for: 10 ms of limit every 300 ms keeps the
    // 0.5 s settle window re-arming for ever.
    AC_Geometric_L1_Augment a = engaged();
    AC_Geometric_L1_Augment_Inputs in = healthy();
    int i = 0;
    auto flicker = [&](float seconds) {
        for (int n = int(seconds / DT + 0.5f); n > 0; n--, i++) {
            in.thrust_limited = (i % 120) < 4;
            a.update(in, DT);
        }
    };
    flicker(2.4f);
    EXPECT_EQ(AC_Geometric_L1_State::ENGAGED, a.state());
    flicker(0.8f);
    EXPECT_EQ(AC_Geometric_L1_Reason::HOLD_TIMEOUT, a.reason());
    flicker(3.0f);
    EXPECT_EQ(AC_Geometric_L1_State::OFF, a.state()) << "no re-entry while the limit keeps flickering";
    run(a, healthy(), 0.6f);
    EXPECT_NE(AC_Geometric_L1_State::OFF, a.state()) << "re-admitted once the limit is quiet";
    EXPECT_NE(AC_Geometric_L1_State::LATCHED, a.state());
}

TEST(L1Augment, SeparateHoldsDoNotAddUp)
{
    AC_Geometric_L1_Augment a = engaged();
    AC_Geometric_L1_Augment_Inputs lim = healthy();
    lim.thrust_limited = true;
    // The ceiling counts the whole hold, settle window included, so each of
    // these holds lasts 1.5 + 0.5 = 2.0 s: under the ceiling separately,
    // over it if they were added together.
    run(a, lim, 1.5f);
    run(a, healthy(), 1.0f);      // past the settle window: the hold ends
    run(a, lim, 1.5f);
    EXPECT_EQ(AC_Geometric_L1_State::ENGAGED, a.state());
    EXPECT_EQ(AC_Geometric_L1_Reason::NONE, a.reason());
}

TEST(L1Augment, HoldStatusSaysWhatTheEstimateIsDoing)
{
    AC_Geometric_L1_Augment a = engaged();
    EXPECT_EQ(0, a.hold_status());
    AC_Geometric_L1_Augment_Inputs lim = healthy();
    lim.thrust_limited = true;
    run(a, lim, 0.2f);
    EXPECT_EQ(1, a.hold_status());
    AC_Geometric_L1_Augment_Inputs moved = healthy();
    moved.sigma_filtered_ned_mss.z = SIGMA.z - 1.0f;
    run(a, moved, 0.6f);          // settle window over, 1 m/s^2 still to walk
    EXPECT_EQ(2, a.hold_status());
    run(a, moved, 0.5f);
    EXPECT_EQ(0, a.hold_status());
    AC_Geometric_L1_Augment off;
    off.set_config(enabled_config());
    run(off, lim, 0.2f);
    EXPECT_EQ(0, off.hold_status()) << "only meaningful while engaged";
}

AP_GTEST_MAIN()
