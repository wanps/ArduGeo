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
    EXPECT_TRUE(a.handover_ned_mss().is_zero());
    run(a, in, 0.6f);
    EXPECT_EQ(AC_Geometric_L1_State::OFF, a.state());
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

AP_GTEST_MAIN()
