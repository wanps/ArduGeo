#include <AP_gtest.h>

#include <AC_GeometricControl/AC_GeometricControl.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// Closed-loop tests of the L1 wiring inside AC_GeometricControl: the real
// update(), the real position PID, observer and augment, flying a 1-D
// vertical plant whose thrust is k * g * throttle / h. k < 1 is a weaker
// aircraft (or a payload); h is the hover throttle the controller believes.

namespace {

constexpr float DT = 0.0025f;

bool set_group_param(AC_GeometricControl& c, const AP_Param::GroupInfo* group, const char* name, float v)
{
    for (const AP_Param::GroupInfo* gi = AC_GeometricControl::var_info; gi->type != AP_PARAM_NONE; gi++) {
        if (gi->type == AP_PARAM_GROUP && gi->group_info == group) {
            return AP_Param::set_object_value(reinterpret_cast<const uint8_t*>(&c) + gi->offset, group, name, v);
        }
    }
    return false;
}
bool set_obs(AC_GeometricControl& c, const char* n, float v) { return set_group_param(c, AC_Geometric_L1_Observer_Params::var_info, n, v); }
bool set_l1(AC_GeometricControl& c, const char* n, float v) { return set_group_param(c, AC_Geometric_L1_Augment_Params::var_info, n, v); }

struct Loop {
    AC_GeometricControl c;
    AC_Geometric_Target tgt {};
    Vector3f pos, vel;
    float h, k;
    float t = 0.0f;
    bool motor_limited = false;
    bool ever_latched = false;
    // AP_Motors-side cap on collective, below the mapper's own limit. Its
    // flag and applied value reach the controller one frame late, as they do
    // on the vehicle. Negative disables it.
    float motor_cap = -1.0f;
    float applied_last = -1.0f;
    float worst_ij_step = 0.0f;
    float last_ij = 0.0f;

    Loop(float hover, float gain, bool observer, bool l1) : h(hover), k(gain)
    {
        EXPECT_TRUE(set_obs(c, "L1_OBS_EN", observer ? 1 : 0));
        EXPECT_TRUE(set_l1(c, "L1_EN", l1 ? 1 : 0));
        c.set_enabled(true);
        c.set_hover_throttle_reference(h);
        tgt.attitude_body_to_ned.initialise();
        tgt.build_attitude_from_position = true;
        tgt.shape_position_target = false;
        tgt.shape_yaw_target = false;
        c.begin_output_ownership(Vector3f{});
    }
    void step()
    {
        AC_Geometric_State st {};
        st.position_ned_m = pos;
        st.velocity_ned_ms = vel;
        st.attitude_body_to_ned.initialise();
        st.thrust_limited = motor_limited;
        st.applied_throttle_norm = applied_last;
        c.update(st, tgt, DT);
        float thr = c.get_output().mapped.throttle_norm;
        if (motor_cap >= 0.0f) {
            motor_limited = thr > motor_cap;
            thr = MIN(thr, motor_cap);
        }
        applied_last = thr;
        worst_ij_step = MAX(worst_ij_step, fabsf(c.l1_injection_used_z() - last_ij));
        last_ij = c.l1_injection_used_z();
        const float az = GRAVITY_MSS - k * GRAVITY_MSS * thr / h;
        vel.z += az * DT;
        pos.z += vel.z * DT;
        t += DT;
        ever_latched |= c.l1_augment().state() == AC_Geometric_L1_State::LATCHED;
    }
    void run(float seconds) { for (int i = 0; i < int(seconds / DT + 0.5f); i++) { step(); } }
    // Largest |z - z0| over the next `seconds`.
    float excursion(float seconds)
    {
        const float z0 = pos.z;
        float worst = 0.0f;
        for (int i = 0; i < int(seconds / DT + 0.5f); i++) {
            step();
            worst = MAX(worst, fabsf(pos.z - z0));
        }
        return worst;
    }
    AC_Geometric_L1_State st() const { return c.l1_augment().state(); }
};

}  // namespace

TEST(L1ClosedLoop, WithL1OffTheObserverChangesNoCommand)
{
    // R-23 for the configuration that ships: GEO_L1_EN at zero.
    Loop with_obs(0.30f, 0.87f, true, false);
    Loop without(0.30f, 0.87f, false, false);
    for (int i = 0; i < 8000; i++) {
        with_obs.step();
        without.step();
        const AC_Geometric_Mapped_Output& a = with_obs.c.get_output().mapped;
        const AC_Geometric_Mapped_Output& b = without.c.get_output().mapped;
        // Bit-identical, not approximately equal: the observer must not
        // touch the command path at all in this configuration.
        ASSERT_EQ(0, memcmp(&a.throttle_norm, &b.throttle_norm, sizeof(float))) << "frame " << i;
        ASSERT_EQ(0, memcmp(&a.rpy_norm, &b.rpy_norm, sizeof(Vector3f))) << "frame " << i;
    }
}

TEST(L1ClosedLoop, EngagingOverAnAbsorbedDeficitIsForceNeutral)
{
    // Review finding B1: the integrator already holds the deficit sigma
    // reports. Engaging must move it out, not command it a second time.
    Loop l(0.30f, 0.87f, true, false);
    l.run(30.0f);
    const float ki = 5.0f;   // GEO_POS_KI_Z default
    const float held_before = ki * l.c.get_output().position.integral_error_m.z;
    ASSERT_TRUE(set_l1(l.c, "L1_EN", 1));
    const float worst = l.excursion(12.0f);
    EXPECT_EQ(AC_Geometric_L1_State::ENGAGED, l.st());
    EXPECT_LT(worst, 0.02f) << "engage transient";
    const float held_after = ki * l.c.get_output().position.integral_error_m.z;
    EXPECT_LT(fabsf(held_after), 0.2f * fabsf(held_before)) << "the integrator handed its load to L1";
}

TEST(L1ClosedLoop, AStepDeficitIsRejectedFaster)
{
    Loop on(0.30f, 1.0f, true, true), off(0.30f, 1.0f, true, false);
    on.run(10.0f); off.run(10.0f);
    ASSERT_EQ(AC_Geometric_L1_State::ENGAGED, on.st());
    on.k = 0.87f; off.k = 0.87f;
    const float d_on = on.excursion(10.0f), d_off = off.excursion(10.0f);
    EXPECT_LT(d_on, 0.5f * d_off) << "on " << d_on << " off " << d_off;
}

TEST(L1ClosedLoop, CleanExitIsContinuousThroughTheController)
{
    Loop l(0.30f, 0.87f, true, true);
    l.run(20.0f);
    ASSERT_EQ(AC_Geometric_L1_State::ENGAGED, l.st());
    ASSERT_TRUE(set_l1(l.c, "L1_EN", 0));
    EXPECT_LT(l.excursion(5.0f), 0.01f);
    EXPECT_EQ(AC_Geometric_L1_State::OFF, l.st());
    EXPECT_EQ(AC_Geometric_L1_Reason::DISABLED, l.c.l1_augment().reason());
}

TEST(L1ClosedLoop, SwitchingTheObserverOffIsACleanExitNotAFault)
{
    Loop l(0.30f, 0.87f, true, true);
    l.run(20.0f);
    ASSERT_EQ(AC_Geometric_L1_State::ENGAGED, l.st());
    ASSERT_TRUE(set_obs(l.c, "L1_OBS_EN", 0));
    EXPECT_LT(l.excursion(5.0f), 0.01f);
    EXPECT_FALSE(l.ever_latched);
    EXPECT_EQ(AC_Geometric_L1_Reason::DISABLED, l.c.l1_augment().reason());
}

TEST(L1ClosedLoop, AResetMidFlightKeepsOwnershipAndReadmits)
{
    // The SITL regression: Loiter resets the controller while geometry keeps
    // the outputs, and no new rising edge follows.
    Loop l(0.30f, 1.0f, true, true);
    l.run(10.0f);
    ASSERT_EQ(AC_Geometric_L1_State::ENGAGED, l.st());
    l.c.reset();
    EXPECT_EQ(AC_Geometric_L1_Reason::RESET, l.c.l1_augment().reason());
    l.run(8.0f);
    EXPECT_EQ(AC_Geometric_L1_State::ENGAGED, l.st());
}

TEST(L1ClosedLoop, EndingOwnershipRemovesTheInjectionOnTheNextFrame)
{
    Loop l(0.30f, 0.87f, true, true);
    l.run(20.0f);
    ASSERT_GT(fabsf(l.c.l1_injection_used_z()), 0.1f);
    l.c.end_output_ownership();
    l.step();
    EXPECT_FLOAT_EQ(0.0f, l.c.l1_injection_used_z());
    EXPECT_EQ(AC_Geometric_L1_State::OFF, l.st());
}

TEST(L1ClosedLoop, CollectiveSaturationDoesNotLatch)
{
    // Review finding S1. A 15 m climb step with a high hover throttle runs
    // the collective into its limit for seconds. Commanded and applied thrust
    // then differ; L1 must not treat the gap as a disturbance and latch.
    Loop l(0.45f, 1.0f, true, true);
    l.run(10.0f);
    ASSERT_EQ(AC_Geometric_L1_State::ENGAGED, l.st());
    l.tgt.position_ned_m.z = -15.0f;
    bool saturated = false;
    for (int i = 0; i < int(15.0f / DT); i++) {
        l.step();
        saturated |= l.c.get_output().mapped.throttle_limited;
    }
    ASSERT_TRUE(saturated) << "the test must actually saturate";
    EXPECT_FALSE(l.ever_latched);
}

TEST(L1ClosedLoop, MotorSaturationReportedByTheVehicleHolds)
{
    // AP_Motors-side saturation the controller cannot see: the vehicle says so.
    Loop l(0.30f, 0.87f, true, true);
    l.run(20.0f);
    const float lam = l.c.l1_augment().lambda();
    const float inj = l.c.l1_injection_used_z();
    l.motor_limited = true;
    l.run(2.0f);
    EXPECT_FLOAT_EQ(lam, l.c.l1_augment().lambda());
    EXPECT_FLOAT_EQ(inj, l.c.l1_injection_used_z());
    EXPECT_FALSE(l.ever_latched);
}

TEST(L1ClosedLoop, MotorSideSaturationWithRealShortfallDoesNotLatchOrStep)
{
    // Review F1: AP_Motors limits collective (rpy priority, current limit)
    // where the mapper cannot see it. The reviewer's harness latched in every
    // case, on the release frame for two of them.
    for (const float cap : {0.40f, 0.45f, 0.55f}) {
        Loop l(0.30f, 1.0f, true, true);
        l.motor_cap = cap;
        l.run(10.0f);
        ASSERT_EQ(AC_Geometric_L1_State::ENGAGED, l.st()) << "cap " << cap;
        l.worst_ij_step = 0.0f;
        l.tgt.position_ned_m.z = -15.0f;
        bool limited = false;
        float worst_sigma = 0.0f;
        for (int i = 0; i < int(20.0f / DT); i++) {
            l.step();
            limited |= l.motor_limited;
            worst_sigma = MAX(worst_sigma, fabsf(l.c.get_l1_observer_output().sigma_filtered_ned_mss.z));
        }
        ASSERT_TRUE(limited) << "cap " << cap << " must actually limit";
        // The plant matches the model (k = 1), so there is no disturbance to
        // find. Fed what AP_Motors actually applied, the observer must not
        // invent one out of the cap; the reviewer saw 14.5 to 19.4 without.
        EXPECT_LT(worst_sigma, 2.0f) << "cap " << cap;
        EXPECT_FALSE(l.ever_latched) << "cap " << cap;
        EXPECT_LT(l.worst_ij_step, 0.05f) << "cap " << cap;
    }
}

TEST(L1ClosedLoop, ChatteringMotorLimitDoesNotStepTheInjection)
{
    // Review F1, second scenario: the flag at 40 % duty through a deficit step.
    Loop l(0.30f, 1.0f, true, true);
    l.run(10.0f);
    ASSERT_EQ(AC_Geometric_L1_State::ENGAGED, l.st());
    l.worst_ij_step = 0.0f;
    l.k = 0.87f;
    for (int i = 0; i < int(10.0f / DT); i++) {
        l.motor_limited = (i % 40) < 16;
        l.step();
    }
    EXPECT_FALSE(l.ever_latched);
    EXPECT_LT(l.worst_ij_step, 0.05f);
}

TEST(L1ClosedLoop, AFlickeringLimitEndsInACleanExitThatFliesLikeL1Off)
{
    // #13, the reviewer's scenario: the limit flag flickers (10 ms in every
    // 300 ms) and, 10 s in, the deficit goes away. Before the ceiling, L1
    // stayed "engaged" on its frozen value and that value became the
    // disturbance. Now it leaves cleanly and flies as L1 off would.
    Loop on(0.30f, 0.87f, true, true), off(0.30f, 0.87f, true, false);
    on.run(20.0f);
    off.run(20.0f);
    ASSERT_EQ(AC_Geometric_L1_State::ENGAGED, on.st());
    on.worst_ij_step = 0.0f;
    const float z_on = on.pos.z, z_off = off.pos.z;
    float worst_on = 0.0f, worst_off = 0.0f;
    const int change = int(10.0f / DT);
    for (int i = 0; i < int(40.0f / DT); i++) {
        const bool flick = (i % 120) < 4;
        on.motor_limited = flick;
        off.motor_limited = flick;
        if (i == change) {
            on.k = 1.0f;
            off.k = 1.0f;
        }
        on.step();
        off.step();
        if (i >= change) {
            worst_on = MAX(worst_on, fabsf(on.pos.z - z_on));
            worst_off = MAX(worst_off, fabsf(off.pos.z - z_off));
        }
    }
    EXPECT_FALSE(on.ever_latched);
    EXPECT_EQ(AC_Geometric_L1_Reason::HOLD_TIMEOUT, on.c.l1_augment().reason());
    EXPECT_EQ(AC_Geometric_L1_State::OFF, on.st());
    EXPECT_LT(on.worst_ij_step, 0.05f);
    EXPECT_LT(worst_on, worst_off + 0.01f) << "on " << worst_on << " off " << worst_off;
    printf("flicker scenario: L1 on %.3f m, L1 off %.3f m\n", worst_on, worst_off);
}

AP_GTEST_MAIN()
