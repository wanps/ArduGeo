#include <AP_gtest.h>

#include <AC_GeometricControl/AC_Geometric_LoiterReference.h>

#include <limits>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

constexpr float dt_s = 0.01f;

AC_Geometric_LoiterReference_Limits default_limits()
{
    AC_Geometric_LoiterReference_Limits limits;
    limits.speed_xy_max_ms = 2.0f;
    limits.accel_xy_max_mss = 1.0f;
    limits.accel_xy_total_max_mss = 3.0f;
    limits.jerk_xy_max_msss = 2.0f;
    limits.brake_delay_s = 0.2f;
    limits.brake_accel_max_mss = 2.5f;
    limits.brake_jerk_max_msss = 5.0f;
    limits.speed_up_max_ms = 1.5f;
    limits.speed_down_max_ms = 1.0f;
    limits.accel_z_max_mss = 1.0f;
    limits.jerk_z_max_msss = 2.0f;
    limits.brake_accel_z_max_mss = 1.0f;
    limits.brake_jerk_z_max_msss = 5.0f;
    limits.yaw_rate_max_rads = 1.0f;
    limits.yaw_accel_max_radss = 1.0f;
    limits.yaw_jerk_max_radsss = 2.0f;
    limits.yaw_brake_accel_max_radss = 2.0f;
    limits.yaw_brake_jerk_max_radsss = 6.0f;
    return limits;
}

AC_Geometric_State state_at_origin()
{
    AC_Geometric_State state;
    state.attitude_body_to_ned.initialise();
    return state;
}

bool update_reference(AC_Geometric_LoiterReference& reference,
                      const AC_Geometric_LoiterReference_Input& input,
                      const AC_Geometric_LoiterReference_Limits& limits,
                      AC_Geometric_Target& target,
                      AC_Geometric_LoiterReference_Status& status)
{
    return reference.update(input, limits, dt_s, target, status);
}

void expect_target_finite(const AC_Geometric_Target& target)
{
    EXPECT_FALSE(target.position_ned_m.is_nan());
    EXPECT_FALSE(target.position_ned_m.is_inf());
    EXPECT_FALSE(target.velocity_ned_ms.is_nan());
    EXPECT_FALSE(target.velocity_ned_ms.is_inf());
    EXPECT_FALSE(target.accel_ned_mss.is_nan());
    EXPECT_FALSE(target.accel_ned_mss.is_inf());
    EXPECT_TRUE(isfinite(target.yaw_rad));
    EXPECT_TRUE(isfinite(target.yaw_rate_rads));
}

}

TEST(AC_Geometric_LoiterReference, RequiresFiniteResetAndValidDt)
{
    AC_Geometric_LoiterReference reference;
    AC_Geometric_Target target;
    AC_Geometric_LoiterReference_Status status;
    AC_Geometric_LoiterReference_Input input;
    const auto limits = default_limits();

    EXPECT_FALSE(reference.update(input, limits, dt_s, target, status));
    EXPECT_TRUE(reference.reset(state_at_origin(), 0.0f));
    EXPECT_FALSE(reference.update(input, limits, 0.0f, target, status));
    EXPECT_FALSE(reference.update(input, limits, 0.2f, target, status));
    EXPECT_TRUE(reference.update(input, limits, dt_s, target, status));
}

TEST(AC_Geometric_LoiterReference, DedicatedBrakeProfileDefaults)
{
    AC_Geometric_LoiterReference_Params params;
    const AC_Geometric_LoiterReference_Profile profile = params.get();

    EXPECT_FLOAT_EQ(profile.brake_delay_s, 0.2f);
    EXPECT_FLOAT_EQ(profile.brake_accel_max_mss, 2.5f);
    EXPECT_FLOAT_EQ(profile.brake_jerk_max_msss, 5.0f);
    EXPECT_FLOAT_EQ(profile.jerk_z_max_msss, 5.0f);
    EXPECT_FLOAT_EQ(profile.brake_accel_z_max_mss, 2.5f);
    EXPECT_FLOAT_EQ(profile.brake_jerk_z_max_msss, 5.0f);
    EXPECT_FLOAT_EQ(profile.yaw_brake_accel_max_radss, 2.0f);
    EXPECT_FLOAT_EQ(profile.yaw_brake_jerk_max_radsss, 6.0f);
}

TEST(AC_Geometric_LoiterReference, GroundResetAndZeroCommandHoldState)
{
    AC_Geometric_State state = state_at_origin();
    state.position_ned_m = Vector3f{5.0f, -2.0f, -3.0f};
    AC_Geometric_LoiterReference reference;
    ASSERT_TRUE(reference.reset(state, 1.2f));

    const auto limits = default_limits();
    AC_Geometric_LoiterReference_Input input;
    AC_Geometric_Target target;
    AC_Geometric_LoiterReference_Status status;
    for (uint8_t i = 0; i < 100; i++) {
        ASSERT_TRUE(update_reference(reference, input, limits, target, status));
        expect_target_finite(target);
    }

    EXPECT_NEAR(target.position_ned_m.x, 5.0f, 1.0e-5f);
    EXPECT_NEAR(target.position_ned_m.y, -2.0f, 1.0e-5f);
    EXPECT_NEAR(target.position_ned_m.z, -3.0f, 1.0e-5f);
    EXPECT_NEAR(target.velocity_ned_ms.length(), 0.0f, 1.0e-6f);
    EXPECT_NEAR(target.accel_ned_mss.length(), 0.0f, 1.0e-6f);
}

TEST(AC_Geometric_LoiterReference, AirborneResetPreservesMeasuredContinuity)
{
    AC_Geometric_State state = state_at_origin();
    state.position_ned_m = Vector3f{5.0f, -2.0f, -10.0f};
    state.velocity_ned_ms = Vector3f{1.0f, -0.5f, 0.2f};
    AC_Geometric_LoiterReference reference;
    ASSERT_TRUE(reference.reset(state, -0.7f, 0.2f));

    EXPECT_NEAR(reference.position_ref_ned_m().x, 5.0f, 1.0e-6f);
    EXPECT_NEAR(reference.velocity_ref_ned_ms().x, 1.0f, 1.0e-6f);
    EXPECT_NEAR(reference.velocity_ref_ned_ms().y, -0.5f, 1.0e-6f);
    EXPECT_NEAR(reference.velocity_ref_ned_ms().z, 0.2f, 1.0e-6f);

    AC_Geometric_LoiterReference_Input input;
    AC_Geometric_Target target;
    AC_Geometric_LoiterReference_Status status;
    const auto limits = default_limits();
    ASSERT_TRUE(update_reference(reference, input, limits, target, status));
    EXPECT_LE((target.velocity_ned_ms - state.velocity_ned_ms).length(),
              limits.accel_xy_total_max_mss * dt_s + limits.accel_z_max_mss * dt_s);
}

TEST(AC_Geometric_LoiterReference, InvalidLimitsOrCommandFailWithoutMutation)
{
    AC_Geometric_LoiterReference reference;
    ASSERT_TRUE(reference.reset(state_at_origin(), 0.0f));
    auto limits = default_limits();
    AC_Geometric_LoiterReference_Input input;
    AC_Geometric_Target target;
    AC_Geometric_LoiterReference_Status status;
    ASSERT_TRUE(update_reference(reference, input, limits, target, status));
    const Vector3p position_before = reference.position_ref_ned_m();
    const Vector3f velocity_before = reference.velocity_ref_ned_ms();

    limits.speed_xy_max_ms = std::numeric_limits<float>::infinity();
    EXPECT_FALSE(update_reference(reference, input, limits, target, status));
    EXPECT_EQ(reference.position_ref_ned_m(), position_before);
    EXPECT_EQ(reference.velocity_ref_ned_ms(), velocity_before);

    limits = default_limits();
    input.pilot_accel_ne_mss.x = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(update_reference(reference, input, limits, target, status));
    EXPECT_EQ(reference.position_ref_ned_m(), position_before);
    EXPECT_EQ(reference.velocity_ref_ned_ms(), velocity_before);

    input = {};
    limits.yaw_brake_accel_max_radss = 0.0f;
    EXPECT_FALSE(update_reference(reference, input, limits, target, status));
    EXPECT_EQ(reference.position_ref_ned_m(), position_before);
    EXPECT_EQ(reference.velocity_ref_ned_ms(), velocity_before);

    limits = default_limits();
    limits.brake_accel_z_max_mss = 0.0f;
    EXPECT_FALSE(update_reference(reference, input, limits, target, status));
    EXPECT_EQ(reference.position_ref_ned_m(), position_before);
    EXPECT_EQ(reference.velocity_ref_ned_ms(), velocity_before);
}

TEST(AC_Geometric_LoiterReference, HorizontalReferenceIsPvaConsistentAndLimited)
{
    AC_Geometric_LoiterReference reference;
    ASSERT_TRUE(reference.reset(state_at_origin(), 0.0f));
    const auto limits = default_limits();

    AC_Geometric_LoiterReference_Input input;
    input.pilot_accel_ne_mss.x = 10.0f;
    input.pilot_xy_active = true;
    AC_Geometric_Target target;
    AC_Geometric_LoiterReference_Status status;

    Vector3f previous_position;
    Vector3f previous_velocity;
    Vector3f previous_accel;
    for (uint16_t i = 0; i < 800; i++) {
        previous_position = target.position_ned_m;
        previous_velocity = target.velocity_ned_ms;
        previous_accel = target.accel_ned_mss;
        ASSERT_TRUE(update_reference(reference, input, limits, target, status));
        EXPECT_LE(target.velocity_ned_ms.xy().length(), limits.speed_xy_max_ms + 1.0e-4f);
        EXPECT_LE(target.accel_ned_mss.xy().length(), limits.accel_xy_max_mss + 1.0e-4f);
        if (i > 0) {
            if (!status.speed_xy_limited) {
                EXPECT_LE((target.accel_ned_mss.xy() - previous_accel.xy()).length(),
                          limits.jerk_xy_max_msss * dt_s + 1.0e-5f);
            }
            EXPECT_NEAR(target.velocity_ned_ms.x - previous_velocity.x,
                        target.accel_ned_mss.x * dt_s,
                        1.0e-5f);
            EXPECT_NEAR(target.position_ned_m.x - previous_position.x,
                        previous_velocity.x * dt_s + 0.5f * target.accel_ned_mss.x * sq(dt_s),
                        1.0e-5f);
        }
    }
    EXPECT_GT(target.velocity_ned_ms.x, 1.8f);
}

TEST(AC_Geometric_LoiterReference, NeutralStickBrakesWithoutReversal)
{
    AC_Geometric_LoiterReference reference;
    ASSERT_TRUE(reference.reset(state_at_origin(), 0.0f));
    const auto limits = default_limits();
    AC_Geometric_LoiterReference_Input input;
    input.pilot_accel_ne_mss.x = 1.0f;
    input.pilot_xy_active = true;
    AC_Geometric_Target target;
    AC_Geometric_LoiterReference_Status status;

    for (uint16_t i = 0; i < 200; i++) {
        ASSERT_TRUE(update_reference(reference, input, limits, target, status));
    }
    ASSERT_GT(target.velocity_ned_ms.x, 0.1f);

    input.pilot_accel_ne_mss.zero();
    input.pilot_xy_active = false;
    bool saw_braking = false;
    bool saw_settled = false;
    float first_brake_elapsed_s = -1.0f;
    float peak_brake_accel_mss = 0.0f;
    for (uint16_t i = 0; i < 800; i++) {
        ASSERT_TRUE(update_reference(reference, input, limits, target, status));
        const float release_elapsed_s = (i + 1) * dt_s;
        if (status.braking && first_brake_elapsed_s < 0.0f) {
            first_brake_elapsed_s = release_elapsed_s;
        }
        if (release_elapsed_s < limits.brake_delay_s - 1.0e-5f) {
            EXPECT_FALSE(status.braking);
        }
        saw_braking |= status.braking;
        saw_settled |= status.xy_settled;
        peak_brake_accel_mss = MAX(peak_brake_accel_mss, target.accel_ned_mss.xy().length());
        EXPECT_GE(target.velocity_ned_ms.x, -1.0e-5f);
    }
    EXPECT_TRUE(saw_braking);
    EXPECT_TRUE(saw_settled);
    EXPECT_GE(first_brake_elapsed_s, limits.brake_delay_s - 1.0e-5f);
    EXPECT_LE(first_brake_elapsed_s, limits.brake_delay_s + dt_s + 1.0e-5f);
    EXPECT_GT(peak_brake_accel_mss, limits.accel_xy_max_mss + 0.1f);
    EXPECT_LE(peak_brake_accel_mss, limits.accel_xy_total_max_mss + 1.0e-4f);
    EXPECT_NEAR(target.velocity_ned_ms.xy().length(), 0.0f, 1.0e-3f);
    EXPECT_NEAR(target.accel_ned_mss.xy().length(), 0.0f, 1.0e-3f);

    const Vector2f settled_position = target.position_ned_m.xy();
    for (uint8_t i = 0; i < 100; i++) {
        ASSERT_TRUE(update_reference(reference, input, limits, target, status));
        EXPECT_NEAR(target.velocity_ned_ms.xy().length(), 0.0f, 1.0e-6f);
        EXPECT_NEAR(target.accel_ned_mss.xy().length(), 0.0f, 1.0e-6f);
    }
    EXPECT_NEAR((target.position_ned_m.xy() - settled_position).length(), 0.0f, 1.0e-6f);
}

TEST(AC_Geometric_LoiterReference, ReverseCommandRemainsContinuousAndBounded)
{
    AC_Geometric_LoiterReference reference;
    ASSERT_TRUE(reference.reset(state_at_origin(), 0.0f));
    const auto limits = default_limits();
    AC_Geometric_LoiterReference_Input input;
    input.pilot_accel_ne_mss.x = limits.accel_xy_max_mss;
    input.pilot_xy_active = true;
    AC_Geometric_Target target;
    AC_Geometric_LoiterReference_Status status;
    for (uint16_t i = 0; i < 200; i++) {
        ASSERT_TRUE(update_reference(reference, input, limits, target, status));
    }

    input.pilot_accel_ne_mss.x = -limits.accel_xy_max_mss;
    for (uint16_t i = 0; i < 800; i++) {
        const Vector3f previous_accel = target.accel_ned_mss;
        ASSERT_TRUE(update_reference(reference, input, limits, target, status));
        EXPECT_LE(target.velocity_ned_ms.xy().length(), limits.speed_xy_max_ms + 1.0e-4f);
        EXPECT_LE(target.accel_ned_mss.xy().length(), limits.accel_xy_max_mss + 1.0e-4f);
        if (!status.speed_xy_limited) {
            EXPECT_LE((target.accel_ned_mss.xy() - previous_accel.xy()).length(),
                      limits.jerk_xy_max_msss * dt_s + 1.0e-5f);
        }
    }
    EXPECT_LT(target.velocity_ned_ms.x, -0.5f);
}

TEST(AC_Geometric_LoiterReference, ExternalVelocityConstraintRewritesConsistentPva)
{
    AC_Geometric_LoiterReference reference;
    ASSERT_TRUE(reference.reset(state_at_origin(), 0.0f));
    const auto limits = default_limits();
    AC_Geometric_LoiterReference_Input input;
    input.pilot_accel_ne_mss.x = limits.accel_xy_max_mss;
    input.pilot_xy_active = true;
    AC_Geometric_Target target;
    AC_Geometric_LoiterReference_Status status;
    ASSERT_TRUE(update_reference(reference, input, limits, target, status));
    ASSERT_GT(target.velocity_ned_ms.x, 0.0f);

    const Vector3f constrained_velocity;
    ASSERT_TRUE(reference.apply_velocity_constraint(constrained_velocity, target, status));
    EXPECT_TRUE(status.velocity_constraint_applied);
    EXPECT_NEAR(target.position_ned_m.length(), 0.0f, 1.0e-6f);
    EXPECT_NEAR(target.velocity_ned_ms.length(), 0.0f, 1.0e-6f);
    EXPECT_NEAR(target.accel_ned_mss.length(), 0.0f, 1.0e-6f);
    EXPECT_FALSE(reference.apply_velocity_constraint(constrained_velocity, target, status));
}

TEST(AC_Geometric_LoiterReference, VerticalPositionAnchorPreservesVelocityAndAcceleration)
{
    AC_Geometric_State state = state_at_origin();
    state.position_ned_m.z = -4.0f;
    state.velocity_ned_ms.z = -0.3f;
    AC_Geometric_LoiterReference reference;
    ASSERT_TRUE(reference.reset(state, 0.2f));

    const auto limits = default_limits();
    AC_Geometric_LoiterReference_Input input;
    input.climb_rate_up_ms = 0.5f;
    input.pilot_z_active = true;
    AC_Geometric_Target target;
    AC_Geometric_LoiterReference_Status status;
    ASSERT_TRUE(update_reference(reference, input, limits, target, status));
    const Vector3f position_before = target.position_ned_m;
    const Vector3f velocity_before = target.velocity_ned_ms;
    const Vector3f accel_before = target.accel_ned_mss;

    constexpr postype_t measured_z_ned_m = -6.25;
    ASSERT_TRUE(reference.anchor_position_z(measured_z_ned_m, target, status));
    EXPECT_TRUE(status.vertical_position_anchored);
    EXPECT_NEAR(target.position_ned_m.x, position_before.x, 1.0e-6f);
    EXPECT_NEAR(target.position_ned_m.y, position_before.y, 1.0e-6f);
    EXPECT_NEAR(target.position_ned_m.z, measured_z_ned_m, 1.0e-6f);
    EXPECT_EQ(target.velocity_ned_ms, velocity_before);
    EXPECT_EQ(target.accel_ned_mss, accel_before);

    // A later post-update constraint must retain the anchored position rather
    // than reconstructing Z from the pre-anchor integration base.
    ASSERT_TRUE(reference.apply_velocity_constraint(velocity_before, target, status));
    EXPECT_NEAR(target.position_ned_m.z, measured_z_ned_m, 1.0e-6f);
    EXPECT_EQ(target.velocity_ned_ms, velocity_before);
    EXPECT_EQ(target.accel_ned_mss, accel_before);

    const AC_Geometric_Target target_before_invalid = target;
    EXPECT_FALSE(reference.anchor_position_z(std::numeric_limits<postype_t>::quiet_NaN(), target, status));
    EXPECT_EQ(target.position_ned_m, target_before_invalid.position_ned_m);
    EXPECT_EQ(target.velocity_ned_ms, target_before_invalid.velocity_ned_ms);
    EXPECT_EQ(target.accel_ned_mss, target_before_invalid.accel_ned_mss);
}

TEST(AC_Geometric_LoiterReference, EkfResetShiftMovesReferenceAndPendingConstraintBase)
{
    AC_Geometric_State state = state_at_origin();
    state.position_ned_m = Vector3f{1.0f, 2.0f, -3.0f};
    state.velocity_ned_ms = Vector3f{0.5f, -0.2f, 0.1f};
    AC_Geometric_LoiterReference reference;
    ASSERT_TRUE(reference.reset(state, float(M_PI) - 0.2f));

    const auto limits = default_limits();
    AC_Geometric_LoiterReference_Input input;
    AC_Geometric_Target target;
    AC_Geometric_LoiterReference_Status status;
    ASSERT_TRUE(update_reference(reference, input, limits, target, status));
    const Vector3f position_before = target.position_ned_m;
    const Vector3f velocity_before = target.velocity_ned_ms;
    const Vector3f position_delta_ned_m{4.0f, -5.0f, 2.0f};
    constexpr float yaw_delta_rad = 0.5f;

    ASSERT_TRUE(reference.shift_reference(position_delta_ned_m, yaw_delta_rad));
    EXPECT_NEAR(reference.position_ref_ned_m().x, position_before.x + position_delta_ned_m.x, 1.0e-6f);
    EXPECT_NEAR(reference.position_ref_ned_m().y, position_before.y + position_delta_ned_m.y, 1.0e-6f);
    EXPECT_NEAR(reference.position_ref_ned_m().z, position_before.z + position_delta_ned_m.z, 1.0e-6f);
    EXPECT_NEAR(reference.yaw_ref_rad(), wrap_PI(float(M_PI) - 0.2f + yaw_delta_rad), 1.0e-6f);

    // Rewriting from the pending base must include the same EKF translation.
    ASSERT_TRUE(reference.apply_velocity_constraint(velocity_before, target, status));
    EXPECT_NEAR(target.position_ned_m.x, position_before.x + position_delta_ned_m.x, 1.0e-6f);
    EXPECT_NEAR(target.position_ned_m.y, position_before.y + position_delta_ned_m.y, 1.0e-6f);
    EXPECT_NEAR(target.position_ned_m.z, position_before.z + position_delta_ned_m.z, 1.0e-6f);
    EXPECT_NEAR(target.yaw_rad, wrap_PI(float(M_PI) - 0.2f + yaw_delta_rad), 1.0e-6f);

    Vector3f invalid_delta;
    invalid_delta.x = std::numeric_limits<float>::quiet_NaN();
    const Vector3p position_before_invalid = reference.position_ref_ned_m();
    const float yaw_before_invalid = reference.yaw_ref_rad();
    EXPECT_FALSE(reference.shift_reference(invalid_delta, 0.1f));
    EXPECT_EQ(reference.position_ref_ned_m(), position_before_invalid);
    EXPECT_FLOAT_EQ(reference.yaw_ref_rad(), yaw_before_invalid);
}

TEST(AC_Geometric_LoiterReference, OverspeedSameDirectionCommandReturnsMonotonicallyToLimit)
{
    AC_Geometric_State state = state_at_origin();
    state.velocity_ned_ms.x = 3.0f;
    AC_Geometric_LoiterReference reference;
    ASSERT_TRUE(reference.reset(state, 0.0f));

    const auto limits = default_limits();
    AC_Geometric_LoiterReference_Input input;
    input.pilot_accel_ne_mss.x = limits.accel_xy_max_mss;
    input.pilot_xy_active = true;
    AC_Geometric_Target target;
    AC_Geometric_LoiterReference_Status status;

    float previous_speed_ms = state.velocity_ned_ms.xy().length();
    bool saw_limited = false;
    for (uint16_t i = 0; i < 1000; i++) {
        ASSERT_TRUE(update_reference(reference, input, limits, target, status));
        const float speed_ms = target.velocity_ned_ms.xy().length();
        EXPECT_LE(speed_ms, previous_speed_ms + 1.0e-6f);
        EXPECT_LE(target.accel_ned_mss.xy().length(), limits.accel_xy_max_mss + 1.0e-4f);
        if (previous_speed_ms > limits.speed_xy_max_ms + 1.0e-4f) {
            EXPECT_TRUE(status.speed_xy_limited);
        }
        saw_limited |= status.speed_xy_limited;
        previous_speed_ms = speed_ms;
    }
    EXPECT_TRUE(saw_limited);
    EXPECT_NEAR(target.velocity_ned_ms.x, limits.speed_xy_max_ms, 1.0e-3f);
    EXPECT_NEAR(target.velocity_ned_ms.y, 0.0f, 1.0e-6f);
}

TEST(AC_Geometric_LoiterReference, CoordinatedTurnSuppressesNeutralBrake)
{
    AC_Geometric_State state = state_at_origin();
    state.velocity_ned_ms.x = 1.0f;
    AC_Geometric_LoiterReference reference;
    ASSERT_TRUE(reference.reset(state, 0.0f, 0.5f));
    const auto limits = default_limits();
    AC_Geometric_LoiterReference_Input input;
    input.coordinated_turn = true;
    input.pilot_yaw_active = true;
    AC_Geometric_Target target;
    AC_Geometric_LoiterReference_Status status;
    for (uint8_t i = 0; i < 100; i++) {
        ASSERT_TRUE(update_reference(reference, input, limits, target, status));
        EXPECT_FALSE(status.braking);
    }
    EXPECT_GT(target.velocity_ned_ms.y, 0.0f);
}

TEST(AC_Geometric_LoiterReference, ResidualCoordinatedTurnDoesNotSuppressNeutralBrake)
{
    AC_Geometric_State state = state_at_origin();
    state.velocity_ned_ms.x = 1.0f;
    AC_Geometric_LoiterReference reference;
    ASSERT_TRUE(reference.reset(state, 0.0f, 0.5f));
    const auto limits = default_limits();
    AC_Geometric_LoiterReference_Input input;
    input.coordinated_turn = true;
    input.pilot_yaw_active = false;
    AC_Geometric_Target target;
    AC_Geometric_LoiterReference_Status status;

    bool saw_xy_braking = false;
    bool saw_yaw_braking = false;
    for (uint16_t i = 0; i < 400; i++) {
        ASSERT_TRUE(update_reference(reference, input, limits, target, status));
        saw_xy_braking |= status.braking;
        saw_yaw_braking |= status.yaw_braking;
    }
    EXPECT_TRUE(saw_xy_braking);
    EXPECT_TRUE(saw_yaw_braking);
    EXPECT_NEAR(target.velocity_ned_ms.xy().length(), 0.0f, 1.0e-3f);
    EXPECT_NEAR(target.yaw_rate_rads, 0.0f, 1.0e-4f);
}

TEST(AC_Geometric_LoiterReference, ClimbRateUsesNedSignAndVerticalLimits)
{
    AC_Geometric_LoiterReference reference;
    ASSERT_TRUE(reference.reset(state_at_origin(), 0.0f));
    const auto limits = default_limits();
    AC_Geometric_LoiterReference_Input input;
    input.climb_rate_up_ms = 5.0f;
    input.pilot_z_active = true;
    AC_Geometric_Target target;
    AC_Geometric_LoiterReference_Status status;

    for (uint16_t i = 0; i < 400; i++) {
        ASSERT_TRUE(update_reference(reference, input, limits, target, status));
        EXPECT_GE(target.velocity_ned_ms.z, -limits.speed_up_max_ms - 1.0e-4f);
        EXPECT_LE(fabsf(target.accel_ned_mss.z), limits.accel_z_max_mss + 1.0e-4f);
    }
    EXPECT_LT(target.position_ned_m.z, 0.0f);
    EXPECT_LT(target.velocity_ned_ms.z, 0.0f);
}

TEST(AC_Geometric_LoiterReference, VerticalStickReleaseBrakesBothDirectionsWithoutReversal)
{
    const float ned_signs[] {-1.0f, 1.0f};
    for (const float ned_sign : ned_signs) {
        SCOPED_TRACE(ned_sign);
        AC_Geometric_LoiterReference reference;
        ASSERT_TRUE(reference.reset(state_at_origin(), 0.0f));
        const auto limits = default_limits();
        AC_Geometric_LoiterReference_Input input;
        input.climb_rate_up_ms = -ned_sign;
        input.pilot_z_active = true;
        AC_Geometric_Target target;
        AC_Geometric_LoiterReference_Status status;

        for (uint16_t i = 0; i < 250; i++) {
            ASSERT_TRUE(update_reference(reference, input, limits, target, status));
        }
        ASSERT_GT(target.velocity_ned_ms.z * ned_sign, 0.5f);

        input.climb_rate_up_ms = 0.0f;
        input.pilot_z_active = false;
        bool saw_braking = false;
        bool saw_settled = false;
        bool deceleration_started = false;
        float stop_time_s = 0.0f;
        for (uint16_t i = 0; i < 400; i++) {
            const Vector3f previous_position = target.position_ned_m;
            const Vector3f previous_velocity = target.velocity_ned_ms;
            const Vector3f previous_accel = target.accel_ned_mss;
            ASSERT_TRUE(update_reference(reference, input, limits, target, status));

            saw_braking |= status.z_braking;
            EXPECT_GE(target.velocity_ned_ms.z * ned_sign, -1.0e-5f);
            EXPECT_LE(fabsf(target.accel_ned_mss.z),
                      limits.brake_accel_z_max_mss + 1.0e-4f);
            EXPECT_NEAR(target.velocity_ned_ms.z - previous_velocity.z,
                        target.accel_ned_mss.z * dt_s,
                        1.0e-5f);
            EXPECT_NEAR(target.position_ned_m.z - previous_position.z,
                        previous_velocity.z * dt_s +
                            0.5f * target.accel_ned_mss.z * sq(dt_s),
                        1.0e-5f);
            EXPECT_LE(fabsf(target.accel_ned_mss.z - previous_accel.z),
                      limits.brake_jerk_z_max_msss * dt_s + 1.0e-4f);
            deceleration_started |=
                previous_accel.z * previous_velocity.z <= 0.0f;
            if (deceleration_started) {
                EXPECT_LE(fabsf(target.velocity_ned_ms.z),
                          fabsf(previous_velocity.z) + 1.0e-5f);
            }
            if (status.z_settled) {
                saw_settled = true;
                stop_time_s = (i + 1) * dt_s;
                break;
            }
        }
        EXPECT_TRUE(saw_braking);
        EXPECT_TRUE(saw_settled);
        EXPECT_LT(stop_time_s, 3.0f);
        EXPECT_NEAR(target.velocity_ned_ms.z, 0.0f, 1.0e-6f);

        float previous_accel_z_mss = target.accel_ned_mss.z;
        for (uint8_t i = 0; i < 50; i++) {
            ASSERT_TRUE(update_reference(reference, input, limits, target, status));
            EXPECT_FALSE(status.z_braking);
            // Latch the settled state while the stick remains neutral so the
            // decimated DataFlash observer cannot miss the one-frame event.
            EXPECT_TRUE(status.z_settled);
            EXPECT_NEAR(target.velocity_ned_ms.z, 0.0f, 1.0e-6f);
            EXPECT_NEAR(target.accel_ned_mss.z, 0.0f, 1.0e-6f);
            EXPECT_LE(fabsf(target.accel_ned_mss.z - previous_accel_z_mss),
                      limits.brake_jerk_z_max_msss * dt_s + 1.0e-4f);
            previous_accel_z_mss = target.accel_ned_mss.z;
        }
    }
}

TEST(AC_Geometric_LoiterReference, VerticalReverseCommandBrakesBeforeChangingDirection)
{
    AC_Geometric_LoiterReference reference;
    ASSERT_TRUE(reference.reset(state_at_origin(), 0.0f));
    const auto limits = default_limits();
    AC_Geometric_LoiterReference_Input input;
    input.climb_rate_up_ms = 1.0f;
    input.pilot_z_active = true;
    AC_Geometric_Target target;
    AC_Geometric_LoiterReference_Status status;
    for (uint16_t i = 0; i < 250; i++) {
        ASSERT_TRUE(update_reference(reference, input, limits, target, status));
    }
    ASSERT_LT(target.velocity_ned_ms.z, -0.5f);

    input.climb_rate_up_ms = -1.0f;
    bool saw_braking = false;
    bool saw_settled = false;
    for (uint16_t i = 0; i < 400; i++) {
        ASSERT_TRUE(update_reference(reference, input, limits, target, status));
        saw_braking |= status.z_braking;
        EXPECT_LE(target.velocity_ned_ms.z, 1.0e-5f);
        if (status.z_settled) {
            saw_settled = true;
            break;
        }
    }
    EXPECT_TRUE(saw_braking);
    ASSERT_TRUE(saw_settled);
    EXPECT_NEAR(target.velocity_ned_ms.z, 0.0f, 1.0e-6f);

    // Once the one-way brake reaches zero, the still-active reverse command
    // starts a new, normally jerk-shaped trajectory on the following frame.
    ASSERT_TRUE(update_reference(reference, input, limits, target, status));
    EXPECT_FALSE(status.z_braking);
    EXPECT_FALSE(status.z_settled);
    EXPECT_GT(target.velocity_ned_ms.z, 0.0f);
    for (uint16_t i = 0; i < 250; i++) {
        ASSERT_TRUE(update_reference(reference, input, limits, target, status));
    }
    EXPECT_GT(target.velocity_ned_ms.z, 0.5f);
}

TEST(AC_Geometric_LoiterReference, HighRateNearZeroVerticalBrakeRespectsJerk)
{
    constexpr float high_rate_dt_s = 0.0025f;
    AC_Geometric_State state = state_at_origin();
    state.velocity_ned_ms.z = 5.0e-5f;
    AC_Geometric_LoiterReference reference;
    ASSERT_TRUE(reference.reset(state, 0.0f));
    const auto limits = default_limits();
    AC_Geometric_LoiterReference_Input input;
    AC_Geometric_Target target;
    AC_Geometric_LoiterReference_Status status;

    float previous_accel_z_mss = 0.0f;
    bool saw_settled = false;
    for (uint16_t i = 0; i < 400; i++) {
        ASSERT_TRUE(reference.update(input,
                                     limits,
                                     high_rate_dt_s,
                                     target,
                                     status));
        EXPECT_LE(fabsf(target.accel_ned_mss.z - previous_accel_z_mss),
                  limits.brake_jerk_z_max_msss * high_rate_dt_s + 1.0e-4f);
        EXPECT_LE(fabsf(target.accel_ned_mss.z),
                  limits.brake_accel_z_max_mss + 1.0e-4f);
        EXPECT_GE(target.velocity_ned_ms.z, -1.0e-6f);
        previous_accel_z_mss = target.accel_ned_mss.z;
        if (status.z_settled) {
            saw_settled = true;
            break;
        }
    }
    EXPECT_TRUE(saw_settled);
    EXPECT_NEAR(target.velocity_ned_ms.z, 0.0f, 1.0e-6f);
}

TEST(AC_Geometric_LoiterReference, AvoidanceRebaseCannotForceVerticalJerkJump)
{
    constexpr float high_rate_dt_s = 0.0025f;
    AC_Geometric_State state = state_at_origin();
    state.velocity_ned_ms.z = 0.00255f;
    AC_Geometric_LoiterReference reference;
    ASSERT_TRUE(reference.reset(state, 0.0f));
    const auto limits = default_limits();
    AC_Geometric_LoiterReference_Input input;
    input.pilot_z_active = true;
    input.climb_rate_up_ms = -state.velocity_ned_ms.z;
    AC_Geometric_Target target;
    AC_Geometric_LoiterReference_Status status;

    ASSERT_TRUE(reference.update(input,
                                 limits,
                                 high_rate_dt_s,
                                 target,
                                 status));
    Vector3f constrained_velocity_ned_ms = target.velocity_ned_ms;
    constrained_velocity_ned_ms.z = 5.0e-5f;
    ASSERT_TRUE(reference.apply_velocity_constraint(constrained_velocity_ned_ms,
                                                    target,
                                                    status));
    ASSERT_TRUE(status.velocity_constraint_applied);
    EXPECT_FALSE(status.z_settled);
    ASSERT_NEAR(target.accel_ned_mss.z, -1.0f, 1.0e-3f);

    input = {};
    float previous_accel_z_mss = target.accel_ned_mss.z;
    bool saw_settled = false;
    for (uint16_t i = 0; i < 800; i++) {
        ASSERT_TRUE(reference.update(input,
                                     limits,
                                     high_rate_dt_s,
                                     target,
                                     status));
        EXPECT_LE(fabsf(target.accel_ned_mss.z - previous_accel_z_mss),
                  limits.brake_jerk_z_max_msss * high_rate_dt_s + 1.0e-4f);
        EXPECT_LE(fabsf(target.accel_ned_mss.z),
                  limits.brake_accel_z_max_mss + 1.0e-4f);
        previous_accel_z_mss = target.accel_ned_mss.z;
        if (status.z_settled) {
            saw_settled = true;
            break;
        }
    }
    EXPECT_TRUE(saw_settled);
    EXPECT_NEAR(target.velocity_ned_ms.z, 0.0f, 1.0e-6f);
}

TEST(AC_Geometric_LoiterReference, LowAccelHighRateVerticalStopRespectsBrakeLimit)
{
    constexpr float high_rate_dt_s = 0.0005f;
    AC_Geometric_State state = state_at_origin();
    state.velocity_ned_ms.z = 9.0e-5f;
    AC_Geometric_LoiterReference reference;
    ASSERT_TRUE(reference.reset(state, 0.0f));
    auto limits = default_limits();
    limits.brake_accel_z_max_mss = 0.1f;
    AC_Geometric_LoiterReference_Input input;
    AC_Geometric_Target target;
    AC_Geometric_LoiterReference_Status status;

    float previous_accel_z_mss = 0.0f;
    bool saw_settled = false;
    for (uint16_t i = 0; i < 1000; i++) {
        ASSERT_TRUE(reference.update(input,
                                     limits,
                                     high_rate_dt_s,
                                     target,
                                     status));
        EXPECT_LE(fabsf(target.accel_ned_mss.z - previous_accel_z_mss),
                  limits.brake_jerk_z_max_msss * high_rate_dt_s + 1.0e-4f);
        EXPECT_LE(fabsf(target.accel_ned_mss.z),
                  limits.brake_accel_z_max_mss + 1.0e-4f);
        EXPECT_GE(target.velocity_ned_ms.z, -1.0e-6f);
        previous_accel_z_mss = target.accel_ned_mss.z;
        if (status.z_settled) {
            saw_settled = true;
            break;
        }
    }
    EXPECT_TRUE(saw_settled);
    EXPECT_NEAR(target.velocity_ned_ms.z, 0.0f, 1.0e-6f);
}

TEST(AC_Geometric_LoiterReference, PassThroughConstraintPreservesVerticalStopBoundary)
{
    AC_Geometric_LoiterReference reference;
    ASSERT_TRUE(reference.reset(state_at_origin(), 0.0f));
    const auto limits = default_limits();
    AC_Geometric_LoiterReference_Input input;
    input.climb_rate_up_ms = 1.0f;
    input.pilot_z_active = true;
    AC_Geometric_Target target;
    AC_Geometric_LoiterReference_Status status;

    for (uint16_t i = 0; i < 250; i++) {
        ASSERT_TRUE(update_reference(reference, input, limits, target, status));
        const Vector3f passthrough_velocity_ned_ms = target.velocity_ned_ms;
        ASSERT_TRUE(reference.apply_velocity_constraint(passthrough_velocity_ned_ms,
                                                        target,
                                                        status));
        EXPECT_FALSE(status.velocity_constraint_applied);
    }
    ASSERT_LT(target.velocity_ned_ms.z, -0.5f);

    input = {};
    bool saw_settled = false;
    for (uint16_t i = 0; i < 400; i++) {
        ASSERT_TRUE(update_reference(reference, input, limits, target, status));
        const Vector3f passthrough_velocity_ned_ms = target.velocity_ned_ms;
        ASSERT_TRUE(reference.apply_velocity_constraint(passthrough_velocity_ned_ms,
                                                        target,
                                                        status));
        EXPECT_FALSE(status.velocity_constraint_applied);
        if (status.z_settled) {
            saw_settled = true;
            break;
        }
    }
    ASSERT_TRUE(saw_settled);
    EXPECT_NEAR(target.velocity_ned_ms.z, 0.0f, 1.0e-6f);

    for (uint8_t i = 0; i < 100; i++) {
        ASSERT_TRUE(update_reference(reference, input, limits, target, status));
        const Vector3f passthrough_velocity_ned_ms = target.velocity_ned_ms;
        ASSERT_TRUE(reference.apply_velocity_constraint(passthrough_velocity_ned_ms,
                                                        target,
                                                        status));
        EXPECT_FALSE(status.velocity_constraint_applied);
        EXPECT_TRUE(status.z_settled);
        EXPECT_NEAR(target.velocity_ned_ms.z, 0.0f, 1.0e-6f);
        EXPECT_NEAR(target.accel_ned_mss.z, 0.0f, 1.0e-6f);
    }
}

TEST(AC_Geometric_LoiterReference, TakeoffPositionTargetStopsVerticalReference)
{
    AC_Geometric_LoiterReference reference;
    ASSERT_TRUE(reference.reset(state_at_origin(), 0.0f));
    const auto limits = default_limits();
    AC_Geometric_LoiterReference_Input input;
    // The absolute takeoff target remains authoritative with a centred stick;
    // a neutral pilot input must not select the velocity-brake branch.
    input.climb_rate_up_ms = 0.0f;
    input.use_z_position_target = true;
    input.position_z_target_ned_m = -1.0f;
    AC_Geometric_Target target;
    AC_Geometric_LoiterReference_Status status;

    for (uint16_t i = 0; i < 1000; i++) {
        ASSERT_TRUE(update_reference(reference, input, limits, target, status));
    }
    EXPECT_NEAR(target.position_ned_m.z, -1.0f, 0.05f);
    EXPECT_NEAR(target.velocity_ned_ms.z, 0.0f, 0.05f);
}

TEST(AC_Geometric_LoiterReference, YawRateIsShapedAndWraps)
{
    AC_Geometric_LoiterReference reference;
    ASSERT_TRUE(reference.reset(state_at_origin(), float(M_PI) - 0.01f));
    const auto limits = default_limits();
    AC_Geometric_LoiterReference_Input input;
    input.yaw_rate_rads = 5.0f;
    input.pilot_yaw_active = true;
    AC_Geometric_Target target;
    AC_Geometric_LoiterReference_Status status;

    for (uint16_t i = 0; i < 500; i++) {
        ASSERT_TRUE(update_reference(reference, input, limits, target, status));
        EXPECT_LE(fabsf(target.yaw_rate_rads), limits.yaw_rate_max_rads + 1.0e-4f);
        EXPECT_GE(target.yaw_rad, -float(M_PI));
        EXPECT_LE(target.yaw_rad, float(M_PI));
    }
    EXPECT_GT(target.yaw_rate_rads, 0.9f);
    EXPECT_FALSE(target.shape_position_target);
    EXPECT_FALSE(target.shape_yaw_target);
}

TEST(AC_Geometric_LoiterReference, YawReleaseUsesDedicatedMonotonicBrake)
{
    const auto limits = default_limits();

    const float directions[] {1.0f, -1.0f};
    for (const float direction : directions) {
        AC_Geometric_LoiterReference reference;
        ASSERT_TRUE(reference.reset(state_at_origin(), direction * (float(M_PI) - 0.02f)));
        AC_Geometric_LoiterReference_Input input;
        input.yaw_rate_rads = direction * limits.yaw_rate_max_rads;
        input.pilot_yaw_active = true;
        AC_Geometric_Target target;
        AC_Geometric_LoiterReference_Status status;

        // Release while the command shaper is still accelerating in the
        // direction of rotation.  The first brake frame must not allow the
        // residual acceleration to increase the yaw rate.
        for (uint8_t i = 0; i < 40; i++) {
            ASSERT_TRUE(update_reference(reference, input, limits, target, status));
        }
        ASSERT_GT(direction * target.yaw_rate_rads, 0.05f);
        ASSERT_GT(direction * target.omega_dot_body_radss.z, 0.0f);

        input.yaw_rate_rads = 0.0f;
        input.pilot_yaw_active = false;
        bool saw_braking = false;
        bool saw_settled = false;
        float previous_rate_rads = target.yaw_rate_rads;
        float previous_yaw_rad = target.yaw_rad;
        float previous_brake_accel_radss = 0.0f;
        for (uint16_t i = 0; i < 500; i++) {
            ASSERT_TRUE(update_reference(reference, input, limits, target, status));
            const float accel_radss = target.omega_dot_body_radss.z;

            if (i == 0) {
                EXPECT_NEAR(accel_radss,
                            -direction * limits.yaw_brake_jerk_max_radsss * dt_s,
                            1.0e-5f);
            } else if (!status.yaw_settled) {
                EXPECT_LE(fabsf(accel_radss - previous_brake_accel_radss),
                          limits.yaw_brake_jerk_max_radsss * dt_s + 1.0e-5f);
            }
            EXPECT_LE(fabsf(target.yaw_rate_rads), fabsf(previous_rate_rads) + 1.0e-6f);
            EXPECT_GE(target.yaw_rate_rads * previous_rate_rads, -1.0e-7f);
            EXPECT_LE(fabsf(accel_radss), limits.yaw_brake_accel_max_radss + 1.0e-4f);
            EXPECT_NEAR(target.yaw_rate_rads - previous_rate_rads,
                        accel_radss * dt_s,
                        1.0e-6f);
            EXPECT_NEAR(wrap_PI(target.yaw_rad - previous_yaw_rad),
                        previous_rate_rads * dt_s + 0.5f * accel_radss * sq(dt_s),
                        1.0e-6f);

            saw_braking |= status.yaw_braking;
            saw_settled |= status.yaw_settled;
            previous_rate_rads = target.yaw_rate_rads;
            previous_yaw_rad = target.yaw_rad;
            previous_brake_accel_radss = accel_radss;
            if (status.yaw_settled) {
                break;
            }
        }
        EXPECT_TRUE(saw_braking);
        EXPECT_TRUE(saw_settled);
        EXPECT_NEAR(target.yaw_rate_rads, 0.0f, 1.0e-6f);
    }
}

TEST(AC_Geometric_LoiterReference, InitialYawOverspeedReturnsWithoutRateJump)
{
    AC_Geometric_LoiterReference reference;
    constexpr float initial_yaw_rate_rads = 2.0f;
    ASSERT_TRUE(reference.reset(state_at_origin(), 0.0f, initial_yaw_rate_rads));
    const auto limits = default_limits();
    AC_Geometric_LoiterReference_Input input;
    input.yaw_rate_rads = 5.0f;
    input.pilot_yaw_active = true;
    AC_Geometric_Target target;
    AC_Geometric_LoiterReference_Status status;

    float previous_rate_rads = initial_yaw_rate_rads;
    float previous_accel_radss = 0.0f;
    bool saw_limited = false;
    for (uint16_t i = 0; i < 1000; i++) {
        ASSERT_TRUE(update_reference(reference, input, limits, target, status));
        const float accel_radss = target.omega_dot_body_radss.z;
        EXPECT_LE(target.yaw_rate_rads, previous_rate_rads + 1.0e-6f);
        EXPECT_LE(fabsf(target.yaw_rate_rads - previous_rate_rads),
                  limits.yaw_accel_max_radss * dt_s + 1.0e-6f);
        EXPECT_LE(fabsf(accel_radss), limits.yaw_accel_max_radss + 1.0e-4f);
        EXPECT_LE(fabsf(accel_radss - previous_accel_radss),
                  limits.yaw_jerk_max_radsss * dt_s + 1.0e-4f);
        if (previous_rate_rads > limits.yaw_rate_max_rads + 1.0e-4f) {
            EXPECT_TRUE(status.yaw_rate_limited);
        }
        saw_limited |= status.yaw_rate_limited;
        previous_rate_rads = target.yaw_rate_rads;
        previous_accel_radss = accel_radss;
    }
    EXPECT_TRUE(saw_limited);
    EXPECT_NEAR(target.yaw_rate_rads, limits.yaw_rate_max_rads, 1.0e-3f);
}

AP_GTEST_MAIN()
