#include <AP_gtest.h>

#include <AC_GeometricControl/AC_GeometricControl.h>

#include <limits>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

AC_TrajectoryReference valid_trajectory_reference()
{
    AC_TrajectoryReference reference {};
    reference.meta.capability = AC_ControlReferenceCapability::TRAJECTORY;
    reference.meta.valid = true;
    reference.position_ned_m = Vector3p{1.0, 2.0, 3.0};
    reference.velocity_ned_ms = Vector3f{4.0f, 5.0f, 6.0f};
    reference.acceleration_ned_mss = Vector3f{7.0f, 8.0f, 9.0f};
    reference.heading = {
        0.4f,
        -0.2f,
        AC_AttitudeControl::HeadingMode::Angle_And_Rate
    };
    return reference;
}

AC_AttitudeReference valid_attitude_reference()
{
    AC_AttitudeReference reference {};
    reference.meta.capability = AC_ControlReferenceCapability::ATTITUDE;
    reference.meta.valid = true;
    reference.attitude_body_to_ned.from_euler(0.1f, -0.2f, 0.3f);
    reference.angular_velocity_body_rads = Vector3f{0.4f, 0.5f, 0.6f};
    reference.angular_acceleration_body_radss = Vector3f{0.7f, 0.8f, 0.9f};
    return reference;
}

void expect_quaternion_equal(const Quaternion& actual, const Quaternion& expected)
{
    EXPECT_FLOAT_EQ(actual.q1, expected.q1);
    EXPECT_FLOAT_EQ(actual.q2, expected.q2);
    EXPECT_FLOAT_EQ(actual.q3, expected.q3);
    EXPECT_FLOAT_EQ(actual.q4, expected.q4);
}

void expect_target_equal(const AC_Geometric_Target& actual, const AC_Geometric_Target& expected)
{
    EXPECT_EQ(actual.position_ned_m, expected.position_ned_m);
    EXPECT_EQ(actual.velocity_ned_ms, expected.velocity_ned_ms);
    EXPECT_EQ(actual.accel_ned_mss, expected.accel_ned_mss);
    expect_quaternion_equal(actual.attitude_body_to_ned, expected.attitude_body_to_ned);
    EXPECT_EQ(actual.omega_body_rads, expected.omega_body_rads);
    EXPECT_EQ(actual.omega_dot_body_radss, expected.omega_dot_body_radss);
    EXPECT_EQ(actual.build_attitude_from_position, expected.build_attitude_from_position);
    EXPECT_EQ(actual.shape_position_target, expected.shape_position_target);
    EXPECT_EQ(actual.shape_yaw_target, expected.shape_yaw_target);
    EXPECT_FLOAT_EQ(actual.yaw_rad, expected.yaw_rad);
    EXPECT_FLOAT_EQ(actual.yaw_rate_rads, expected.yaw_rate_rads);
    EXPECT_EQ(actual.yaw_from_trajectory, expected.yaw_from_trajectory);
}

} // namespace

TEST(AC_ControlReference, DefaultsAreInvalid)
{
    const AC_TrajectoryReference trajectory {};
    const AC_AttitudeReference attitude {};

    EXPECT_FALSE(trajectory.meta.valid);
    EXPECT_FALSE(trajectory.is_valid());
    EXPECT_FALSE(attitude.meta.valid);
    EXPECT_FALSE(attitude.is_valid());
}

TEST(AC_ControlReference, ValidatesFrameAndCapability)
{
    AC_TrajectoryReference reference = valid_trajectory_reference();
    reference.meta.timestamp_ms = 123U;
    reference.meta.sequence = 456U;
    EXPECT_TRUE(reference.is_valid());
    EXPECT_EQ(reference.meta.timestamp_ms, 123U);
    EXPECT_EQ(reference.meta.sequence, 456U);

    reference.meta.valid = false;
    EXPECT_FALSE(reference.is_valid());

    reference = valid_trajectory_reference();
    reference.meta.frame = static_cast<AC_ControlReferenceFrame>(1);
    EXPECT_FALSE(reference.is_valid());

    reference = valid_trajectory_reference();
    reference.meta.capability = AC_ControlReferenceCapability::ATTITUDE;
    EXPECT_FALSE(reference.is_valid());

    AC_AttitudeReference attitude = valid_attitude_reference();
    EXPECT_TRUE(attitude.is_valid());
    attitude.meta.capability = AC_ControlReferenceCapability::RATE;
    EXPECT_FALSE(attitude.is_valid());
}

TEST(AC_ControlReference, TimestampFreshnessHandlesWrap)
{
    AC_ControlReferenceMeta meta {};
    meta.valid = true;
    meta.timestamp_ms = 100U;

    EXPECT_TRUE(meta.is_fresh(110U, 10U));
    EXPECT_FALSE(meta.is_fresh(111U, 10U));
    meta.valid = false;
    EXPECT_FALSE(meta.is_fresh(100U, UINT32_MAX));

    meta.valid = true;
    meta.timestamp_ms = UINT32_MAX - 5U;
    EXPECT_TRUE(meta.is_fresh(4U, 10U));
    EXPECT_FALSE(meta.is_fresh(4U, 9U));
}

TEST(AC_ControlReference, TrajectoryRejectsNonfiniteAndInvalidHeading)
{
    AC_TrajectoryReference reference = valid_trajectory_reference();

    reference.position_ned_m.x = std::numeric_limits<postype_t>::quiet_NaN();
    EXPECT_FALSE(reference.is_valid());
    reference = valid_trajectory_reference();
    reference.velocity_ned_ms.y = std::numeric_limits<float>::infinity();
    EXPECT_FALSE(reference.is_valid());
    reference = valid_trajectory_reference();
    reference.acceleration_ned_mss.z = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(reference.is_valid());
    reference = valid_trajectory_reference();
    reference.heading.yaw_angle_rad = std::numeric_limits<float>::infinity();
    EXPECT_FALSE(reference.is_valid());
    reference = valid_trajectory_reference();
    reference.heading.yaw_rate_rads = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(reference.is_valid());
    reference = valid_trajectory_reference();
    reference.heading.heading_mode = static_cast<AC_AttitudeControl::HeadingMode>(3);
    EXPECT_FALSE(reference.is_valid());
}

TEST(AC_ControlReference, TrajectoryAdapterAcceptsAngleOnlyWithZeroRate)
{
    AC_TrajectoryReference reference = valid_trajectory_reference();
    reference.heading.heading_mode = AC_AttitudeControl::HeadingMode::Angle_Only;
    AC_Geometric_Target target {};

    ASSERT_TRUE(AC_GeometricControl::reference_to_target(reference, target));
    EXPECT_FLOAT_EQ(target.yaw_rad, reference.heading.yaw_angle_rad);
    EXPECT_FLOAT_EQ(target.yaw_rate_rads, 0.0f);
    EXPECT_FLOAT_EQ(target.omega_body_rads.z, 0.0f);
}

TEST(AC_ControlReference, AttitudeRejectsNonfiniteAndInvalidQuaternion)
{
    AC_AttitudeReference reference = valid_attitude_reference();
    EXPECT_TRUE(reference.is_valid());

    reference.attitude_body_to_ned.zero();
    EXPECT_FALSE(reference.is_valid());
    reference = valid_attitude_reference();
    reference.attitude_body_to_ned.q2 = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(reference.is_valid());
    reference = valid_attitude_reference();
    reference.attitude_body_to_ned.q1 *= 2.0f;
    EXPECT_FALSE(reference.is_valid());
    reference = valid_attitude_reference();
    reference.angular_velocity_body_rads.x = std::numeric_limits<float>::infinity();
    EXPECT_FALSE(reference.is_valid());
    reference = valid_attitude_reference();
    reference.angular_acceleration_body_radss.y = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(reference.is_valid());
}

TEST(AC_ControlReference, TrajectoryAdapterPreservesAngleAndRate)
{
    const AC_TrajectoryReference reference = valid_trajectory_reference();
    AC_Geometric_Target target {};

    ASSERT_TRUE(AC_GeometricControl::reference_to_target(reference, target));
    EXPECT_EQ(target.position_ned_m, reference.position_ned_m.tofloat());
    EXPECT_EQ(target.velocity_ned_ms, reference.velocity_ned_ms);
    EXPECT_EQ(target.accel_ned_mss, reference.acceleration_ned_mss);
    EXPECT_TRUE(target.build_attitude_from_position);
    EXPECT_FALSE(target.shape_position_target);
    EXPECT_FALSE(target.shape_yaw_target);
    EXPECT_FLOAT_EQ(target.yaw_rad, reference.heading.yaw_angle_rad);
    EXPECT_FLOAT_EQ(target.yaw_rate_rads, reference.heading.yaw_rate_rads);
    EXPECT_FLOAT_EQ(target.omega_body_rads.z, reference.heading.yaw_rate_rads);
    EXPECT_FALSE(target.yaw_from_trajectory);
}

TEST(AC_ControlReference, TrajectoryAdapterRejectsRateOnlyAtomically)
{
    AC_TrajectoryReference reference = valid_trajectory_reference();
    AC_Geometric_Target target {};
    ASSERT_TRUE(AC_GeometricControl::reference_to_target(reference, target));
    const AC_Geometric_Target expected = target;

    reference.heading.heading_mode = AC_AttitudeControl::HeadingMode::Rate_Only;
    EXPECT_TRUE(reference.is_valid());
    EXPECT_FALSE(AC_GeometricControl::reference_to_target(reference, target));
    expect_target_equal(target, expected);
}

TEST(AC_ControlReference, AttitudeAdapterPreservesFields)
{
    const AC_AttitudeReference reference = valid_attitude_reference();
    AC_Geometric_Target target {};

    ASSERT_TRUE(AC_GeometricControl::reference_to_target(reference, target));
    expect_quaternion_equal(target.attitude_body_to_ned, reference.attitude_body_to_ned);
    EXPECT_EQ(target.omega_body_rads, reference.angular_velocity_body_rads);
    EXPECT_EQ(target.omega_dot_body_radss, reference.angular_acceleration_body_radss);
    EXPECT_FALSE(target.build_attitude_from_position);
    EXPECT_FALSE(target.shape_position_target);
    EXPECT_FALSE(target.shape_yaw_target);
}

TEST(AC_ControlReference, CompositeAdapterPreservesTrajectoryPolicy)
{
    const AC_TrajectoryReference trajectory = valid_trajectory_reference();
    const AC_AttitudeReference attitude = valid_attitude_reference();
    const AC_GeometricReferencePolicy policy {
        true,
        true,
        true,
        true
    };
    AC_Geometric_Target target {};

    ASSERT_TRUE(AC_GeometricControl::references_to_target(trajectory,
                                                          &attitude,
                                                          policy,
                                                          target));
    EXPECT_EQ(target.position_ned_m, trajectory.position_ned_m.tofloat());
    EXPECT_EQ(target.velocity_ned_ms, trajectory.velocity_ned_ms);
    EXPECT_EQ(target.accel_ned_mss, trajectory.acceleration_ned_mss);
    EXPECT_EQ(target.omega_body_rads, attitude.angular_velocity_body_rads);
    EXPECT_EQ(target.omega_dot_body_radss, attitude.angular_acceleration_body_radss);
    EXPECT_TRUE(target.build_attitude_from_position);
    EXPECT_TRUE(target.shape_position_target);
    EXPECT_TRUE(target.shape_yaw_target);
    EXPECT_TRUE(target.yaw_from_trajectory);
    EXPECT_FLOAT_EQ(target.yaw_rad, trajectory.heading.yaw_angle_rad);
    EXPECT_FLOAT_EQ(target.yaw_rate_rads, trajectory.heading.yaw_rate_rads);
}

TEST(AC_ControlReference, CompositeAdapterPreservesDirectAttitude)
{
    const AC_TrajectoryReference trajectory = valid_trajectory_reference();
    const AC_AttitudeReference attitude = valid_attitude_reference();
    const AC_GeometricReferencePolicy policy {
        false,
        false,
        false,
        false
    };
    AC_Geometric_Target target {};

    ASSERT_TRUE(AC_GeometricControl::references_to_target(trajectory,
                                                          &attitude,
                                                          policy,
                                                          target));
    EXPECT_EQ(target.position_ned_m, trajectory.position_ned_m.tofloat());
    EXPECT_EQ(target.velocity_ned_ms, trajectory.velocity_ned_ms);
    EXPECT_EQ(target.accel_ned_mss, trajectory.acceleration_ned_mss);
    expect_quaternion_equal(target.attitude_body_to_ned,
                            attitude.attitude_body_to_ned);
    EXPECT_EQ(target.omega_body_rads, attitude.angular_velocity_body_rads);
    EXPECT_EQ(target.omega_dot_body_radss, attitude.angular_acceleration_body_radss);
    EXPECT_FALSE(target.build_attitude_from_position);
}

TEST(AC_ControlReference, CompositeAdapterFailsAtomically)
{
    AC_Geometric_Target target {};
    target.position_ned_m = Vector3f{1.0f, 2.0f, 3.0f};
    target.yaw_rad = 1.0f;
    const AC_Geometric_Target expected = target;
    AC_TrajectoryReference trajectory = valid_trajectory_reference();
    AC_AttitudeReference attitude = valid_attitude_reference();
    const AC_GeometricReferencePolicy direct_attitude_policy {
        false,
        false,
        false,
        false
    };

    trajectory.heading.heading_mode = AC_AttitudeControl::HeadingMode::Rate_Only;
    EXPECT_FALSE(AC_GeometricControl::references_to_target(trajectory,
                                                           &attitude,
                                                           direct_attitude_policy,
                                                           target));
    expect_target_equal(target, expected);

    trajectory = valid_trajectory_reference();
    EXPECT_FALSE(AC_GeometricControl::references_to_target(trajectory,
                                                           nullptr,
                                                           direct_attitude_policy,
                                                           target));
    expect_target_equal(target, expected);

    attitude.angular_velocity_body_rads.x = std::numeric_limits<float>::infinity();
    EXPECT_FALSE(AC_GeometricControl::references_to_target(trajectory,
                                                           &attitude,
                                                           direct_attitude_policy,
                                                           target));
    expect_target_equal(target, expected);
}

TEST(AC_ControlReference, AdapterFailureDoesNotModifyTarget)
{
    AC_Geometric_Target target {};
    target.position_ned_m = Vector3f{1.0f, 2.0f, 3.0f};
    target.velocity_ned_ms = Vector3f{4.0f, 5.0f, 6.0f};
    target.accel_ned_mss = Vector3f{7.0f, 8.0f, 9.0f};
    target.attitude_body_to_ned.from_euler(0.1f, 0.2f, 0.3f);
    target.omega_body_rads = Vector3f{0.4f, 0.5f, 0.6f};
    target.omega_dot_body_radss = Vector3f{0.7f, 0.8f, 0.9f};
    target.build_attitude_from_position = true;
    target.shape_position_target = false;
    target.yaw_rad = 1.0f;
    target.yaw_rate_rads = -1.0f;
    target.yaw_from_trajectory = true;
    const AC_Geometric_Target expected = target;

    AC_TrajectoryReference trajectory = valid_trajectory_reference();
    trajectory.meta.frame = static_cast<AC_ControlReferenceFrame>(1);
    EXPECT_FALSE(AC_GeometricControl::reference_to_target(trajectory, target));
    expect_target_equal(target, expected);

    trajectory = valid_trajectory_reference();
    trajectory.meta.capability = AC_ControlReferenceCapability::ATTITUDE;
    EXPECT_FALSE(AC_GeometricControl::reference_to_target(trajectory, target));
    expect_target_equal(target, expected);

    trajectory = valid_trajectory_reference();
    trajectory.velocity_ned_ms.x = std::numeric_limits<float>::infinity();
    EXPECT_FALSE(AC_GeometricControl::reference_to_target(trajectory, target));
    expect_target_equal(target, expected);

#if HAL_WITH_POSTYPE_DOUBLE
    trajectory = valid_trajectory_reference();
    trajectory.position_ned_m.x = std::numeric_limits<postype_t>::max();
    ASSERT_TRUE(trajectory.is_valid());
    EXPECT_FALSE(AC_GeometricControl::reference_to_target(trajectory, target));
    expect_target_equal(target, expected);
#endif

    AC_AttitudeReference attitude = valid_attitude_reference();
    attitude.attitude_body_to_ned.zero();
    EXPECT_FALSE(AC_GeometricControl::reference_to_target(attitude, target));
    expect_target_equal(target, expected);

    attitude = valid_attitude_reference();
    attitude.meta.capability = AC_ControlReferenceCapability::TRAJECTORY;
    EXPECT_FALSE(AC_GeometricControl::reference_to_target(attitude, target));
    expect_target_equal(target, expected);

    attitude = valid_attitude_reference();
    attitude.angular_acceleration_body_radss.z = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(AC_GeometricControl::reference_to_target(attitude, target));
    expect_target_equal(target, expected);
}

AP_GTEST_MAIN()
