#include <AP_gtest.h>

#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <AP_AHRS/AP_AHRS.h>

#include <limits>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

class TestMotors final : public AP_Motors {
public:
    float get_throttle_hover() const override { return 0.5f; }

    void set_desired_spool_state(DesiredSpoolState spool) override { _spool_desired = spool; }
    void init(motor_frame_class, motor_frame_type) override {}
    void set_frame_class_and_type(motor_frame_class, motor_frame_type) override {}
    void output() override {}
    void output_min() override {}
    uint32_t get_motor_mask() override { return 0; }

protected:
    void output_armed_stabilizing() override {}
    void update_throttle_filter() override {}
    const char* _get_frame_string() const override { return "TEST"; }
    void _output_test_seq(uint8_t, int16_t) override {}
};

class TestAttitudeControl final : public AC_AttitudeControl {
public:
    TestAttitudeControl(AP_AHRS_View& ahrs, AP_Motors& motors) :
        AC_AttitudeControl(ahrs, motors),
        _pid_roll(0.1f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f),
        _pid_pitch(0.1f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f),
        _pid_yaw(0.1f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f)
    {}

    AC_PID& get_rate_roll_pid() override { return _pid_roll; }
    AC_PID& get_rate_pitch_pid() override { return _pid_pitch; }
    AC_PID& get_rate_yaw_pid() override { return _pid_yaw; }
    const AC_PID& get_rate_roll_pid() const override { return _pid_roll; }
    const AC_PID& get_rate_pitch_pid() const override { return _pid_pitch; }
    const AC_PID& get_rate_yaw_pid() const override { return _pid_yaw; }

    void rate_controller_run() override { rate_controller_runs++; }
    void update_althold_lean_angle_max(float) override {}
    void set_throttle_out(float, bool, float) override { throttle_writes++; }

    const Quaternion& attitude_error_quat() const { return _attitude_ang_error; }

    uint32_t rate_controller_runs = 0;
    uint32_t throttle_writes = 0;

private:
    AC_PID _pid_roll;
    AC_PID _pid_pitch;
    AC_PID _pid_yaw;
};

AP_AHRS ahrs{AP_AHRS::FLAG_ALWAYS_USE_EKF};
AP_AHRS_View ahrs_view{ahrs, ROTATION_NONE};
TestMotors motors;
TestAttitudeControl attitude_control{ahrs_view, motors};

void expect_quaternion_near(const Quaternion& actual, const Quaternion& expected, float tolerance)
{
    // q and -q represent the same attitude.  Compare using the sign that puts
    // the two quaternions in the same hemisphere.
    const float dot = actual.q1 * expected.q1 +
                      actual.q2 * expected.q2 +
                      actual.q3 * expected.q3 +
                      actual.q4 * expected.q4;
    const float sign = dot < 0.0f ? -1.0f : 1.0f;
    EXPECT_NEAR(actual.q1, sign * expected.q1, tolerance);
    EXPECT_NEAR(actual.q2, sign * expected.q2, tolerance);
    EXPECT_NEAR(actual.q3, sign * expected.q3, tolerance);
    EXPECT_NEAR(actual.q4, sign * expected.q4, tolerance);
}

TEST(ExternalAttitudeTarget, PublishesCompleteBookkeepingWithoutRunningControllers)
{
    constexpr float roll_rad = 0.20f;
    constexpr float pitch_rad = -0.10f;
    constexpr float yaw_rad = 0.35f;
    const Vector3f body_rate_rads{0.12f, -0.08f, 0.25f};

    Quaternion target;
    target.from_euler(roll_rad, pitch_rad, yaw_rad);

    EXPECT_TRUE(attitude_control.set_external_attitude_target(target, body_rate_rads));

    expect_quaternion_near(attitude_control.get_attitude_target_quat(), target, 1.0e-6f);
    EXPECT_NEAR(attitude_control.get_att_target_euler_rad().x, roll_rad, 1.0e-6f);
    EXPECT_NEAR(attitude_control.get_att_target_euler_rad().y, pitch_rad, 1.0e-6f);
    EXPECT_NEAR(attitude_control.get_att_target_euler_rad().z, yaw_rad, 1.0e-6f);
    EXPECT_NEAR(attitude_control.get_attitude_target_ang_vel().x, body_rate_rads.x, 1.0e-6f);
    EXPECT_NEAR(attitude_control.get_attitude_target_ang_vel().y, body_rate_rads.y, 1.0e-6f);
    EXPECT_NEAR(attitude_control.get_attitude_target_ang_vel().z, body_rate_rads.z, 1.0e-6f);

    const Vector3f expected_euler_rate{
        body_rate_rads.x + sinf(roll_rad) * tanf(pitch_rad) * body_rate_rads.y +
            cosf(roll_rad) * tanf(pitch_rad) * body_rate_rads.z,
        cosf(roll_rad) * body_rate_rads.y - sinf(roll_rad) * body_rate_rads.z,
        sinf(roll_rad) / cosf(pitch_rad) * body_rate_rads.y +
            cosf(roll_rad) / cosf(pitch_rad) * body_rate_rads.z
    };
    EXPECT_NEAR(attitude_control.get_rate_ef_target_rads().x, expected_euler_rate.x, 1.0e-6f);
    EXPECT_NEAR(attitude_control.get_rate_ef_target_rads().y, expected_euler_rate.y, 1.0e-6f);
    EXPECT_NEAR(attitude_control.get_rate_ef_target_rads().z, expected_euler_rate.z, 1.0e-6f);

    // The test AHRS view is identity, so measured-to-target error equals target.
    expect_quaternion_near(attitude_control.attitude_error_quat(), target, 1.0e-6f);
    const float expected_thrust_error_rad = acosf(cosf(roll_rad) * cosf(pitch_rad));
    EXPECT_NEAR(radians(attitude_control.get_att_error_angle_deg()), expected_thrust_error_rad, 1.0e-6f);
    EXPECT_NEAR(attitude_control.lean_angle_rad(), 0.0f, 1.0e-6f);

    EXPECT_EQ(attitude_control.rate_controller_runs, 0U);
    EXPECT_EQ(attitude_control.throttle_writes, 0U);
    EXPECT_FLOAT_EQ(motors.get_roll(), 0.0f);
    EXPECT_FLOAT_EQ(motors.get_pitch(), 0.0f);
    EXPECT_FLOAT_EQ(motors.get_yaw(), 0.0f);
}

TEST(ExternalAttitudeTarget, RejectsInvalidInputsAtomically)
{
    Quaternion valid_target;
    valid_target.from_euler(-0.1f, 0.05f, -0.2f);
    const Vector3f valid_rate{0.01f, 0.02f, 0.03f};
    ASSERT_TRUE(attitude_control.set_external_attitude_target(valid_target, valid_rate));

    const Quaternion target_before = attitude_control.get_attitude_target_quat();
    const Vector3f rate_before = attitude_control.get_attitude_target_ang_vel();
    const Vector3f euler_before = attitude_control.get_att_target_euler_rad();
    const float error_before_deg = attitude_control.get_att_error_angle_deg();

    Quaternion zero_target;
    zero_target.zero();
    EXPECT_FALSE(attitude_control.set_external_attitude_target(zero_target, valid_rate));

    Quaternion nan_target = valid_target;
    nan_target.q2 = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(attitude_control.set_external_attitude_target(nan_target, valid_rate));

    Vector3f infinite_rate = valid_rate;
    infinite_rate.z = std::numeric_limits<float>::infinity();
    EXPECT_FALSE(attitude_control.set_external_attitude_target(valid_target, infinite_rate));

    expect_quaternion_near(attitude_control.get_attitude_target_quat(), target_before, 0.0f);
    EXPECT_EQ(attitude_control.get_attitude_target_ang_vel(), rate_before);
    EXPECT_EQ(attitude_control.get_att_target_euler_rad(), euler_before);
    EXPECT_FLOAT_EQ(attitude_control.get_att_error_angle_deg(), error_before_deg);
    EXPECT_EQ(attitude_control.rate_controller_runs, 0U);
    EXPECT_EQ(attitude_control.throttle_writes, 0U);
}

TEST(ExternalAttitudeTarget, NormalizesFiniteNonzeroQuaternion)
{
    Quaternion target;
    target.from_euler(0.15f, -0.25f, 0.45f);
    Quaternion scaled_target{target.q1 * 3.0f,
                             target.q2 * 3.0f,
                             target.q3 * 3.0f,
                             target.q4 * 3.0f};

    EXPECT_TRUE(attitude_control.set_external_attitude_target(scaled_target, Vector3f{}));
    EXPECT_TRUE(attitude_control.get_attitude_target_quat().is_unit_length());
    expect_quaternion_near(attitude_control.get_attitude_target_quat(), target, 1.0e-6f);
}

} // namespace

AP_GTEST_MAIN()
