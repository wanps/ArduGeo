#include <AP_gtest.h>

#include <AC_GeometricControl/AC_Geometric_GuidedTargetManager.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(AC_Geometric_GuidedTargetManager, FirstDestinationAcceptsFullPosition)
{
    AC_Geometric_GuidedTargetManager manager;

    const Vector3p target{1.0, 2.0, -5.0};
    const Vector3p& stored = manager.set_destination_target(target, false);

    EXPECT_TRUE(manager.target_valid());
    EXPECT_FALSE(manager.is_terrain_alt());
    EXPECT_DOUBLE_EQ(stored.x, target.x);
    EXPECT_DOUBLE_EQ(stored.y, target.y);
    EXPECT_DOUBLE_EQ(stored.z, target.z);
}

TEST(AC_Geometric_GuidedTargetManager, HorizontalDestinationHoldsAltitude)
{
    AC_Geometric_GuidedTargetManager manager;

    manager.set_position_target(Vector3p{0.0, 0.0, -6.0}, false);
    const Vector3p& stored = manager.set_destination_target(Vector3p{2.0, 0.0, -5.4}, false);

    EXPECT_DOUBLE_EQ(stored.x, 2.0);
    EXPECT_DOUBLE_EQ(stored.y, 0.0);
    EXPECT_DOUBLE_EQ(stored.z, -6.0);
}

TEST(AC_Geometric_GuidedTargetManager, SmallHorizontalChangeAllowsAltitudeUpdate)
{
    AC_Geometric_GuidedTargetManager manager;

    manager.set_position_target(Vector3p{1.0, 1.0, -6.0}, false);
    const Vector3p& stored = manager.set_destination_target(Vector3p{1.1, 1.1, -4.0}, false);

    EXPECT_DOUBLE_EQ(stored.x, 1.1);
    EXPECT_DOUBLE_EQ(stored.y, 1.1);
    EXPECT_DOUBLE_EQ(stored.z, -4.0);
}

TEST(AC_Geometric_GuidedTargetManager, TerrainFrameDoesNotHoldOriginAltitude)
{
    AC_Geometric_GuidedTargetManager manager;

    manager.set_position_target(Vector3p{0.0, 0.0, -6.0}, false);
    const Vector3p& stored = manager.set_destination_target(Vector3p{2.0, 0.0, -3.0}, true);

    EXPECT_TRUE(manager.is_terrain_alt());
    EXPECT_DOUBLE_EQ(stored.x, 2.0);
    EXPECT_DOUBLE_EQ(stored.y, 0.0);
    EXPECT_DOUBLE_EQ(stored.z, -3.0);
}

TEST(AC_Geometric_GuidedTargetManager, FullPositionTargetAlwaysUpdatesAltitude)
{
    AC_Geometric_GuidedTargetManager manager;

    manager.set_position_target(Vector3p{0.0, 0.0, -6.0}, false);
    const Vector3p& stored = manager.set_position_target(Vector3p{2.0, 0.0, -4.0}, false);

    EXPECT_DOUBLE_EQ(stored.x, 2.0);
    EXPECT_DOUBLE_EQ(stored.y, 0.0);
    EXPECT_DOUBLE_EQ(stored.z, -4.0);
}

AP_GTEST_MAIN()
