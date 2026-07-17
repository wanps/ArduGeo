#pragma once

#include <AP_Math/AP_Math.h>

// Owns Guided command semantics before setpoint shaping and geometric
// feedback. It separates waypoint-style horizontal destination updates from
// full 3D position-target updates so vehicle code does not need to infer
// altitude intent from the measured vehicle state.
class AC_Geometric_GuidedTargetManager {
public:
    AC_Geometric_GuidedTargetManager() = default;

    enum class TargetType : uint8_t {
        None = 0,
        Position = 1,
        FirstDestination = 2,
        HorizontalDestination = 3,
        VerticalDestination = 4,
        HorizontalVerticalDestination = 5,
        TerrainDestination = 6,
        SmallDestination = 7,
        PositionAltitudeMerge = 8,
    };

    void reset();

    // Full 3D position command: update horizontal and vertical targets.
    const Vector3p& set_position_target(const Vector3p& position_ned_m, bool is_terrain_alt);
    const Vector3p& set_position_target(const Vector3p& position_ned_m, bool is_terrain_alt, const Vector3p* current_pos_ned_m);

    // Waypoint-style Guided destination: update XY, but hold the previous
    // geometric altitude target when this is clearly a horizontal move.
    const Vector3p& set_destination_target(const Vector3p& position_ned_m, bool is_terrain_alt);

    bool target_valid() const { return _target_valid; }
    const Vector3p& position_target_ned_m() const { return _position_target_ned_m; }
    bool is_terrain_alt() const { return _is_terrain_alt; }
    TargetType target_type() const { return _target_type; }
    // True when the latest Guided target represents a horizontal trajectory
    // segment whose shaped XY velocity may drive geometric yaw-follow.
    bool trajectory_yaw_allowed() const { return _trajectory_yaw_allowed; }

private:
    static constexpr float horizontal_destination_min_m = 0.50f;
    static constexpr float vertical_destination_min_m = 0.50f;
    static constexpr float trajectory_yaw_destination_min_m = 2.00f;
    static constexpr float altitude_merge_current_xy_max_m = 1.00f;

    bool is_horizontal_destination(const Vector3p& position_ned_m, bool is_terrain_alt) const;
    bool is_vertical_destination(const Vector3p& position_ned_m, bool is_terrain_alt) const;
    bool is_trajectory_yaw_destination(const Vector3p& position_ned_m, bool is_terrain_alt) const;
    bool is_destination_target_type() const;
    bool should_merge_altitude_for_position_target(const Vector3p& position_ned_m, bool is_terrain_alt, const Vector3p* current_pos_ned_m) const;
    bool should_hold_altitude_for_destination(const Vector3p& position_ned_m, bool is_terrain_alt) const;

    Vector3p _position_target_ned_m;
    bool _is_terrain_alt = false;
    bool _target_valid = false;
    bool _trajectory_yaw_allowed = false;
    TargetType _target_type = TargetType::None;
};
