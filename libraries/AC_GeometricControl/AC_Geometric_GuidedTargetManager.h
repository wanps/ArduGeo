#pragma once

#include <AP_Math/AP_Math.h>

// Owns the geometric Guided target semantics before setpoint shaping.  It
// separates waypoint-style horizontal destination updates from full 3D
// position-target updates so vehicle code does not need to infer altitude
// intent from the measured vehicle state.
class AC_Geometric_GuidedTargetManager {
public:
    AC_Geometric_GuidedTargetManager() = default;

    void reset();

    // Full 3D position command: update horizontal and vertical targets.
    const Vector3p& set_position_target(const Vector3p& position_ned_m, bool is_terrain_alt);

    // Waypoint-style Guided destination: update XY, but hold the previous
    // geometric altitude target when this is clearly a horizontal move.
    const Vector3p& set_destination_target(const Vector3p& position_ned_m, bool is_terrain_alt);

    bool target_valid() const { return _target_valid; }
    const Vector3p& position_target_ned_m() const { return _position_target_ned_m; }
    bool is_terrain_alt() const { return _is_terrain_alt; }

private:
    static constexpr float horizontal_destination_min_m = 0.50f;

    bool should_hold_altitude_for_destination(const Vector3p& position_ned_m, bool is_terrain_alt) const;

    Vector3p _position_target_ned_m;
    bool _is_terrain_alt = false;
    bool _target_valid = false;
};
