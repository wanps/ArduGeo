#include "AC_Geometric_GuidedTargetManager.h"

void AC_Geometric_GuidedTargetManager::reset()
{
    _position_target_ned_m.zero();
    _is_terrain_alt = false;
    _target_valid = false;
}

const Vector3p& AC_Geometric_GuidedTargetManager::set_position_target(const Vector3p& position_ned_m, bool is_terrain_alt)
{
    _position_target_ned_m = position_ned_m;
    _is_terrain_alt = is_terrain_alt;
    _target_valid = true;
    return _position_target_ned_m;
}

const Vector3p& AC_Geometric_GuidedTargetManager::set_destination_target(const Vector3p& position_ned_m, bool is_terrain_alt)
{
    if (should_hold_altitude_for_destination(position_ned_m, is_terrain_alt)) {
        _position_target_ned_m.x = position_ned_m.x;
        _position_target_ned_m.y = position_ned_m.y;
        return _position_target_ned_m;
    }

    return set_position_target(position_ned_m, is_terrain_alt);
}

bool AC_Geometric_GuidedTargetManager::should_hold_altitude_for_destination(const Vector3p& position_ned_m, bool is_terrain_alt) const
{
    if (!_target_valid || _is_terrain_alt || is_terrain_alt) {
        return false;
    }

    return get_horizontal_distance(position_ned_m.xy().tofloat(), _position_target_ned_m.xy().tofloat()) >= horizontal_destination_min_m;
}
