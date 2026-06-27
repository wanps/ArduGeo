#include "AC_Geometric_GuidedTargetManager.h"

void AC_Geometric_GuidedTargetManager::reset()
{
    _position_target_ned_m.zero();
    _is_terrain_alt = false;
    _target_valid = false;
    _trajectory_yaw_allowed = false;
    _target_type = TargetType::None;
}

const Vector3p& AC_Geometric_GuidedTargetManager::set_position_target(const Vector3p& position_ned_m, bool is_terrain_alt)
{
    return set_position_target(position_ned_m, is_terrain_alt, nullptr);
}

const Vector3p& AC_Geometric_GuidedTargetManager::set_position_target(const Vector3p& position_ned_m,
                                                                      bool is_terrain_alt,
                                                                      const Vector3p* current_pos_ned_m)
{
    if (should_merge_altitude_for_position_target(position_ned_m, is_terrain_alt, current_pos_ned_m)) {
        _position_target_ned_m.z = position_ned_m.z;
        _target_type = TargetType::PositionAltitudeMerge;
        return _position_target_ned_m;
    }

    _trajectory_yaw_allowed = false;
    _position_target_ned_m = position_ned_m;
    _is_terrain_alt = is_terrain_alt;
    _target_valid = true;
    _target_type = TargetType::Position;
    return _position_target_ned_m;
}

const Vector3p& AC_Geometric_GuidedTargetManager::set_destination_target(const Vector3p& position_ned_m, bool is_terrain_alt)
{
    const bool first_target = !_target_valid;
    const bool horizontal_destination = is_horizontal_destination(position_ned_m, is_terrain_alt);
    const bool vertical_destination = is_vertical_destination(position_ned_m, is_terrain_alt);

    _trajectory_yaw_allowed = first_target ? !is_terrain_alt : is_trajectory_yaw_destination(position_ned_m, is_terrain_alt);
    if (is_terrain_alt || _is_terrain_alt) {
        _target_type = TargetType::TerrainDestination;
    } else if (first_target) {
        _target_type = TargetType::FirstDestination;
    } else if (horizontal_destination && vertical_destination) {
        _target_type = TargetType::HorizontalVerticalDestination;
    } else if (horizontal_destination) {
        _target_type = TargetType::HorizontalDestination;
    } else if (vertical_destination) {
        _target_type = TargetType::VerticalDestination;
    } else {
        _target_type = TargetType::SmallDestination;
    }

    if (should_hold_altitude_for_destination(position_ned_m, is_terrain_alt)) {
        _position_target_ned_m.x = position_ned_m.x;
        _position_target_ned_m.y = position_ned_m.y;
        return _position_target_ned_m;
    }

    _position_target_ned_m = position_ned_m;
    _is_terrain_alt = is_terrain_alt;
    _target_valid = true;
    return _position_target_ned_m;
}

bool AC_Geometric_GuidedTargetManager::is_horizontal_destination(const Vector3p& position_ned_m, bool is_terrain_alt) const
{
    if (!_target_valid || _is_terrain_alt || is_terrain_alt) {
        return false;
    }

    return get_horizontal_distance(position_ned_m.xy().tofloat(), _position_target_ned_m.xy().tofloat()) >= horizontal_destination_min_m;
}

bool AC_Geometric_GuidedTargetManager::is_vertical_destination(const Vector3p& position_ned_m, bool is_terrain_alt) const
{
    if (!_target_valid || _is_terrain_alt || is_terrain_alt) {
        return false;
    }

    return fabsf(float(position_ned_m.z - _position_target_ned_m.z)) >= vertical_destination_min_m;
}

bool AC_Geometric_GuidedTargetManager::is_trajectory_yaw_destination(const Vector3p& position_ned_m, bool is_terrain_alt) const
{
    if (!_target_valid || _is_terrain_alt || is_terrain_alt) {
        return false;
    }

    return get_horizontal_distance(position_ned_m.xy().tofloat(), _position_target_ned_m.xy().tofloat()) >= trajectory_yaw_destination_min_m;
}

bool AC_Geometric_GuidedTargetManager::is_destination_target_type() const
{
    switch (_target_type) {
    case TargetType::FirstDestination:
    case TargetType::HorizontalDestination:
    case TargetType::VerticalDestination:
    case TargetType::HorizontalVerticalDestination:
    case TargetType::SmallDestination:
    case TargetType::PositionAltitudeMerge:
        return true;
    case TargetType::None:
    case TargetType::Position:
    case TargetType::TerrainDestination:
        return false;
    }

    return false;
}

bool AC_Geometric_GuidedTargetManager::should_merge_altitude_for_position_target(const Vector3p& position_ned_m,
                                                                                 bool is_terrain_alt,
                                                                                 const Vector3p* current_pos_ned_m) const
{
    if (!_target_valid || _is_terrain_alt || is_terrain_alt || current_pos_ned_m == nullptr) {
        return false;
    }
    if (!is_destination_target_type() || !is_vertical_destination(position_ned_m, is_terrain_alt)) {
        return false;
    }

    return get_horizontal_distance(position_ned_m.xy().tofloat(), current_pos_ned_m->xy().tofloat()) <= altitude_merge_current_xy_max_m;
}

bool AC_Geometric_GuidedTargetManager::should_hold_altitude_for_destination(const Vector3p& position_ned_m, bool is_terrain_alt) const
{
    return is_horizontal_destination(position_ned_m, is_terrain_alt) &&
           !is_vertical_destination(position_ned_m, is_terrain_alt);
}
