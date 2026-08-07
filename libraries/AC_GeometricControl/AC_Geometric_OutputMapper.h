#pragma once

#include <AP_Param/AP_Param.h>

#include "AC_Geometric_Types.h"

// Persistent normalization parameters for the ArduPilot-facing mapper.
class AC_Geometric_OutputMapper_Params {
public:
    AC_Geometric_OutputMapper_Params();

    static const AP_Param::GroupInfo var_info[];

    float hover_throttle_override() const { return _hover_throttle_norm.get(); }
    Vector3f moment_norm() const;
    void convert_legacy_params(uint16_t old_key);

private:
    // Frozen migration table. Do not change mapped member types or add future
    // parameters here.
    static const AP_Param::GroupInfo legacy_var_info[];

    AP_Float _hover_throttle_norm;
    AP_Float _moment_norm_x;
    AP_Float _moment_norm_y;
    AP_Float _moment_norm_z;
};

// Integration adapter between geometric outputs (f, M) and ArduPilot's
// normalized actuator-intent vector u_geo. This is not a rotor-force
// allocator: AP_Motors retains limiting, frame mixing, spool management,
// and hardware output.
class AC_Geometric_OutputMapper {
public:
    AC_Geometric_OutputMapper() = default;

    // Compute candidate normalized actuator intent only. The vehicle-level
    // ownership gate decides whether this result is the sole actuator-intent
    // writer for the current main-rate frame; this class does not write motors.
    void update(const AC_Geometric_Position_Output& position,
                const AC_Geometric_Attitude_Output& attitude,
                float hover_throttle_norm,
                const Vector3f& moment_norm,
                AC_Geometric_Mapped_Output& output) const;
};
