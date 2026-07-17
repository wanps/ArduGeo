#pragma once

#include "AC_Geometric_Types.h"

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
