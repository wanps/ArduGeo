#pragma once

#include "AC_Geometric_Types.h"

class AC_Geometric_OutputMapper {
public:
    AC_Geometric_OutputMapper() = default;

    // Build shadow ArduPilot-facing commands from geometric outputs. The
    // mapper does not write attitude_control or motors.
    void update(const AC_Geometric_Position_Output& position,
                const AC_Geometric_Attitude_Output& attitude,
                float hover_throttle_norm,
                const Vector3f& moment_norm,
                AC_Geometric_Mapped_Output& output) const;
};
