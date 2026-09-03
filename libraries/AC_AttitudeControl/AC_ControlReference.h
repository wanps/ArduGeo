#pragma once

#include <AP_Math/AP_Math.h>

#include "AC_AttitudeControl.h"

enum class AC_ControlReferenceFrame : uint8_t {
    LOCAL_NED = 0,
};

enum class AC_ControlReferenceCapability : uint8_t {
    NONE = 0,
    TRAJECTORY,
    ATTITUDE,
    RATE,
};

struct AC_ControlReferenceMeta {
    AC_ControlReferenceFrame frame = AC_ControlReferenceFrame::LOCAL_NED;
    AC_ControlReferenceCapability capability = AC_ControlReferenceCapability::NONE;
    uint32_t timestamp_ms = 0;
    uint32_t sequence = 0;
    bool valid = false;

    bool is_valid_for(AC_ControlReferenceCapability required_capability) const
    {
        return valid &&
               frame == AC_ControlReferenceFrame::LOCAL_NED &&
               capability == required_capability;
    }

    bool is_fresh(uint32_t now_ms, uint32_t max_age_ms) const
    {
        return valid && (now_ms - timestamp_ms) <= max_age_ms;
    }
};

struct AC_TrajectoryReference {
    AC_ControlReferenceMeta meta;
    Vector3p position_ned_m;
    Vector3f velocity_ned_ms;
    Vector3f acceleration_ned_mss;
    AC_AttitudeControl::HeadingCommand heading {};

    bool is_valid() const
    {
        const bool heading_mode_valid =
            heading.heading_mode == AC_AttitudeControl::HeadingMode::Angle_Only ||
            heading.heading_mode == AC_AttitudeControl::HeadingMode::Angle_And_Rate ||
            heading.heading_mode == AC_AttitudeControl::HeadingMode::Rate_Only;
        return meta.is_valid_for(AC_ControlReferenceCapability::TRAJECTORY) &&
               !position_ned_m.is_nan() &&
               !position_ned_m.is_inf() &&
               !velocity_ned_ms.is_nan() &&
               !velocity_ned_ms.is_inf() &&
               !acceleration_ned_mss.is_nan() &&
               !acceleration_ned_mss.is_inf() &&
               isfinite(heading.yaw_angle_rad) &&
               isfinite(heading.yaw_rate_rads) &&
               heading_mode_valid;
    }
};

struct AC_AttitudeReference {
    AC_ControlReferenceMeta meta;
    Quaternion attitude_body_to_ned;
    Vector3f angular_velocity_body_rads;
    Vector3f angular_acceleration_body_radss;

    bool is_valid() const
    {
        return meta.is_valid_for(AC_ControlReferenceCapability::ATTITUDE) &&
               isfinite(attitude_body_to_ned.q1) &&
               isfinite(attitude_body_to_ned.q2) &&
               isfinite(attitude_body_to_ned.q3) &&
               isfinite(attitude_body_to_ned.q4) &&
               attitude_body_to_ned.is_unit_length() &&
               !angular_velocity_body_rads.is_nan() &&
               !angular_velocity_body_rads.is_inf() &&
               !angular_acceleration_body_radss.is_nan() &&
               !angular_acceleration_body_radss.is_inf();
    }
};
