#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

#include "AC_Geometric_Attitude_PD.h"
#include "AC_Geometric_OutputMapper.h"
#include "AC_Geometric_Position_PID.h"
#include "AC_Geometric_Types.h"

class AC_GeometricControl {
public:
    AC_GeometricControl();
    CLASS_NO_COPY(AC_GeometricControl);

    static const AP_Param::GroupInfo var_info[];

    // Clear controller integrators and cached outputs.
    void reset();

    // The geometric path is opt-in. Disabling clears output so callers do not
    // accidentally consume stale geometric commands.
    void set_enabled(bool enabled);
    bool enabled() const { return _enabled; }
    bool output_enabled() const { return _output_enabled; }
    uint32_t output_age_ms(uint32_t now_ms) const;
    bool output_is_fresh(uint32_t now_ms, uint32_t max_age_ms) const;

    // Run the geometric position-to-attitude and attitude PD cascade.
    // This library only computes outputs; vehicle code decides whether to write motors.
    void update(const AC_Geometric_State& state,
                const AC_Geometric_Target& target,
                float dt);

    const AC_Geometric_Output& get_output() const { return _output; }

private:
    void update_gains_from_params();

    bool _enabled = false;
    AC_Geometric_Position_PID _position_pid;
    AC_Geometric_Attitude_PD _attitude_pd;
    AC_Geometric_OutputMapper _output_mapper;
    AC_Geometric_Output _output;
    uint32_t _last_update_ms = 0;

    AP_Float _pos_kx_xy;
    AP_Float _pos_kx_z;
    AP_Float _pos_ki_xy;
    AP_Float _pos_ki_z;
    AP_Float _pos_kv_xy;
    AP_Float _pos_kv_z;
    AP_Float _pos_error_filt_hz;
    AP_Float _vel_error_filt_hz;

    AP_Float _att_kr_x;
    AP_Float _att_kr_y;
    AP_Float _att_kr_z;
    AP_Float _att_ko_x;
    AP_Float _att_ko_y;
    AP_Float _att_ko_z;
    AP_Float _omega_error_filt_hz;

    AP_Float _hover_throttle_norm;
    AP_Float _mom_norm_x;
    AP_Float _mom_norm_y;
    AP_Float _mom_norm_z;
    AP_Int8 _output_enabled;
};
