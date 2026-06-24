#pragma once

#include "AC_Geometric_Types.h"

class AC_Geometric_Attitude_PD {
public:
    AC_Geometric_Attitude_PD() = default;

    void set_gains(const AC_Geometric_Attitude_Gains& gains) { _gains = gains; }
    const AC_Geometric_Attitude_Gains& get_gains() const { return _gains; }

    void update(const AC_Geometric_State& state,
                const AC_Geometric_Target& target,
                float dt,
                AC_Geometric_Attitude_Output& output) const;

private:
    AC_Geometric_Attitude_Gains _gains;
};
