# Geometric Control

`AC_GeometricControl` implements an experimental Lee-style SE(3) cascade for Copter Guided mode. The library computes a position PID channel, a SO(3) attitude PID channel, and a mapper from geometric thrust and moment proxies to normalized ArduPilot motor inputs.

The library itself does not write motors. Copter decides whether the latest mapped output is allowed to replace the normal Guided rate-controller output.

## Current Scope

- Vehicle: Copter
- Mode: Guided
- Thread path: main rate-controller path only
- Active output: enabled by default on this geometric-control branch
- Rate-thread support: blocked while `using_rate_thread` is true

This controller is intended for SITL and controlled development testing until the parameter set, output limits, and recovery strategy have been validated for the target vehicle.

## Enable Levels

The geometric controller has separate observer and active-output levels.

```text
Branch default:
GUID_OPTIONS = 258
GEO_OUT_EN = 1
GEO_SHAPE_EN = 1
GEO_SHAPE_YAW = 1

GUID_OPTIONS = 258 means:
bit 1 = Geometric observer
bit 8 = Geometric motor output

Observer only:
GUID_OPTIONS bit 1 = 1
GUID_OPTIONS bit 8 = 0
```

To return to the normal Guided output path, clear `GUID_OPTIONS bit 8`. `GEO_OUT_EN` can remain set while testing if the mode-side active-output bit is off.

## Active Output Gates

Copter writes geometric outputs to `AP_Motors` only when all of these are true:

- Flight mode is Guided
- `GUID_OPTIONS bit 8` allows geometric motor output
- `GEO_OUT_EN` allows geometric motor output at the controller level
- `using_rate_thread` is false
- Motors are armed
- The geometric observer is enabled
- The geometric output is fresh

If any gate is false, the normal Guided controller path remains responsible for motor output.

When active geometric output is allowed, Guided position-style targets are no
longer advanced through the native `AC_PosControl` PID loops before they enter
the geometric controller. Guided Pos and Guided WP targets are passed as final
goals to the geometric setpoint shaper, which uses ArduPilot's jerk-limited
square-root shaping helpers to generate the `x_d`, `v_d`, and `a_d`
references consumed by the Lee position channel. A separate geometric yaw
shaper generates yaw and yaw-rate references. For Guided yaw-follow modes, yaw
is derived from the geometric shaper's own shaped horizontal velocity and
acceleration rather than from `AC_PosControl` yaw state.
Clearing `GUID_OPTIONS bit 8` switches these paths back to the normal
WPNav/AC_PosControl/AC_AttitudeControl loop. Terrain-frame position targets
currently fall back to the native path until geometric terrain handling is
implemented.

## Parameters

All parameters use the `GEO_` prefix in Copter.

| Parameter | Purpose |
| --- | --- |
| `GEO_OUT_EN` | Controller-level active-output enable. Default is enabled on this branch. |
| `GEO_POS_KX_XY` | Lee position error gain for horizontal axes. |
| `GEO_POS_KX_Z` | Lee position error gain for vertical axis. |
| `GEO_POS_KI_XY` | Geometric position integral gain for horizontal axes. |
| `GEO_POS_KI_Z` | Geometric position integral gain for vertical axis. |
| `GEO_POS_KV_XY` | Lee velocity error gain for horizontal axes. |
| `GEO_POS_KV_Z` | Lee velocity error gain for vertical axis. |
| `GEO_POS_IMAX_XY` | Horizontal geometric position integral-state limit in meters. |
| `GEO_POS_IMAX_Z` | Vertical geometric position integral-state limit in meters. |
| `GEO_POS_INT_C` | Position-error weight inside `e_XI = integral(e_v + c_x*e_x)`. |
| `GEO_ATT_KR_X` | SO(3) attitude error gain for body X. |
| `GEO_ATT_KR_Y` | SO(3) attitude error gain for body Y. |
| `GEO_ATT_KR_Z` | SO(3) attitude error gain for body Z. |
| `GEO_ATT_KO_X` | Angular velocity error gain for body X. |
| `GEO_ATT_KO_Y` | Angular velocity error gain for body Y. |
| `GEO_ATT_KO_Z` | Angular velocity error gain for body Z. Defaults to `0.4` after Gazebo yaw-step tuning. |
| `GEO_ATT_KI_X` | Roll geometric integral gain. Defaults to `0`. |
| `GEO_ATT_KI_Y` | Pitch geometric integral gain. Defaults to `0`. |
| `GEO_ATT_KI_Z` | Yaw geometric integral gain. Defaults to `0`. |
| `GEO_ATT_IMAX_X` | Roll geometric integral-state limit. Defaults to `0`. |
| `GEO_ATT_IMAX_Y` | Pitch geometric integral-state limit. Defaults to `0`. |
| `GEO_ATT_IMAX_Z` | Yaw geometric integral-state limit. Defaults to `0` so yaw integral accumulation is off unless explicitly enabled. |
| `GEO_ATT_INT_C` | Attitude-error weight inside the geometric integral state. |
| `GEO_ATT_J_X` | Diagonal roll inertia term used by the Lee SO(3) moment formula. Defaults to `0.011`, matching the Gao reference model `10^-2 diag(1.1,2.0,2.3)` kg*m*m. |
| `GEO_ATT_J_Y` | Diagonal pitch inertia term used by the Lee SO(3) moment formula. Defaults to `0.020`, matching the Gao reference model `10^-2 diag(1.1,2.0,2.3)` kg*m*m. |
| `GEO_ATT_J_Z` | Diagonal yaw inertia term used by the Lee SO(3) moment formula. Defaults to `0.023`, matching the Gao reference model `10^-2 diag(1.1,2.0,2.3)` kg*m*m. |
| `GEO_POS_FLTE` | Optional first-order low-pass filter for position error. `0` disables it. |
| `GEO_VEL_FLTE` | Optional first-order low-pass filter for velocity error. `0` disables it. |
| `GEO_OMG_FLTE` | Optional first-order low-pass filter for angular velocity error. `0` disables it. |
| `GEO_OMG_C_FLT` | Optional first-order low-pass filter for position-generated `Omega_c`. Defaults to `5 Hz`. |
| `GEO_DOMG_C_FLT` | Optional first-order low-pass filter for position-generated `dot(Omega_c)`. Defaults to `2 Hz`. |
| `GEO_SHAPE_EN` | Optional position setpoint shaper enable. Default is enabled on this branch. |
| `GEO_SHAPE_VXY` | Horizontal reference velocity limit for the setpoint shaper. |
| `GEO_SHAPE_AXY` | Horizontal reference acceleration limit for the setpoint shaper. |
| `GEO_SHAPE_VUP` | Upward reference velocity limit for the setpoint shaper. |
| `GEO_SHAPE_VDN` | Downward reference velocity limit for the setpoint shaper. |
| `GEO_SHAPE_AZ` | Vertical reference acceleration limit for the setpoint shaper. |
| `GEO_SHAPE_YAW` | Optional shaper for explicit yaw commands. Default is enabled on this branch. Geometric yaw-follow is generated from shaped trajectory velocity/acceleration separately. |
| `GEO_SHAPE_YRAT` | Yaw reference rate limit for explicit yaw shaping and geometric yaw-follow. |
| `GEO_SHAPE_YACC` | Yaw reference acceleration limit for explicit yaw shaping and geometric yaw-follow. |
| `GEO_HOV_THR` | Hover throttle reference used to normalize geometric thrust. `0` uses the vehicle `MOT_THST_HOVER` estimate; non-zero values override it. |
| `GEO_MOM_NORM_X` | Body-X moment proxy magnitude that maps to full roll output. |
| `GEO_MOM_NORM_Y` | Body-Y moment proxy magnitude that maps to full pitch output. |
| `GEO_MOM_NORM_Z` | Body-Z moment proxy magnitude that maps to full yaw output. |

The gain parameters affect the observer path whenever `GUID_OPTIONS bit 1` is set. They also affect active motor output when both `GEO_OUT_EN` and `GUID_OPTIONS bit 8` are set.

The current yaw baseline is PD by default:

```text
GEO_ATT_KR_Z = 2
GEO_ATT_KO_Z = 0.4
GEO_ATT_KI_Z = 0
GEO_ATT_IMAX_Z = 0
GEO_OMG_FLTE = 0
```

In the Gazebo yaw-step sweep, increasing `GEO_ATT_KO_Z` from `0.2` to `0.4`
reduced tail yaw activity without introducing overshoot. `GEO_OMG_FLTE=20`
made the response smoother but increased the remaining yaw error, so it is
kept as an optional noise experiment rather than a default.

The error filters are disabled by default so the controller preserves the
validated raw Lee state-error path. For SITL noise experiments, a conservative
starting point is:

```text
GEO_POS_FLTE = 5
GEO_VEL_FLTE = 5
GEO_OMG_FLTE = 20
```

The 5 Hz translational filters follow the same order of magnitude as
ArduPilot's position/velocity controller filters and the previous PDNN
prototype. The 20 Hz angular-velocity filter follows the multicopter rate-loop
filter order and keeps attitude damping latency lower than the translational
path.

The commanded angular terms generated from the position channel are filtered by
default because `Omega_c` and `dot(Omega_c)` are estimated from discrete `R_c`
changes. This protects GoToLocation and other trajectory transitions from
finite-difference spikes before the SO(3) attitude feed-forward term sees them:

```text
GEO_OMG_C_FLT = 5
GEO_DOMG_C_FLT = 2
```

The setpoint shaper is enabled by default on this branch. In active geometric
Guided Pos/WP, the final target point is converted into jerk-limited
square-root position, velocity and acceleration references before the Lee
position channel computes `R_c`; the native `AC_PosControl` PID output is not
used to create those references.
For Guided yaw-follow (`LOOK_AT_NEXT_WP` or `LOOK_AHEAD`), yaw is generated from
the geometric yaw shaper using the shaped horizontal velocity and acceleration.
At low speed it holds the last yaw reference to avoid target-direction
ambiguity near the destination. Explicit yaw commands still enter through the
Guided yaw command path, but `GEO_SHAPE_YAW` now lets the geometric yaw shaper
smooth those commands without reading back `AC_AttitudeControl` yaw target
state. The current default branch starting point is:

```text
GEO_SHAPE_EN = 1
GEO_SHAPE_VXY = 1
GEO_SHAPE_AXY = 0.5
GEO_SHAPE_VUP = 2.5
GEO_SHAPE_VDN = 1.5
GEO_SHAPE_AZ = 1
GEO_SHAPE_YAW = 1
GEO_SHAPE_YRAT = 1
GEO_SHAPE_YACC = 1
```

## Logs

Guided geometric logging is emitted while the observer level is enabled.

| Message | Meaning |
| --- | --- |
| `GEOP` | Position-channel errors, specific force command, projected thrust, commanded attitude, and ArduPilot reference attitude/thrust-vector comparison. |
| `GEOA` | SO(3) attitude error, angular-rate error, integral error, moment proxy, and rate-target proxy. |
| `GEOC` | Commanded attitude coupling diagnostics: `R_c`, native attitude target, position-generated `Omega_c`, `dot(Omega_c)`, and yaw inputs. |
| `GEOS` | Raw and shaped position targets plus shaper and trajectory-yaw state. |
| `GEOT` | Guided target semantics: target type, AutoYaw mode, pause state, shaping request, and trajectory-yaw permission. |
| `GESV` | Shaped velocity, acceleration, yaw, and yaw-rate targets. |
| `GEOZ` | Vertical-channel target, state, error, and shaped vertical feed-forward terms. |
| `GEZI` | Vertical integral, specific force, and normalized throttle diagnostics. |
| `GEOO` | Output-mapper attitude and normalized throttle values before motor write status checks. |
| `GEOM` | Output-mapper roll, pitch, and yaw normalized actuator values. |
| `GEOX` | Active motor-output hook state: mode allow, controller enable, rate-thread state, write recency, output age, normalized outputs, and limit flags. |

`GEOX` is the primary message for checking active-output state:

- `Allow`: `GUID_OPTIONS bit 8` is set
- `OEn`: `GEO_OUT_EN` is set
- `RT`: rate-thread path is active and geometric motor output is blocked
- `Wrote`: geometric output recently wrote `AP_Motors`
- `GAge`: age of the latest geometric controller output
- `WAge`: age of the latest geometric write to `AP_Motors`

## SITL Coverage

Current autotests cover these Guided scenarios:

- Observer-only geometric position and attitude logging
- Active hover
- Active local position step
- Active shaped local position step with the setpoint shaper enabled
- Active yaw step
- Active combined local position and yaw step
- Active filtered hover with SITL IMU/GPS/barometer noise
- Disabled active output with `GUID_OPTIONS bit 8` set and `GEO_OUT_EN=0`
- In-flight switching from normal Guided output to geometric output, back to normal Guided output, then back to geometric output

Run a single test with:

```text
CCACHE_DISABLE=1 Tools/autotest/autotest.py test.Copter.GeometricGuidedMotorOutputSwitching
```

The full geometric Guided group currently lives in `Tools/autotest/arducopter.py`.
