# Geometric Control

`AC_GeometricControl` implements an experimental Lee-style SE(3) cascade for Copter Guided mode. The library computes a position PID channel, a SO(3) attitude PD channel, and a mapper from geometric thrust and moment proxies to normalized ArduPilot motor inputs.

The library itself does not write motors. Copter decides whether the latest mapped output is allowed to replace the normal Guided rate-controller output.

## Current Scope

- Vehicle: Copter
- Mode: Guided
- Thread path: main rate-controller path only
- Active output: disabled by default
- Rate-thread support: blocked while `using_rate_thread` is true

This controller is intended for SITL and controlled development testing until the parameter set, output limits, and recovery strategy have been validated for the target vehicle.

## Enable Levels

The geometric controller has separate observer and active-output levels.

```text
Default:
GEO_OUT_EN = 0
GUID_OPTIONS bit 8 = 0

Observer only:
GUID_OPTIONS bit 1 = 1

Active motor output in Guided:
GEO_OUT_EN = 1
GUID_OPTIONS bit 1 = 1
GUID_OPTIONS bit 8 = 1
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

## Parameters

All parameters use the `GEO_` prefix in Copter.

| Parameter | Purpose |
| --- | --- |
| `GEO_OUT_EN` | Controller-level active-output enable. Default is disabled. |
| `GEO_POS_KX_XY` | Lee position error gain for horizontal axes. |
| `GEO_POS_KX_Z` | Lee position error gain for vertical axis. |
| `GEO_POS_KI_XY` | Integral position error gain for horizontal axes. |
| `GEO_POS_KI_Z` | Integral position error gain for vertical axis. |
| `GEO_POS_KV_XY` | Lee velocity error gain for horizontal axes. |
| `GEO_POS_KV_Z` | Lee velocity error gain for vertical axis. |
| `GEO_ATT_KR_X` | SO(3) attitude error gain for body X. |
| `GEO_ATT_KR_Y` | SO(3) attitude error gain for body Y. |
| `GEO_ATT_KR_Z` | SO(3) attitude error gain for body Z. |
| `GEO_ATT_KO_X` | Angular velocity error gain for body X. |
| `GEO_ATT_KO_Y` | Angular velocity error gain for body Y. |
| `GEO_ATT_KO_Z` | Angular velocity error gain for body Z. |
| `GEO_POS_FLTE` | Optional first-order low-pass filter for position error. `0` disables it. |
| `GEO_VEL_FLTE` | Optional first-order low-pass filter for velocity error. `0` disables it. |
| `GEO_OMG_FLTE` | Optional first-order low-pass filter for angular velocity error. `0` disables it. |
| `GEO_HOV_THR` | Hover throttle reference used to normalize geometric thrust. |
| `GEO_MOM_NORM_X` | Body-X moment proxy magnitude that maps to full roll output. |
| `GEO_MOM_NORM_Y` | Body-Y moment proxy magnitude that maps to full pitch output. |
| `GEO_MOM_NORM_Z` | Body-Z moment proxy magnitude that maps to full yaw output. |

The gain parameters affect the observer path whenever `GUID_OPTIONS bit 1` is set. They also affect active motor output when both `GEO_OUT_EN` and `GUID_OPTIONS bit 8` are set.

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

## Logs

Guided geometric logging is emitted while the observer level is enabled.

| Message | Meaning |
| --- | --- |
| `GEOP` | Position-channel errors, specific force command, projected thrust, commanded attitude, and ArduPilot reference attitude/thrust-vector comparison. |
| `GEOA` | SO(3) attitude error, angular-rate error, moment proxy, and rate-target proxy. |
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
