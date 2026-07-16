# Geometric Control

`AC_GeometricControl` implements an experimental Lee-style SE(3) cascade for Copter. The library computes a position PID channel, a SO(3) attitude PID channel, and a mapper from geometric thrust and moment proxies to normalized ArduPilot motor inputs.

The library itself does not write motors. Copter decides whether the latest mapped output is allowed to replace the normal mode rate-controller output.

## Current Scope

- Vehicle: Copter
- Modes: Guided active output; experimental Loiter active output enabled for fresh Copter parameter stores
- Thread path: main rate-controller path only
- Active output: enabled by branch default for Guided and fresh Copter Loiter configurations, with independent mode gates
- Rate-thread support: blocked while `using_rate_thread` is true

This controller is intended for SITL and controlled development testing until the parameter set, output limits, recovery strategy, and mode-specific semantics have been validated for the target vehicle.

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
GUID_OPTIONS = 2
GUID_OPTIONS bit 1 = 1
GUID_OPTIONS bit 8 = 0
```

`GUID_OPTIONS=258` is the branch default (`bit 1 + bit 8`). It requests
active geometric output only for the supported Guided submodes described
below; it is not blanket authorization for every Guided command type.

Loiter uses independent option bits:

```text
Fresh ArduCopter parameter-store default:
LOIT_OPTIONS = 7

LOIT_OPTIONS bits:
bit 0 = Coordinated turn during velocity control
bit 1 = Geometric observer
bit 2 = Allow full-lifecycle geometric motor output

Native Loiter:
LOIT_OPTIONS = 1

Observer-only Loiter:
LOIT_OPTIONS = 3

Full-geometric Loiter request:
LOIT_OPTIONS = 7
```

The default is conditional: other firmware that shares `AC_Loiter` retains
`LOIT_OPTIONS=1`. Parameter defaults do not migrate an existing EEPROM or
parameter file, so a vehicle already storing `LOIT_OPTIONS=3` must be changed
to `7` explicitly (or tested from a deliberately wiped parameter store).

The Loiter active-output bit requires the observer bit. To return to native
Loiter, clear bit 2 (`LOIT_OPTIONS=3` when coordinated turn and the observer
remain enabled, or `LOIT_OPTIONS=1` to disable the observer too).
`GEO_OUT_EN` can remain set while the mode-side active-output bit is off.

For the supported ordinary-Copter main-loop path, `LOIT_OPTIONS bit 2` is a
fail-closed request rather than a request to "try geometry, otherwise arm
native." In particular, `LOIT_OPTIONS=7` with `GEO_OUT_EN=0` does not permit
arming. A raw bit-2 request without its observer prerequisite is also invalid;
for example, `LOIT_OPTIONS=5` (bits 0 and 2, but not bit 1) rejects arming
instead of falling through to native output. Select `LOIT_OPTIONS=1` or `3`
explicitly when native motor output is required. Guided has the same
fail-closed rule when `GUID_OPTIONS bit 8` is requested but the configured
geometric path is not ready.

## Meaning of Full-Geometric

In this document, **full-geometric** is a frame-level actuator-ownership
claim over an explicitly stated interval. For every armed main-loop
rate-controller frame that can issue an effective roll, pitch, yaw, and
collective command, the mapped geometric output must be the command written to
`AP_Motors`; the native rate PID must write no effective motor command. With
the cumulative frame logs, the exact criterion is:

```text
delta MFrm = delta GFrm
delta NFrm = 0
```

Disarmed observer/pre-warm updates are not effective motor-command frames and
are outside that accounting. Mode-specific reference generation and AP_Motors
arm/interlock/spool handling are separate from actuator ownership. The full
Loiter path described below skips the native Loiter/position/attitude feedback
updates entirely on every successful dedicated-reference frame; observer-only
Loiter deliberately retains them without writing geometric motor output.
`GEOX.Wrote` is a recent-write status bit, not exact per-frame proof. The
rate-thread path is outside this definition because active geometric output is
currently supported only by the main-loop rate-controller path.

## Experimental Loiter Output

Observer-only Loiter (`LOIT_OPTIONS=3`) retains the native Loiter stack and
maps its already-shaped PVA/yaw targets into the geometric observer without
allowing geometric motor writes. Full-geometric Loiter (`LOIT_OPTIONS=7`) is a
different path: `AC_Geometric_LoiterReference` generates controller-independent
PVA and yaw references, and `ModeLoiter::run()` returns before calling native
`AC_Loiter`, `AC_PosControl`, or attitude feedback updates whenever that path
succeeds.

The dedicated reference keeps the established RC deadzone and Simple-mode
input transforms, then converts pilot lean commands into earth-frame NE
acceleration. It applies drag, dedicated delayed neutral-stick braking,
acceleration and jerk shaping, bounded overspeed recovery, dedicated vertical
velocity/takeoff shaping and finite-time release/reversal braking, and
independent command/brake yaw shaping. Horizontal
braking is gated by raw pilot input, so residual coordinated-turn reference
state cannot restart its neutral timer. Yaw braking starts without a delay when
the raw yaw stick becomes neutral; it uses separate acceleration and jerk
limits and removes same-direction yaw acceleration at that release boundary so
the shaped yaw-rate magnitude cannot continue growing after release. The
vertical brake starts without a delay when the throttle stick returns to its
deadzone or reverses. It follows a discrete S-curve with independent
acceleration and jerk limits, converges to a zero-acceleration stop without a
terminal acceleration jump, latches the stopped reference while the stick
remains neutral, and admits an active reverse command on the following frame. This
prevents both the generic shaper's long low-speed tail and a one-frame vertical
velocity reversal. The profile limits are independent of both the Guided
shaper and native
`LOIT_BRK_*` tuning. Physical limits still include the EKF ground-speed limit,
Loiter lean limit, and pilot climb/descent limits. Enabled avoidance may
constrain the final velocity; the reference recomputes a discretely consistent
PVA target after that constraint. EKF NE/D/yaw resets shift the live reference
without discarding its shaper state.

With `LOIT_OPTIONS=7`, Loiter pre-warms the geometric controller before arming
using a current-state direct-SO(3) target and a `+g` NED acceleration reference.
This produces nominal zero collective and zero attitude/rate tracking error on
the ground. It is not a strict zero-moment target: rigid-body transport
feed-forward may retain a finite non-zero moment proxy when the measured body
rate is non-zero. The raw moment proxy has no separate ground-state magnitude
bound; only the mapped RPY actuator command is constrained to `[-1, 1]`.
AP_Motors still owns arming, interlock, idle, and spool constraints.

The Loiter `Takeoff` state uses `_TakeOff` only to retain the established
takeoff lifecycle and altitude boundary; the dedicated reference owns the
actual vertical PVA generation and lets throttle-stick deflection bound climb
speed. During a flying descent, the Z position reference is anchored to the
measured NED position while its bounded downward velocity and acceleration are
preserved. This prevents the reference from integrating through the ground and
lets the standard land detector observe a feasible low-thrust, level contact.

The shared geometric position controller treats a conventional multicopter as
a unidirectional-thrust system. At the zero-collective boundary it projects the
vertical integral analytically, suppresses lateral-force leakage, regularizes
the target to level-plus-yaw with hysteresis, and resets commanded-rate
differentiation across the transition. Reverse input can release the boundary.
This prevents a grounded descent request from winding up into a 90/180-degree
target while preserving ordinary tilted-flight force commands.

The geometric controller owns effective roll, pitch, yaw and collective output
from the final pre-arm snapshot through takeoff, `Flying`, pilot-controlled
descent, land detection, and AP_Motors spool-down to `GROUND_IDLE`. Loiter then
applies the existing `PILOT_THR_BHV` disarm-on-land policy. `GELF`
frame-counter snapshots provide exact phase-0-through-phase-4 ownership
evidence, including the armed wait before takeoff and touchdown spool-down.

If an armed takeoff is cancelled while the vehicle is still landed, Loiter
synchronously replaces the final takeoff PVA command with the ground-safe
target before the next rate frame. `GELF` phase 5 records that edge and its
prepared throttle instead of treating the cancellation as a native handoff.

Mapped-output limiting, finite position/attitude errors and finite vehicle tilt
are normal controller boundaries; they no longer trigger a native handoff.
NaN/Inf, stale or disabled output, rate-thread operation, an unsupported mode
or airframe, and mode exit remain hard gates. A hard in-flight gate failure
performs one explicit native failsafe handoff and latches geometric re-entry
until the operator clears the Loiter motor-output request or re-enters the mode.

This remains an experimental SITL checkpoint, not a physical-flight readiness
claim. Precision Loiter, traditional helicopters, rate-thread operation, and
active rangefinder surface tracking are explicit unsupported boundaries.
Avoidance is represented as a final velocity constraint in the dedicated
reference, but complex fence/proximity scenarios and physical-flight recovery
remain to be validated. Active geometric frames retain only AP_Motors and the
bookkeeping needed by the standard land detector; the native feedback loops do
not run on the successful dedicated path.

## Active Output Gates

Copter writes geometric outputs to `AP_Motors` only when all of these are true:

- The current flight mode explicitly authorizes geometric motor output
- `GEO_OUT_EN` allows geometric motor output at the controller level
- `using_rate_thread` is false
- Motors are armed
- The geometric observer is enabled
- The geometric output is fresh
- Raw and constrained mapped actuator outputs are finite
- The core specific-force and attitude-moment outputs are finite
- The current mode's additional safety predicate passes

Guided authorization additionally requires the exact `GUIDED` mode,
`GUID_OPTIONS bits 1 and 8`, and a semantically compatible observer update for
one of these submodes:

- non-terrain `TakeOff`
- non-terrain `Pos`
- `PosVelAccel`
- the internal geometric `Land` submode entered by `MAV_CMD_NAV_LAND`

`WP`, `Angle`, `VelAccel`, and `Accel`, plus terrain-relative `TakeOff` or
`Pos`, are explicit native-output boundaries even while bit 8 remains set.
Guided-derived modes do not inherit exact-`GUIDED` permission. An armed
supported-to-unsupported transition performs an explicit native handoff. A
later supported `Pos` or `PosVelAccel` position command prepares its compatible
observer output synchronously before re-authorizing geometry; `GEFB` records
the immediately following rate frame so that a hidden one-frame native bridge
cannot be mistaken for a geometric transition.

Loiter authorization additionally requires `LOIT_OPTIONS bits 1 and 2`, the
geometric output enable, an ordinary multicopter, no precision-Loiter request,
and no active rangefinder surface tracking. Its ground-state mode path prepares
a ground-safe target. Arming enforces the request/configuration, active,
enabled, fresh, and finite predicates; it does not independently compare
prepared throttle with zero or identify which target produced the output.
Landed, possibly-landed and AP_Motors spool states do not relinquish geometric
ownership during a normal lifecycle.

Before arming, a full-geometric mode may keep a finite mode-produced
ground-state output pre-warmed even though the armed-only motor hook is closed.
During an armed lifecycle, a hard gate failure transfers responsibility to the
native controller explicitly; normal mapped-output saturation does not.
Disarming ends the lifecycle and allows the inactive native path to resume on
subsequent disarmed frames.

Guided and Loiter use the same finite-output predicate both before arming and
in the runtime motor gate. It covers raw and constrained mapped RPY/throttle,
position error and specific force, attitude error, and the geometric-PID
moment. A non-finite pre-warmed output therefore rejects arming instead of
becoming a first-frame runtime handoff.

When active geometric output is allowed, geometry owns the effective motor
command for the supported Guided submodes. Non-terrain `TakeOff` and `Pos`, and
the internal `Land` submode, use dedicated geometric paths that bypass native
position feedback. `PosVelAccel` retains its established `AC_PosControl` and
attitude shadow updates for target semantics, bookkeeping, and fallback, but
their rate output does not own the effective motor command: geometry consumes
the explicit PVA target and writes the motors. The internal `Land` path holds
horizontal position and yaw while generating a smooth vertical descent target,
then retains geometric motor ownership through touchdown and `GROUND_IDLE`.

A separate geometric yaw shaper generates yaw and yaw-rate references. For
Guided yaw-follow modes, yaw is derived from the geometric shaper's own shaped
horizontal velocity and acceleration rather than from `AC_PosControl` yaw
state. Clearing `GUID_OPTIONS bit 8` switches the mode back to its established
native controllers. Setting bit 8 does not geometrically authorize WPNav,
terrain-relative, direct-attitude/thrust, velocity-only, or acceleration-only
semantics; those paths remain native until dedicated implementations and
equivalence tests exist.

Full-geometric Loiter has its own mode-specific reference generator; it is
neither Guided underneath nor a consumer of native `AC_Loiter` feedback-loop
targets. Observer-only and explicit hard-fallback paths still use the
established native Loiter stack. The active attitude channel is always the
PID-only `AC_Geometric_Attitude_PID` path on this branch.

### External Compatibility Publication

Supported full-geometric Guided position paths and full-geometric Loiter
publish their selected PVA and geometric attitude/body-rate targets into the
existing compatibility caches. Publication is allowed only while the mode has
selected and prepared full-geometric actuator ownership. Observer-only,
rejected, output-disabled, rate-thread-blocked, native-fallback, and mode-exit
paths clear the external reference instead of leaving a stale geometric target
under native ownership.

`AC_PosControl::publish_external_reference_NED_m()` atomically mirrors the
externally owned position, velocity, and acceleration into both the desired and
target caches. Source-aware position-error accessors then report the published
target minus the estimate. The publisher does not evaluate a native position
feedback controller, alter its PID state, or refresh its controller-run ticks.
Consequently, `NE_is_active()` and `D_is_active()` retain their strict meaning:
the corresponding native feedback controller actually ran recently. The
separate `NE_reference_is_active()` and `D_reference_is_active()` predicates
mean that either a native controller target or a fresh externally owned target
is available. Copter's GCS control-status/base-mode reporting and barometric
ground-effect logic use these reference predicates, so they remain coherent
without pretending that the native position PID ran. Starting either native
axis update clears the full-NED external ownership before native feedback is
evaluated.

`AC_AttitudeControl::set_external_attitude_target()` similarly validates and
publishes the geometric attitude and body-rate caches for downstream
bookkeeping without executing the native attitude or rate feedback loops.
Native PSCN/PSCE/PSCD logging remains isolated: PSC messages require genuine
native controller activity and no active external reference. Dedicated
geometric logs, rather than misleading PSC records, therefore describe the
externally owned trajectory.

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
| `GEO_ATT_KI_Z` | Yaw geometric integral gain. Defaults to `0.1`. |
| `GEO_ATT_IMAX_X` | Roll geometric integral-state limit. Defaults to `0`. |
| `GEO_ATT_IMAX_Y` | Pitch geometric integral-state limit. Defaults to `0`. |
| `GEO_ATT_IMAX_Z` | Yaw geometric integral-state limit. Defaults to `1`. |
| `GEO_ATT_INT_C` | Attitude-error weight inside the geometric integral state. |
| `GEO_ATT_J_X` | Diagonal roll inertia term used by the Lee SO(3) moment formula. Defaults to `0.010` kg*m*m. |
| `GEO_ATT_J_Y` | Diagonal pitch inertia term used by the Lee SO(3) moment formula. Defaults to `0.020` kg*m*m. |
| `GEO_ATT_J_Z` | Diagonal yaw inertia term used by the Lee SO(3) moment formula. Defaults to `0.020` kg*m*m. |
| `GEO_POS_FLTE` | Optional first-order low-pass filter for position error. Defaults to `5 Hz`; `0` disables it. |
| `GEO_VEL_FLTE` | Optional first-order low-pass filter for velocity error. Defaults to `0`; `0` disables it. |
| `GEO_OMG_FLTE` | Optional first-order low-pass filter for angular velocity error. Defaults to `0`; `0` disables it. |
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
| `GEO_LREF_VXY` | Dedicated Loiter horizontal speed limit; the lower of this, `LOIT_SPEED_MS`, and the EKF speed limit is used. Default `2 m/s`. |
| `GEO_LREF_AXY` | Dedicated Loiter pilot/drag horizontal acceleration limit. Default `1 m/s/s`. |
| `GEO_LREF_JXY` | Dedicated Loiter pilot horizontal jerk limit. Default `2 m/s/s/s`. |
| `GEO_LREF_JZ` | Dedicated Loiter vertical jerk limit for commanded climb, descent, and takeoff. Default `5 m/s/s/s`. Existing parameter storage retains an explicitly saved older value until it is changed. |
| `GEO_LREF_YACC` | Dedicated Loiter yaw-acceleration limit. Default `1 rad/s/s`. |
| `GEO_LREF_YJRK` | Dedicated Loiter yaw-jerk limit. Default `2 rad/s/s/s`. |
| `GEO_LREF_BDLY` | Neutral-stick delay before dedicated horizontal braking. Default `0.2 s`. |
| `GEO_LREF_BACC` | Dedicated Loiter extra horizontal brake-acceleration component; it is added to speed-dependent `GEO_LREF_AXY` drag, then the total is bounded by the physical Loiter lean limit. Default `2.5 m/s/s`. |
| `GEO_LREF_BJRK` | Dedicated Loiter horizontal brake-jerk limit. Default `5 m/s/s/s`. |
| `GEO_LREF_YBACC` | Dedicated no-delay yaw brake-acceleration limit. Default `2 rad/s/s`. |
| `GEO_LREF_YBJRK` | Dedicated yaw brake-jerk limit. Default `6 rad/s/s/s`. |
| `GEO_LREF_ZBACC` | Dedicated no-delay vertical brake-acceleration limit for throttle release and reversal. The effective value is also bounded by `PILOT_ACC_Z`. Default `2.5 m/s/s`. |
| `GEO_LREF_ZBJRK` | Dedicated vertical brake-jerk limit for the finite-time S-curve. Default `5 m/s/s/s`. |
| `GEO_HOV_THR` | Hover throttle reference used to normalize geometric thrust. `0` uses the vehicle `MOT_THST_HOVER` estimate; non-zero values override it. |
| `GEO_MOM_NORM_X` | Body-X moment proxy magnitude that maps to full roll output. |
| `GEO_MOM_NORM_Y` | Body-Y moment proxy magnitude that maps to full pitch output. |
| `GEO_MOM_NORM_Z` | Body-Z moment proxy magnitude that maps to full yaw output. |

The gain parameters affect the observer paths whenever `GUID_OPTIONS bit 1` or
`LOIT_OPTIONS bit 1` is set. They affect active motor output only when the
corresponding mode requests full-geometric ownership and all runtime gates pass.

The current yaw baseline includes bounded integral action:

```text
GEO_ATT_KR_Z = 2
GEO_ATT_KO_Z = 0.4
GEO_ATT_KI_Z = 0.1
GEO_ATT_IMAX_Z = 1
GEO_OMG_FLTE = 0
```

In the Gazebo yaw-step sweep, increasing `GEO_ATT_KO_Z` from `0.2` to `0.4`
reduced tail yaw activity without introducing overshoot. `GEO_OMG_FLTE=20`
made the response smoother but increased the remaining yaw error, so it is
kept as an optional noise experiment rather than a default.

The position-error filter defaults to `5 Hz`; the velocity-error and measured
angular-velocity error filters default to disabled. The current baseline is:

```text
GEO_POS_FLTE = 5
GEO_VEL_FLTE = 0
GEO_OMG_FLTE = 0
```

`GEO_VEL_FLTE=5` and `GEO_OMG_FLTE=20` remain optional SITL noise experiments;
they are not branch defaults.

The commanded angular terms generated from the position channel are filtered by
default because `Omega_c` and `dot(Omega_c)` are estimated from discrete `R_c`
changes. This protects GoToLocation and other trajectory transitions from
finite-difference spikes before the SO(3) attitude feed-forward term sees them:

```text
GEO_OMG_C_FLT = 5
GEO_DOMG_C_FLT = 2
```

The setpoint shaper is enabled by default on this branch. In active geometric
non-terrain Guided `TakeOff`, `Pos`, and `PosVelAccel`, the position-derived
target is converted into jerk-limited square-root position, velocity and
acceleration references before the Lee position channel computes `R_c`.
Native thrust-vector and attitude shadow calculations, where retained for
submode semantics, are not the effective motor-command source.
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
| `GEOL` | Loiter altitude state, active/write state, and the selected PVA/yaw reference (dedicated when full-geometric; native when observer-only/fallback). |
| `GELR` | Dedicated Loiter PVA reference, braking/limit state, external velocity constraint, and vertical-position anchor state. |
| `GELO` | Dedicated Loiter yaw reference and `YL`/`YB` yaw limit/brake state; `ZB`/`ZS` vertical brake/settled state; `NER`/`DR` generic NE/D reference freshness; `NEN`/`DN` native NE/D feedback-controller activity; and total/dedicated/native reference-frame counters. |
| `GELT` | Full-geometric Loiter takeoff state, altitude gain, and estimated upward velocity. |
| `GELI` | Exact Loiter mode-entry snapshot: airborne flag, prepared active state, mapped throttle, and cumulative main/geometric/native frame counters. |
| `GELF` | Exact Loiter lifecycle snapshots from one static critical log type, with cumulative frame counters and prepared normalized throttle: phase 0 final pre-arm ground-safe snapshot, phase 1 takeoff lifecycle/start edge before the first PVA update, phase 2 airborne entry/recovery, phase 3 land complete, phase 4 `GROUND_IDLE` before disarm, and phase 5 armed takeoff cancellation after synchronous ground-safe replacement. |
| `GEFC` | Exact Guided lifecycle snapshots from one static critical log type: phase 0 stored pre-arm edge synchronized by the final arming hook, phase 1 takeoff accepted, phase 2 internal landing accepted, phase 3 land complete, and phase 4 `GROUND_IDLE` before disarm. |
| `GEFB` | Exact Guided unsupported-to-supported `Pos`/`PosVelAccel` position-command boundary pair: phase 0 after synchronous compatible-target preparation and phase 1 after the immediately following main-loop rate frame. |
| `GEFR` | Decimated streaming snapshot of cumulative main-loop, geometric-output, and native-rate frame counters. |
| `GELC` | Observer-only/fallback Loiter comparison between geometric position output and native attitude/thrust-vector references; not emitted by the dedicated full-geometric path. |
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
| `GEOH` | One-shot active-to-native handoff event: mode, failure mask, previous/next active state, handoff result, and rate-thread state. |

`MFrm`, `GFrm`, and `NFrm` are cumulative counters. On the supported
main-loop path, subtracting two exact snapshots gives the effective controller
ownership in the enclosed interval. `GEFR` is intentionally decimated, so an
arbitrary first/last `GEFR` collection window can miss a command boundary. Use
`GEFB` for the first frame after Guided unsupported-to-supported `Pos` or
`PosVelAccel` position-command recovery, `GEFC` for the Guided
arm-to-ground-idle lifecycle, and `GELI`/`GELF` for Loiter entry and lifecycle
edges.

`GELF.Thr` is the normalized geometric throttle prepared at the exact edge.
Phase 0 captures the final armable pre-arm ground-state output; the lifecycle
test requires its throttle to be finite and nominally zero. Phase 5 is emitted
only after an armed, still-landed takeoff cancellation has synchronously
replaced the final non-zero takeoff PVA command with the ground-safe target;
its expected throttle is zero before the next rate frame.

`GEFC` and `GELF` are separate statically registered packet types written with
the logger's critical-block path. Their lifecycle tests require exactly one FMT
ID for each name, preventing a duplicate dynamic schema from silently hiding
phases while also reserving critical-buffer capacity for these one-shot edges.

During dedicated Loiter ownership, `GELO.NER=1` and `GELO.DR=1` show that the
published external PVA remains fresh for generic consumers. `GELO.NEN=0` and
`GELO.DN=0` independently show that neither native position feedback controller
ran recently. The lifecycle regression requires all four predicates on every
sample in its dedicated-reference windows; reference availability is therefore
not conflated with native-controller execution.

For Guided, the final arming hook synchronously stores the `GEFC` phase-0
counter baseline so an immediately adjacent `ARM` and `TAKEOFF` command pair
cannot miss the lifecycle edge. A native Guided arm invalidates any older
full-geometric pre-arm snapshot. Phase 0 and phase 1 may occur in the same
scheduler tick, so their counter delta is allowed to be zero; a non-empty
arm-to-takeoff interval is not required.

`GEOX` is the primary message for checking active-output state:

- `Allow`: the current exact mode has completed its mode-specific output authorization
- `OEn`: `GEO_OUT_EN` is set
- `RT`: rate-thread path is active and geometric motor output is blocked
- `Wrote`: geometric output recently wrote `AP_Motors`
- `GAge`: age of the latest geometric controller output
- `WAge`: age of the latest geometric write to `AP_Motors`

`GEOH.Fail` is a bitmask: bit 0 mode authorization, bit 1 controller output
disabled, bit 2 rate-thread use, bit 3 disarmed, bit 4 controller disabled, bit 5
stale output, bit 6 non-finite output, and bit 7 mode-specific safety. `GEOH` is
event logging rather than a stream; one row is expected for each active-to-native
handoff.

## SITL Coverage

The PID-only Guided/Loiter architecture port based on `5904bfbb29` was
validated on 2026-07-16 after the Guided/Loiter architecture port and the
single-FMT logging fix:

- Copter SITL firmware build: PASS (`Text=4,481,196 B`, `Data=216,861 B`,
  `BSS=226,240 B`, total flash usage `4,698,057 B`).
- All 80 PID-only unit tests: PASS. This comprises Loiter reference 25/25,
  position PID 14/14, attitude PID 10/10, Guided target manager 11/11,
  output mapper 5/5, setpoint shaper 7/7, yaw shaper 5/5, and external
  attitude-target publication 3/3.
- Copter logger metadata, ArduCopter parameter metadata, source-diff checks,
  and source/binary checks for excluded controller symbols and parameters:
  PASS.
- Seven focused SITL regressions: PASS after the final logging change.
  `GeometricGuidedFullLifecycle` also asserts that `GEFC`, `GEOX`, and `GEFR`
  each have exactly one FMT ID after both Loiter and Guided have logged.

The focused SITL evidence is:

- `GeometricLoiterObserver`: observer enable/isolation, native-reference
  comparison and no geometric motor write passed. Mean thrust-vector and
  attitude alignment were `0.998800` and `0.997445`.
- `GeometricLoiterTakeoffLandingMotorOutput`: aborted-takeoff reset, invalid
  option rejection, takeoff cancellation, dedicated-reference ownership and
  two same-mode takeoff/landing cycles passed. The first lifecycle recorded
  `(MFrm,GFrm,NFrm)=(12127,12127,0)`, including pre-takeoff
  `(1011,1011,0)` and reference ownership `(12058,12058,0)`; the second was
  `(5889,5889,0)`, including pre-takeoff `(1009,1009,0)`.
- `GeometricLoiterAirborneEntry`: immediate non-zero hover preparation passed
  with `(MFrm,GFrm,NFrm)=(522,522,0)`, `Thr=0.375919`, and `0.005977 m`
  altitude span.
- `GeometricLoiterMotorOutput`: RC translation/yaw/climb, dedicated braking,
  limiting, hard-fault latch and explicit recovery passed. Frame deltas were
  observer-only `(282,0,282)`, active `(201,201,0)`, climb `(1009,1009,0)`,
  vertical return `(889,889,0)`, limited `(768,768,0)`, fault-off
  `(165,0,165)`, latched `(286,0,286)`, bit-clear `(205,0,205)`, and
  recovered `(161,161,0)`. Measured stop windows were `0.402339 s` vertical,
  `0.805511 s` horizontal and `1.017093 s` yaw; explicit recovery took
  `42,483 us`.
- `GeometricGuidedFullLifecycle`: two full takeoff-through-ground-idle cycles
  passed with `(MFrm,GFrm,NFrm)=(14906,14906,0)` and `(10030,10030,0)`.
- `GeometricGuidedMotorOutputPositionYawStep`: combined position/yaw response
  passed with `5.693111 m` X progress, final distance `0.225446 m`, final yaw
  error `5.200067 deg`, lateral drift `0.015944 m`, and vertical drift
  `0.074933 m`.
- `GeometricGuidedMotorOutputSwitching`: native/geometric switching,
  output-disable hard-fault latching, explicit acknowledgement and recovery
  passed; the telemetry window stayed within `0.036432 m` horizontal and
  `0.354999 m` vertical drift.

The other Guided observer, hover, single-axis step and filtered-noise tests
remain available in `Tools/autotest/arducopter.py`, but were not rerun in this
focused post-port validation.

There is no current exact-frame SITL coverage for Guided `PosVelAccel`, ground
arming rejection with `GEO_OUT_EN=0`, non-finite pre-arm fault injection,
terrain-relative targets, WPNav, `VelAccel`, `Accel`, traditional helicopters,
precision Loiter, rate-thread operation, active surface tracking, or complex
avoidance/fence scenarios. The zero-collective vertical anti-windup boundary is
covered by unit tests and both Guided/Loiter landing regressions, but prolonged
multi-axis saturation still needs dedicated coverage. None of the listed tests
is HIL or physical-flight validation.

Run a single test with:

```text
CCACHE_DISABLE=1 Tools/autotest/autotest.py test.Copter.GeometricLoiterTakeoffLandingMotorOutput
```

The geometric Copter test group currently lives in `Tools/autotest/arducopter.py`.
