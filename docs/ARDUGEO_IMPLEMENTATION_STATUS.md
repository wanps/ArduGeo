# ArduGeo Controller Framework — Implementation Status

**Branch:** `architecture/controller-framework-v1`

**Architecture content revision:** v1.3

**ArduGeo release baseline:** v1.2.0 (`697c4da33c0cf0d7b888e9eb6ec2645b6749e91c`)

## Completed

- C0 ✅ Baseline / remotes / parameter-migration audit
- C1 ✅ Safe defaults — Native output by default, Geo explicit opt-in
- C2 ✅ Controller-neutral trajectory / attitude reference types
- C3 ✅ Copter-level geometric integration bridge
- C4 ✅ Guided / Loiter neutral-reference migration
- C5 ✅ AUTO WP/Spline observer-only
- C6 ✅ AUTO WP/Spline active geometric ownership
- C7 ✅ RTL WPNav phase observer-only
- C8 ✅ RTL Return Home / Loiter At Home active geometric ownership
- C9 ✅ Flight Mode capability taxonomy + selection/handoff consolidation audit (read-only)
- C10 ✅ Full-Trajectory Authorization Minimal Consolidation
- C11 ✅ Guided Engineering Closure & Coverage Verification
- C12 ✅ Guided lifecycle closure
- Guided WP Active Closure ✅ Non-terrain WP active geometric ownership
- Circle Observer ✅ Non-terrain ordinary Circle neutral-reference observation
- Circle Active Ownership ✅ Non-terrain ordinary Circle active geometric ownership
- Mode Transition Hardening ✅ Cross-mode ownership, lifecycle, and cache protection

## Guided Engineering v1.0 support matrix

| Support class | Guided paths | Decision |
| --- | --- | --- |
| Geo Active Supported | Initial entry hold, Pos, PosVelAccel, non-terrain WP, Pause, Guided Land | Pause and compatible full-trajectory transitions keep continuous Geo ownership |
| Geo Observer Only | None in the Engineering v1.0 Guided Full-Trajectory support set | Non-terrain WP was promoted to active after ownership closure |
| Native by Design | TakeOff, VelAccel, Accel, Angle, Rate, direct thrust, terrain WP, Guided_NoGPS, tradheli, rate-thread | TakeOff and WP `Rate_Only` preserve Native heading semantics |

Guided Engineering v1.0 Full-Trajectory closure: ✅ Complete.

Guided WP active ownership remains fail-closed at these Native-by-design boundaries:

```text
terrain WP
Rate_Only heading
traditional helicopter
rate-thread output
```

## Circle current support matrix

| Support class | Circle paths | Decision |
| --- | --- | --- |
| Geo Observer Supported | Non-terrain ordinary Circle without active opt-in | Native shaped P/V/A and compatible heading feed the neutral-reference observer; Native retains actuator ownership |
| Geo Active Supported | Non-terrain ordinary Circle with radius greater than zero | `GEO_OUT_EN` plus `CIRCLE_OPTIONS` bit 8 explicitly authorizes fresh, finite, valid main-thread output |
| Native by Design | `Rate_Only`, radius-zero panorama, terrain, surface tracking, tradheli, rate-thread | Structural boundaries fail closed without setting the hard-fault latch |

Circle mode exit invalidates its own authorization and reference state. A geometric-controller update generation token prevents the outgoing Mode from clearing a newer output already published by the entering Mode during ArduPilot's new-Mode-init-before-old-Mode-exit sequence. The token protects only the shared geometric-controller lifecycle: it does not grant actuator ownership, which remains determined by the current Mode authorization and the vehicle-level arbiter. This targeted generation check is not a general ControllerManager.

AUTO and RTL use the same generation-aware exit pattern for their WPNav geometric observer/active paths. Their exit cleanup always clears Mode-local support and authorization state, but only disables the shared geometric controller/cache when no newer entering Mode output has already been published. This protects shared-controller lifecycle only; it does not authorize actuator ownership and does not replace current Mode authorization plus the vehicle-level arbiter.

## Other current verified coverage

```text
Loiter supported lifecycle        Geo active
AUTO WP / Spline                  Geo observer + active
RTL Return Home                   Geo observer + active
RTL Loiter At Home                Geo observer + active
```

Loiter structural unsupported transitions do not create the runtime hard-fault rejected latch. Existing hard-fault latches are preserved across structural unsupported periods and still require explicit mode-level acknowledge before Geo active recovery.

## Current explicit Native boundaries

```text
RTL Initial Climb                 Rate_Only → Native
RTL Final Descent                 Native
RTL Land                          Native
RTL Terrain                       Native
AUTO unsupported submodes         Native
Rate/direct/special Mode family   Native unless separately designed
```

## Safety invariants currently preserved

- one main-rate frame → one actuator-intent writer
- Geo→Native same-frame handoff
- stale / nonfinite / disabled / unsupported fail closed
- active-path runtime hard faults latch
- no automatic re-entry after hard fault
- explicit mode-level acknowledge required
- AP_Motors / mixer / HAL remain Native infrastructure
- WPNav / AutoYaw semantics remain Native
- observer-first required before new active ownership

## Recent merged milestones

- C5 AUTO observer: `2c3ac2c7c5bfede34e87a74827326a3409d458a3`
- C6 AUTO active: `96144bd0e383da2a0c9b805aafb3e8503b8c523d`
- C7 RTL observer: `cbb5ed8a49c11f83822d8cf5caef557222e4fa6f`
- C8 RTL active: `407477c467e4a4478dddd42bab02dabab1ce6828`

## Hardware gate H1 (R-26)

**Passed** on the X400 airframe, 2026-09-23, against `e46f64c`. Props removed,
`YJUAV_A6SE_H743` connected over USB.

| R-26 item | Result |
|---|---|
| target board build | `-Werror`, zero warnings |
| flash/RAM budget | 1,569,400 B used, 396,676 B free (20.2%) |
| main-loop CPU timing | armed `PM.MaxT` 2530 µs against a 3000 µs overtime threshold |
| watchdog/scheduler margin | armed `PM.NLon` 0; one long loop in 112,000, disarmed, at log-file open |
| bench arm/disarm | 5/5 cycles, all four outputs symmetric at `MOT_SPIN_ARM` throughout |
| no-prop motor output | motor-test order A→1 B→4 C→2 D→3, standard QUAD-X, no cross-talk |
| Native↔Geo switch | `GFrm + NFrm == MFrm` held 21886/21886; 360,764 frames each attributed to exactly one writer across 24 ownership flips |
| fault fallback | hard-fault latch does not auto-recover; invalid configuration refused 3/3; latch clears only on Loiter re-entry |

Notes for repeating this gate:

- The scheduler criterion is `PM.NLon`, not `PM.MaxT` against the nominal period.
  `MaxT` is the maximum over the logging window and sits slightly above the nominal
  2500 µs in normal operation. The overtime threshold is `1e6 / rate_hz * 1.2`
  (`AP_Scheduler/PerfInfo.cpp`).
- `GEFR` is written with `WriteStreaming`, so consecutive rows span many frames. A
  row pair with both `dGFrm > 0` and `dNFrm > 0` is an interval containing a
  handover, not a same-frame double write. The invariants to check are
  `GFrm + NFrm == MFrm` and its per-interval form `dGFrm + dNFrm == dMFrm`.
- Arming with `LOIT_OPTIONS` bit 2 set while `GEO_OUT_EN` is 0 is refused outright
  rather than falling back silently to the Native rate PID. This is deliberate
  (`ModeLoiter::allows_arming`) and was confirmed on hardware here for the first
  time.
- The stick-driven differential check was not run. `AP_MotorsMatrix` is unmodified
  and the airframe had already flown the geometric controller in Loiter, Circle and
  RTL, which exercises the same signs far more strongly than a bench check. On a
  no-prop bench the altitude loop never sees a climb response, so collective thrust
  saturates; that is a bench artifact, not a controller property.

## Release follow-up

REL-FQ-01: Circle → Loiter transition flight-quality transient. **Characterized.**

The transient is the sum of two reference discontinuities at the handover, not a
control defect:

1. The Circle reference holds a centripetal acceleration of `omega^2 * r`, which
   the incoming Loiter reference drops to zero in one frame. This component is
   inherent to ending a circle and cannot be removed by parameters.
2. When the circle's tangential speed `omega * r` exceeds the Loiter reference
   speed limit, the Loiter reference enters overspeed and commands saturated
   deceleration from its first frame. The drag term
   `accel_xy_max * speed / speed_max` saturates immediately and bypasses the jerk
   shaper that the overspeed recovery term does use.

Evidence. X400 flight at 20 deg/s and 10 m: step 3.740 m/s², maximum attitude
error 0.536 rad, geometric roll command saturated, geometric ownership retained
across the handover. SITL reproduces it to three decimal places (3.741 m/s²,
0.497 rad). The original 4.258 m/s² figure corresponds to the suite's 25 deg/s
and 16 m configuration: centripetal 3.046 plus saturated braking 3.0, which are
perpendicular, giving 4.28.

Mitigation validated in SITL: raising the Loiter reference speed limit above the
circle's tangential speed removes component 2 entirely. At `GEO_LREF_VXY` 5.0 the
step falls to 1.196 m/s² (-68%), attitude error to 0.180 rad (-64%), and the roll
command no longer saturates. Component 1 remains and requires a C1-continuous
handover, which is tracked as deferred work below.

The mitigation is now flight-validated and adopted. X400 flight at `GEO_LREF_VXY`
8.0 on 2026-09-23 gives a 1.223 m/s² step against the 4.258 m/s² suite reference
(-71%), maximum attitude error 0.232 rad, and an unsaturated roll command.
`engineering-v1.0-rc2` ships `GEO_LREF_VXY` 8.0 and `GEO_LREF_BDLY` 1.0 in
`geo-p9r.param`. Component 1 is unchanged.

## Known residual: geometric attitude-loop lead

The geometric moment law is a PD on SO(3) with feedforward. Native's cascade is
equivalent to a PD on attitude error plus a second-derivative term
(`ATC_RAT_*_D`). Expanded against attitude error, Native is
`0.6075*e + 0.135*e' + 0.0036*e''` while the geometric law is `0.608*e + 0.137*e'`
at `GEO_MOM_NORM` 6.58, so the proportional and rate terms match and the
acceleration term has no geometric equivalent.

At 18.2 rad/s the Native compensator leads by 103.4 deg and the geometric one by
76.3 deg, a 27 deg difference. This is the measured residual behind the X400's
2.5-3.5 Hz hover mode.

Measured on the X400 over stick-centred hover windows, geometric 2-5 Hz gyro band
power against a Native control flown in the same session and configuration:

```text
GEO_MOM_NORM 6.58, GEO_LREF_VXY 5   geo 0.0510 / native 0.0213   2.4x   (19/11 windows)
GEO_MOM_NORM 6.58, GEO_LREF_VXY 8   geo 0.0383 / native 0.0291   1.3x   (97/76 windows)
```

The second pair is the better-grounded measurement. Native's own level rose
between the two sessions, so conditions were rougher, and the geometric level
still fell. Raising the reference speed limit appears to keep the Loiter
reference out of the drag and overspeed paths that bypass its jerk shaper, which
otherwise inject roughness the attitude loop then tracks.

Reducing `GEO_MOM_NORM` lowers the crossover to where less lead is required and
was validated as a mitigation, but it cannot close a missing term. Restoring the
lead is a controller capability change, is out of Full-Trajectory scope, and is a
prerequisite for any adaptive augmentation, which assumes a well-damped nominal
loop and adds its own phase lag.

## Next

Awaiting the next separately approved stage.

## Separate future track

SO(3) controller capability is not part of current Full-Trajectory work.

Candidate order:

```text
Guided_NoGPS
→ Stabilize
→ AltHold
```

A dedicated attitude/collective ownership contract is required first.

## Known deferred architecture work

- Compatibility publication generalization
- Generic selection/handoff cleanup beyond AUTO/RTL minimal state
- HeadingCommand header dependency cleanup
- centralized reference freshness policy
- rate-thread support
- terrain reference model
- partial-axis ownership
- HIL and a formal physical-flight validation campaign (bench gate H1 is complete)
