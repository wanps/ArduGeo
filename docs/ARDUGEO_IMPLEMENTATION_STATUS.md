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

## Other current verified coverage

```text
Loiter supported lifecycle        Geo active
AUTO WP / Spline                  Geo observer + active
RTL Return Home                   Geo observer + active
RTL Loiter At Home                Geo observer + active
```

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
- hardware/HIL/physical-flight validation
