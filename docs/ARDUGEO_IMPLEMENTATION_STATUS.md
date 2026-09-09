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

## Guided Engineering v1.0 support matrix

| Support class | Guided paths | Decision |
| --- | --- | --- |
| Geo Active Supported | Initial entry hold, Pos, PosVelAccel, Pause, Guided Land | Pause keeps continuous Geo ownership |
| Geo Observer Only | WP | No Geo actuator ownership |
| Native by Design | TakeOff, VelAccel, Accel, Angle, Rate, direct thrust, Terrain, Guided_NoGPS | TakeOff preserves the Native `Rate_Only` heading semantics |

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

Awaiting the next approved stage. Circle has not started.

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
