# ArduGeo Architecture v1.2

**状态：** 架构与源码手术基线，供 Codex 规划、实施和代码审查使用
**目标车辆：** ArduPilot Copter / Multicopter
**目标：** Native controller 与 Full Geometric Controller 安全并存，并逐步扩展到更多控制 primitive；为后续 L1、自适应和抗扰增强提供稳定基础。

> 本文中的接口签名属于设计草案。Codex 必须先核对冻结源码中的真实类型和 API，再按责任边界实施；不得把伪代码机械复制进仓库。

---

## 1. 文档目的

当前 ArduGeo 已经证明了一条完整的内部几何控制路径能够在 ArduPilot Copter 中运行：

```text
Mode-specific reference generation
             ↓
PVA + yaw reference
             ↓
Geometric position controller
             ↓
SO(3) attitude controller
             ↓
Normalized actuator intent
             ↓
Vehicle-level ownership arbitration
             ↓
AP_Motors / mixer / HAL
```

但当前实现主要围绕 Guided 和 Loiter 纵向接入，Mode、reference generation、controller execution、fallback、arming、landing、logging 和 compatibility state 之间仍有较强耦合。

本架构的目的不是推倒 ArduGeo 重写，而是：

1. 冻结已经正确的几何控制和安全语义；
2. 建立 controller-neutral reference 边界；
3. 把通用状态采样、controller execution、ownership 和 handoff 收敛到 Copter vehicle orchestration；
4. 保留 ArduPilot 原生 Mode、WPNav、AutoYaw、AP_Motors 和安全基础设施；
5. 通过 observer-first 方式扩展 AUTO-WP、RTL 等原生导航路径；
6. 为后续 L1/自适应增强提供不侵入 Mode 的算法挂载点。

---

## 2. 基线与证据边界

### 2.1 冻结基线与仓库身份

```text
Working fork:
https://github.com/wanps/ArduGeo

Expected base branch:
ardugeo-pid

ArduGeo PID v1.2.0 release commit:
697c4da33c0cf0d7b888e9eb6ec2645b6749e91c

Previous v1.1.0 release commit:
7a907217175ace2842e2e0390a8f976ec6482d04

Expected ArduPilot base:
48644320dc6c4a19cc20c44051e56908eee97f4e

Expected remotes:
origin   = https://github.com/wanps/ArduGeo.git
upstream = https://github.com/TianhuaGao/ArduGeo.git
```

Codex 必须在本地验证 branch、HEAD、tag、remotes、submodule 状态和工作树；发现不一致时先报告，不能静默切换基线。后续施工必须从 v1.2.0 冻结点创建独立 feature branch，不得直接在 `ardugeo-pid` 基线分支持续堆改动。

### 2.2 v1.2.0 参数归属与迁移边界

v1.2.0 相比 v1.1.0 的主要结构变化不是控制公式，而是参数定义和存储归属模块化：

```text
43 个原先扁平定义的参数迁入消费它们的模块：
- Position PID: 13
- Attitude PID: 17
- Output Mapper: 4
- Setpoint Shaper: 6
- Yaw Shaper: 3

仍保持：
- 57 个公开 GEO_ 参数名称不变
- 57 个编译默认值不变
- GEO_OUT_EN 存储身份不变
- GEO_LREF_ 子组存储身份不变
```

v1.2.0 提供旧存储到新模块存储的自动前向迁移；迁移是幂等的，但不是双向镜像。降级到 v1.1.0 或更早构建前，必须导出参数；降级后清空并重新应用导出的参数，不能假设旧扁平存储仍与新值同步。

因此以下内容现在是冻结约束：

```text
不得重新扁平化参数定义
不得移动或复用现有参数 ID / subgroup identity
不得删除、绕过或改变旧→新迁移语义
不得把参数重构与 controller reference 重构混入同一 commit
不得声称降级安全，除非按导出/清空/重导入流程验证
```

基线审计必须识别并保留：

```text
Tools/autotest/ardugeo_parameter_module_upgrade.py
参数元数据与默认值检查
旧→新 EEPROM migration 检查
迁移后重启幂等性检查
```

### 2.3 论文与实现给出的事实

论文将当前实现划分为：

```text
Guided frontend
Loiter frontend
Common geometric feedback
Output mapper
Reference publication
Frame arbitration
AP_Motors actuation/safety
Logging/evidence
```

其中：

- Guided 与 Loiter 产生统一的 position、velocity、acceleration、yaw、yaw-rate reference；
- 几何位置控制器产生 specific force、collective、commanded attitude `Rc`、`Ωc` 与 `Ω̇c`；
- SO(3) attitude controller 产生 body moment；
- OutputMapper 只把 collective 与 moment 映射为 normalized roll/pitch/yaw/throttle intent，并不做 rotor-level allocation；
- AP_Motors 继续承担 spool、限幅、frame mixer 和硬件输出；
- 每个 main-rate frame 由 ownership arbiter 在 Geo 与 Native rate writer 之间二选一；
- Geo authorization 下降时，先在同帧完成 Native 状态同步，再运行 Native rate controller；
- 当前 full-geometric 支持边界主要是 Guided 的部分 submode 与 Loiter；其余 Copter modes 仍由 Native path 负责；
- 当前证据是 unit test、SITL 与 Gazebo nominal tracking，不等于 HIL、真实飞行安全或广泛扰动鲁棒性证明。

### 2.4 本文不声称

本文不声称：

```text
Geo 优于 Native PID
现有增益可直接用于目标飞机
当前实现已经达到物理飞行安全
所有 Copter Mode 都已经具备统一 PVA reference
AC_CustomControl 可直接承载 Full SE(3) Geo
```

---

## 3. 当前架构的正确部分

### 3.1 完整几何级联

当前 ArduGeo 不是 attitude-only backend，而是：

```text
X  = (x, v, R, Ω)
Xd = (xd, vd, ad, ψd, ψ̇d)

Xd + X
   ↓
Geometric Position PID
   ↓
specific force A
collective f
commanded attitude Rc
commanded rate Ωc
commanded angular acceleration Ω̇c
   ↓
SO(3) Attitude PID
   ↓
body moment M
   ↓
OutputMapper
   ↓
normalized R/P/Y/T intent
```

因此，Full Geo 同时替代原生位置反馈、姿态反馈与 rate PID 的主要控制作用，而不是只替代 `AC_AttitudeControl` 的末级 rate loop。

### 3.2 AP_Motors 边界正确

必须继续保持：

```text
Geo calculates f + M
        ↓
OutputMapper produces normalized intent
        ↓
---------------- integration boundary ----------------
        ↓
AP_Motors handles limits, spool, mixer and hardware path
```

Geo 不应自行实现 frame mixer 或 per-rotor allocator，除非未来另立项目并完成单独安全评审。

### 3.3 Frame-exclusive ownership 正确

当前核心安全语义：

```text
每一个 main-rate frame
          ↓
检查 authorization
      ↙           ↘
 Geo valid       otherwise
    ↓               ↓
Geo writer       Native writer
    ↘               ↙
       AP_Motors
```

不是 blend，也不是“两个都写，后写覆盖前写”。

### 3.4 Same-frame handoff 正确

Geo ownership falling edge：

```text
Geo authorization lost
          ↓
sync native collective/throttle state
rebuild/relax native attitude-rate state from current gyro
clear ownership / record reason
          ↓
Native rate controller runs in the same frame
```

这条时序必须视为框架协议，而不是 Geo 私有小技巧。

### 3.5 Two-phase schedule 正确且必须保护

当前实现近似为：

```text
Phase A — rate-control phase
consume cached Geo output from previous update
select Geo or Native writer
emit current-frame actuator intent

Phase B — mode/control update
sample latest state and reference
run geometric cascade
cache output for next eligible rate phase
```

重构时不得无意改变为“本轮算、本轮立即输出”。任何时序变化必须单独评审并重新验证稳定性、freshness 和 handoff。

---

## 4. 当前主要架构问题

### 4.1 Mode-specific reference 扩展会导致横向复制

当前 Guided 与 Loiter 各有独立 frontend。这对于建立 baseline 是合理的，但若继续按相同方法扩展：

```text
ModeAuto  → GeoAutoReference
ModeRTL   → GeoRTLReference
ModeLand  → GeoLandReference
ModeCircle→ GeoCircleReference
```

将等于重新实现半个 ArduCopter Mode/navigation 语义层。

目标应改为：

> 复用 ArduPilot 已经生成的原生飞行意图与轨迹 reference，Geo 只替换 feedback control。

### 4.2 `AC_Geometric_Target` 混合了多种责任

当前 target 同时表达：

```text
trajectory data
attitude data
controller capability selection
是否由 position 构造 attitude
是否 shape translation/yaw
是否 trajectory yaw
```

目标数据、reference policy 与 controller capability 不应长期混在一个结构体中。

### 4.3 Mode 承担了过多通用 controller integration

当前 Mode 基类及具体 Mode 可能同时负责：

```text
reference generation
状态采样
Geo controller execution
output freshness/finite 检查
部分 Native handoff
Mode lifecycle cleanup
```

其中状态采样、controller execution、通用 handoff 应迁到 Copter orchestration。

### 4.4 Geo-specific API 泄漏到通用 Mode 与消费者

长期应避免所有 Mode、GCS、land detector、surface tracking 等模块直接判断：

```cpp
geometric_control.enabled()
```

它们应该消费更通用的状态：

```text
reference active?
active actuator owner?
active control path healthy?
```

### 4.5 `AC_CustomControl` 与 Full Geo 层级不同

`AC_CustomControl` 更接近 attitude/rate-output 级扩展点；Full Geo 是 trajectory → position → SO(3) → actuator intent 的完整级联。

因此长期不是把 Full Geo 强行塞入一个 attitude backend，而是采用 capability-based architecture。

---

## 5. Target Architecture v1.2

```text
┌──────────────────────────────────────────────────────┐
│ ArduPilot Flight Modes                               │
│ Guided / Loiter / Auto / RTL / Land / Stabilize... │
└──────────────────────┬───────────────────────────────┘
                       │
                       │ command semantics + lifecycle
                       ▼
┌──────────────────────────────────────────────────────┐
│ Native / Mode-specific Reference Generation          │
│ Guided manager / Loiter ref / WPNav / Takeoff        │
│ Land reference / AutoYaw / pilot mapping             │
└──────────────────────┬───────────────────────────────┘
                       │
                       ▼
              Controller-neutral Reference
              ┌────────────────────────────┐
              │ capability/type            │
              │ P / V / A                  │
              │ heading command            │
              │ or attitude/rate reference │
              │ frame / validity           │
              │ sequence / timestamp       │
              └─────────────┬──────────────┘
                            │
                 ┌──────────┴──────────┐
                 │                     │
                 ▼                     ▼
        Native Control Stack      Geometric Family
        Pos → Att → Rate          Full SE(3) cascade
                                  or SO(3)-only path
                 │                     │
                 └──────────┬──────────┘
                            ▼
             Copter Vehicle-level Selection
             ┌────────────────────────────┐
             │ authorization              │
             │ output freshness/finite    │
             │ one-frame one-writer       │
             │ same-frame handoff         │
             │ fault latch / diagnostics  │
             └─────────────┬──────────────┘
                           ▼
                       AP_Motors
                           ▼
                      Mixer / HAL

Controller-neutral reference / Geo commanded state
                           │
                           ▼
               Compatibility Publication
          AC_PosControl / AC_AttitudeControl caches
          without running Native feedback loops
```

---

## 6. Controller capability model

不要创建一个万能：

```cpp
class IController {
    virtual update(OneUniversalReference) = 0;
};
```

Native ArduPilot、Full SE(3) Geo、attitude-only custom controller 与 Acro rate path 不是同一能力层级。

目标能力族：

```text
FullTrajectoryController
    input: P/V/A + heading
    output: normalized actuator intent

AttitudeController
    input: attitude + angular rate/acceleration reference
    output: normalized actuator intent

RateController / Direct path
    input: body-rate/direct intent
    output: normalized actuator intent
```

当前首要实现对象是：

```text
FullTrajectoryController = ArduGeo Position + SO(3) cascade
```

Stabilize、Acro 等模式后续使用合适的 capability，不为统一接口伪造 PVA。

---

## 7. Controller-neutral reference design

### 7.1 设计原则

Reference 表达：

> “Mode/navigation 已经解析完成后，飞机应该如何运动。”

Reference 不表达：

```text
Geo 是否需要启用
输出归谁
是否允许 arming
controller gain
是否应该运行 Native feedback
```

### 7.2 Draft types

以下为设计草案，需 Codex 核对冻结代码中的真实类型：

```cpp
#pragma once

#include <AP_Math/AP_Math.h>
#include "AC_AttitudeControl.h"

enum class AC_ControlReferenceFrame : uint8_t {
    LOCAL_NED = 0,
    // TERRAIN and other frames are intentionally unsupported
    // until their semantics are explicitly implemented.
};

enum class AC_ControlReferenceCapability : uint8_t {
    NONE = 0,
    TRAJECTORY,
    ATTITUDE,
    RATE,
};

struct AC_ControlReferenceMeta {
    AC_ControlReferenceFrame frame{AC_ControlReferenceFrame::LOCAL_NED};
    AC_ControlReferenceCapability capability{AC_ControlReferenceCapability::NONE};
    uint32_t timestamp_ms{0};
    uint32_t sequence{0};

    bool valid{false};
    bool is_finite() const;
};

struct AC_TrajectoryReference {
    AC_ControlReferenceMeta meta{};

    Vector3p position_ned_m{};
    Vector3f velocity_ned_ms{};
    Vector3f acceleration_ned_mss{};

    AC_AttitudeControl::HeadingCommand heading{};

    bool is_valid() const;
};

struct AC_AttitudeReference {
    AC_ControlReferenceMeta meta{};

    Quaternion attitude_body_to_ned{};
    Vector3f angular_velocity_body_rads{};
    Vector3f angular_acceleration_body_radss{};

    bool is_valid() const;
};
```

### 7.3 Why `HeadingCommand` matters

Yaw intent 不永远是固定角度。原生语义可能是：

```text
Angle + Rate
Rate only
固定 yaw
ROI
look ahead
face waypoint
pilot rate
weathervane
```

因此 reference 必须保留 yaw angle、yaw rate 与 heading semantic，不能只存两个裸 `float` 后丢失类型。

### 7.4 Validity and freshness

Reference 至少需要：

```text
finite check
frame check
capability check
timestamp / age
source lifecycle reset
```

第一版不得用默认零值假装缺失字段有效。

### 7.5 Reference policy separate from data

这些属于 frontend policy，不属于 reference 数据：

```text
shape_translation
shape_heading
heading_from_trajectory
brake profile
jerk limit source
```

可在过渡期保留单独的：

```cpp
struct AC_GeometricReferencePolicy {
    bool shape_translation;
    bool shape_heading;
    bool heading_from_trajectory;
};
```

最终 shaped reference 进入 Geo Core 时，不再携带“还要不要 shape”的模糊状态。

---

## 8. Geometric controller boundary

### 8.1 Must remain true

```text
Input:
state + complete reference + dt

Output:
controller output / normalized actuator intent + diagnostics

Side effects not allowed:
AP_Motors write
Mode transition
arming decision
mission interpretation
```

### 8.2 Draft façade APIs

```cpp
class AC_GeometricControl {
public:
    void update_trajectory(
        const AC_Geometric_State& state,
        const AC_TrajectoryReference& reference,
        float dt);

    void update_attitude(
        const AC_Geometric_State& state,
        const AC_AttitudeReference& reference,
        float dt);

    const AC_Geometric_Output& output() const;

    void reset();
    void set_enabled(bool enabled);
    bool enabled() const;
};
```

过渡期保留旧 `update(state, AC_Geometric_Target, dt)`，通过 adapter 调用新 API，避免立即破坏 Guided/Loiter baseline。

### 8.3 Internal extension point for L1/adaptive augmentation

未来增强放在 Geo Core 内，而不是 Mode：

```text
Trajectory reference
       ↓
Nominal geometric position control
       ↓
nominal force / model quantities
       ↓
L1 / adaptive / disturbance augmentation
       ↓
commanded force / attitude
       ↓
SO(3) attitude control
       ↓
optional moment augmentation
       ↓
OutputMapper
```

具体增强放在 force 侧、moment 侧或两者都用，必须由对应算法设计决定；架构只保证它不需要重新实现 Flight Mode。

---

## 9. Reference composition ownership

### 9.1 Do not make WPNav depend on Geo

禁止：

```cpp
AC_WPNav::update(...)
{
    ...
    geometric_control.set_target(...);
}
```

Navigation library 只能产生自己的 trajectory state；Geo 消费由上层 composition/orchestration 完成。

### 9.2 First implementation: lightweight Mode/Copter composition

第一版不急于创建重量级 `ReferenceManager`。建议：

```cpp
class Mode {
public:
    virtual bool get_trajectory_reference(
        AC_TrajectoryReference& reference) const
    {
        return false;
    }

    virtual bool get_attitude_reference(
        AC_AttitudeReference& reference) const
    {
        return false;
    }
};
```

或由 Copter helper 从当前 Mode/Native generators 汇总。最终选择需根据冻结代码减少耦合，但必须满足：

```text
Mode knows command semantics
Copter knows controllers and ownership
Geo does not know Mode
```

### 9.3 AUTO-WP first observer source

普通 non-terrain AUTO waypoint 路径优先复用：

```text
WPNav / AC_PosControl desired PVA
+
AutoYaw final HeadingCommand
```

第一版允许 Native position/attitude 继续 shadow calculate；Geo 只 observer，不写 Motors。

后续若证明确需去除 shadow calculation，再单独评审是否将 WPNav 拆成：

```text
update_reference()
run_native_feedback()
```

这不是第一轮任务。

---

## 10. Vehicle-level geometric integration bridge

当前从 Mode 移出的通用职责：

```text
sample EKF/AHRS state
build AC_Geometric_State
run Geo controller
publish compatibility target
cache timestamped Geo output
validate finite/freshness
```

建议作为 `Copter` helper，暂不创建持有大量 Copter 引用的独立 Manager：

```cpp
bool Copter::update_geometric_controller(
    const AC_TrajectoryReference& reference,
    float dt);

bool Copter::sample_geometric_state(
    AC_Geometric_State& state) const;

void Copter::clear_geometric_control_path();
```

建议源码位置：

```text
ArduCopter/geometric_control.cpp
```

名称可调整，但职责不可重新塞回所有 Mode。

---

## 11. Ownership and selection protocol

### 11.1 Keep selection at vehicle-level

仲裁继续位于 Native rate writer 与 `AP_Motors` 之间的 Copter 主控制路径。

不要：

```text
把仲裁移入 AC_GeometricControl
把 AP_Motors 指针交给 controller
现在就创建复杂 ControllerManager 注册系统
```

### 11.2 Draft state

```cpp
enum class AC_ActuatorOwner : uint8_t {
    NATIVE = 0,
    GEOMETRIC,
};

enum AC_ControlFailureFlag : uint16_t {
    FAIL_NONE             = 0,
    FAIL_MODE_UNSUPPORTED = 1U << 0,
    FAIL_OUTPUT_DISABLED  = 1U << 1,
    FAIL_RATE_THREAD      = 1U << 2,
    FAIL_DISARMED         = 1U << 3,
    FAIL_CONTROLLER_OFF   = 1U << 4,
    FAIL_OUTPUT_STALE     = 1U << 5,
    FAIL_OUTPUT_NONFINITE = 1U << 6,
    FAIL_MODE_UNSAFE      = 1U << 7,
    FAIL_REFERENCE_INVALID= 1U << 8,
};

struct AC_ControlSelection {
    AC_ActuatorOwner owner{AC_ActuatorOwner::NATIVE};
    uint16_t failure_flags{FAIL_NONE};
    bool handoff_to_native{false};
};
```

实际 bit layout 必须兼容现有日志或做显式 migration，不得静默改变现有证据解释。

### 11.3 Draft helpers

```cpp
AC_ControlSelection Copter::evaluate_control_selection() const;

void Copter::perform_native_handoff(uint16_t failure_flags);

void Copter::write_geometric_actuator_intent();
```

`run_rate_controller_main()` 仍是最终二选一的位置，目标只是在不改变时序的前提下缩小函数、使规则可测试。

### 11.4 Handoff split

通用 handoff 由 Copter 负责：

```text
sync throttle/collective cache
relax/rebuild native attitude-rate state
clear owner
record failure reason
reset/disable Geo path
```

Mode-specific hook 仅负责：

```text
clear Mode private reference state
clear lifecycle flags
rebuild Mode-native targets if needed
set Mode-specific fault latch
```

Draft：

```cpp
class Mode {
public:
    virtual void on_alternate_to_native_handoff() {}
};
```

---

## 12. Compatibility publication

### 12.1 Purpose

Geo active 时，Native feedback 可不运行，但其他 ArduPilot 模块仍可能读取：

```text
position desired/target
velocity/acceleration reference
attitude target
heading state
controller activity/reference activity
```

所以需要：

```text
Geo/controller reference
        ├── Geo feedback
        └── Native compatibility caches
```

### 12.2 Required semantic distinction

```text
Native controller active
!=
External/controller reference active
```

例如 `NE_is_active()` 不应因为 Geo publication 而伪装成 Native NE feedback 已运行。

### 12.3 Draft generic API

```cpp
enum class AC_ReferenceOwner : uint8_t {
    NONE = 0,
    NATIVE,
    GEOMETRIC,
};

bool AC_PosControl::publish_controller_reference_NED_m(
    const AC_TrajectoryReference& reference,
    AC_ReferenceOwner owner);

void AC_PosControl::clear_controller_reference(
    AC_ReferenceOwner owner);

AC_ReferenceOwner AC_PosControl::reference_owner() const;

bool AC_AttitudeControl::publish_external_attitude_reference(
    const AC_AttitudeReference& reference,
    AC_ReferenceOwner owner);
```

过渡期：旧 `publish_external_reference_*` / `set_external_attitude_target()` 调用 generic API，不能一步删除。

### 12.4 Clear rules

必须在以下情况清除 external ownership/reference：

```text
Mode exit
controller disabled
handoff to Native
reference source reset
Native feedback重新接管对应 axis
arming cancellation/disarm
```

---

## 13. Arming protocol

当用户明确请求 Active Geo 时，arming 前必须重新验证：

```text
Mode/submode supported
reference prepared and valid
controller enabled
cached output fresh
output finite
mode lifecycle ready
rate-thread unsupported condition absent
```

准备失败时：

```text
拒绝 arming并给出明确原因
```

不得静默退回 Native 后继续 arm，因为这会掩盖错误配置。

建议入口：

```cpp
bool Copter::prepare_active_control_path_for_arming(
    AP_Arming::Method method,
    char* failure_msg,
    size_t failure_msg_len);
```

实际错误传递方式需遵循现有 ArduPilot arming API。

---

## 14. Safe defaults

最终产品策略：

```text
Native output ownership = default
Geo observer = explicit opt-in
Geo active output = separate explicit opt-in
```

不应因为 fresh parameter store 而自动让实验 controller 接管支持的 Guided/Loiter 路径。

如果当前参数默认与该策略不同，修改必须是独立 diff，并配套：

```text
parameter default tests
stored parameter compatibility review
documentation update
observer-only activation test
active-output activation test
```

注意：源码默认变化不会覆盖已有 EEPROM 存储值；测试与文档必须说明这一点。

---

## 15. Source surgery map

### 15.1 Keep algorithm core

| Area | Decision | Notes |
|---|---|---|
| `AC_Geometric_Position_PID.*` | KEEP | 不改控制公式，除非有独立算法任务 |
| `AC_Geometric_Attitude_PID.*` | KEEP | 不改 SO(3) error/moment 公式 |
| `AC_Geometric_OutputMapper.*` | KEEP | 保持 AP_Motors integration boundary |
| Existing unit tests | KEEP + EXTEND | 新接口不得降低现有覆盖 |

### 15.2 Generalize controller/reference boundary

| Area | Decision | Target |
|---|---|---|
| `AC_Geometric_Types.h` | MIGRATE | 拆出 controller-neutral reference |
| `AC_GeometricControl.*` | KEEP FACADE + ADD API | 分清 shaped reference 与 core update |
| Guided/Loiter shapers | KEEP | 作为 frontend，不塞进通用 control reference |
| New reference adapter | ADD | 兼容 legacy target |

### 15.3 Vehicle-level orchestration

| Area | Decision | Target |
|---|---|---|
| `ArduCopter/Attitude.cpp` | KEEP BOUNDARY, SHRINK | 仍负责最终 one-writer selection |
| `ArduCopter/Copter.h` | GENERALIZE | owner/gate/handoff state最小化 |
| `ArduCopter/geometric_control.cpp` | ADD | state sampling + Geo execution bridge |
| `ArduCopter/controller_selection.cpp` | ADD | gate evaluation + handoff helper |

### 15.4 Mode layer

| Area | Near-term | Long-term |
|---|---|---|
| `mode.h/.cpp` | 兼容现有 hook | 改成 capability/reference/handoff generic hook |
| `mode_guided.cpp` | 行为冻结 | 只保留 Guided semantics/reference/lifecycle |
| `mode_loiter.cpp` | 行为冻结 | 只保留 Loiter semantics/reference/lifecycle |
| `mode_auto.cpp` | observer-only later | 复用 WPNav + AutoYaw reference |
| `mode_rtl.cpp` | 暂不 active | 按 phase逐段接入 |

### 15.5 Compatibility and consumers

| Area | Decision |
|---|---|
| `AC_PosControl.*` external reference | KEEP + GENERALIZE |
| `AC_AttitudeControl.*` external target | KEEP + GENERALIZE |
| GCS/MAVLink consumers | 改读 generic reference/owner状态 |
| Land detector / surface / baro helpers | 不应知道具体 Geo 算法 |
| Logging | 保留现有 evidence，并逐步加 generic owner/reference logs |

### 15.6 Do not touch in first migration

```text
AP_Motors/*
EKF/AHRS/*
HAL/*
WPNav S-Curve/Spline equations
AutoYaw semantic logic
Native PID formulas
rate-thread path
GEO_* parameter ID、module ownership、subgroup identity 与 migration logic
```

---

## 16. Migration plan

每一步都是独立、可构建、可回退的逻辑变更。

### C0 — Baseline freeze

不修改代码。

输出：

```text
verified HEAD/base
working-tree status
build commands
unit-test commands
SITL/autotest commands
baseline parameters/log schema
architecture-to-source name differences
```

### C1 — Safe-default audit

先只审计：

```text
GUID_OPTIONS / LOIT_OPTIONS / GEO_OUT_EN 当前默认
fresh parameter behavior
stored parameter behavior
arming behavior
```

如需改默认，作为独立变更，不与接口重构混合。

### C2 — Controller-neutral reference types

新增：

```text
libraries/AC_AttitudeControl/AC_ControlReference.h
reference validity/finite/frame tests
legacy AC_Geometric_Target adapter
```

`AC_ControlReference.h` 不得放在 `libraries/AC_GeometricControl/` 下。

验收：

```text
无 Mode 行为变化
无 AP_Param 变化
无 motor ownership 变化
无日志语义变化
```

### C3 — Compatibility publication generalization

目标：

```text
owner + timestamp
new generic API
old API delegates to new API
Native active 与 external reference active 明确分离
```

### C4 — Geometric integration bridge

将通用 state sampling 与 Geo execution 从 Mode 移到 Copter helper。

验收：

```text
Guided/Loiter reference 数值不变
Geo cached output时序不变
motor ownership不变
```

### C5 — Selection/handoff helper extraction

收敛：

```text
gate evaluation
failure flags
falling-edge detection
native handoff
Geo motor write helper
```

最终二选一位置仍在 `run_rate_controller_main()`。

### C6 — Guided/Loiter migrate through adapter

只改接口，不改生命周期：

```text
takeoff
pause
landing
ground idle
pilot brake
fault latch
arming preparation
```

现有 tests/log evidence 必须继续通过。

### C7 — AUTO-WP observer-only

初始范围严格限制：

```text
normal multicopter
non-terrain
main-thread rate path
ordinary WP/Spline control primitive
Geo never writes Motors
```

reference 来源：

```text
WPNav/PosControl desired PVA
+
AutoYaw final HeadingCommand
```

必须证明：

```text
Geo computed frames > 0
Geo output-write frames = 0
Native rate frames = main frames
```

### C8 — AUTO-WP active ownership

只有 C7 证据通过并单独批准后才能实施。

必须覆盖：

```text
stale output
nonfinite output
Mode exit
mission item transition
WP completion
Geo disable
pilot intervention按原生语义
same-frame Native handoff
```

### Later — RTL and other modes

RTL 不按一个布尔“支持/不支持”，而按 phase：

```text
initial climb
return home
loiter at home
final descent
land
```

每个 phase 重复 observer-first 流程。

---

## 17. Validation matrix

### 17.1 Global invariants

| Scenario | Required result |
|---|---|
| Geo disabled | Native behavior unchanged |
| Observer-only | Geo computes; zero motor writes |
| Geo active | one and only one writer per main-rate frame |
| Output stale | same-frame handoff to Native |
| NaN/Inf | same-frame handoff + fault latch |
| Unsupported mode/submode | Native ownership |
| Rate thread | current version denies Geo ownership |
| Mode exit | clear reference, owner and Mode-private lifecycle state |
| Native resumes | throttle/gyro/controller state synchronized |
| Arming with active Geo | prepared reference and fresh finite output required |
| Parameter migration | v1.1→v1.2 forward migration、57 个公开名称/默认值、存储 identity 与幂等重启保持可验证 |
| Logging | can prove reference, gate, owner, handoff and frame counts |

### 17.2 Frame ownership proof

Active Geo bounded interval should satisfy the equivalent of：

```text
ΔMainFrames = ΔGeoFrames
ΔNativeFrames = 0
```

Observer interval should satisfy：

```text
ΔGeoComputedFrames > 0
ΔGeoWriteFrames = 0
ΔNativeFrames = ΔMainFrames
```

只看 decimated log 或“最终一次输出”不能证明区间内每帧 ownership。

### 17.3 Required test layers

```text
unit tests
SITL regression
mode lifecycle tests
fault injection
DataFlash ownership evidence
board compile for target hardware
HIL plan
physical-flight test plan
```

真实飞行前必须先完成 Native baseline、tethered/contained test、低风险 hover 和明确的人工切回策略；具体飞行试验协议另立文档。

---

## 18. Fork and branch workflow

所有施工都在 `wanps/ArduGeo` fork 中完成，并保留作者仓库作为 `upstream`：

```text
upstream/ardugeo-pid
        ↓ 同步 release 和后续作者修复
origin/ardugeo-pid
        ↓ 只作为本项目冻结基线，不直接施工
feature/c1-control-reference
feature/c2-reference-publication
feature/c3-integration-bridge
...
```

推荐纪律：

```text
一个 migration stage = 一个 feature branch
一个逻辑边界 = 一个或少量可独立审查 commit
每个阶段通过本地构建/测试后推送 origin
在自己的 fork 内开 PR：feature/* → ardugeo-pid
PR 链接、commit SHA、git diff 和测试结果用于外部架构复核
```

禁止：

```text
直接 force-push 改写作者 tag
直接在 ardugeo-pid 基线分支试验
把多个 C 阶段一次性合并成巨型 diff
未经验证就同步作者后续变更并继续施工
```

---

## 19. First Codex task

Codex 第一次启动只执行 C0，不改代码。

建议提示词：

```text
请先阅读仓库根目录 AGENTS.md，以及：
- docs/ARDUGEO_CODEX_RULES_V1_2.md
- docs/ARDUGEO_ARCHITECTURE_V1_2.md

当前不要修改任何文件，也不要创建 commit。

只完成 C0 Baseline Freeze：
1. 报告当前仓库 URL、分支、HEAD、tag、工作区和 submodule 状态；
2. 验证 origin 指向 wanps/ArduGeo，upstream 指向 TianhuaGao/ArduGeo；
3. 验证 ardugeo-pid 当前冻结 HEAD 为
   697c4da33c0cf0d7b888e9eb6ec2645b6749e91c；
4. 验证预期 ArduPilot base 为
   48644320dc6c4a19cc20c44051e56908eee97f4e；
5. 核对 v1.2.0 的参数模块 ownership、43 个迁移参数、57 个公开参数名/默认值、
   GEO_OUT_EN 与 GEO_LREF_ storage identity，以及迁移测试入口；
6. 找出现有 Waf build、91 个 geometric unit tests、16 项 SITL regression、
   参数 metadata/default 与 EEPROM migration 测试的真实命令或入口；
7. 核对架构文档所列关键文件、类和方法，记录真实名称差异；
8. 审计当前 Geo output 默认参数与 fresh-parameter 行为，但不要修改；
9. 为下一阶段“controller-neutral reference types”输出精确到文件、类型、调用者和测试的计划；
10. 证明本阶段没有修改任何文件，然后停止等待批准。

不得顺手修复、重构、格式化、创建分支或提交代码。
任何没有实际运行的测试必须明确标为未运行，禁止推测为通过。
```

---

## 20. Architecture decisions frozen in v1.2

已经冻结：

```text
Geo Core 不写 AP_Motors
AP_Motors/mixer/HAL继续原生
每帧单 writer
Geo→Native same-frame handoff
Copter vehicle-level ownership
controller-neutral reference
capability-based controller family
Mode负责语义与生命周期
Copter负责状态采样、controller execution和通用handoff
Compatibility publication不得运行Native feedback
Guided/Loiter先作为行为baseline
新Mode必须observer-first
实验输出默认不应自动接管
v1.2.0 模块化参数 ownership 与 v1.1→v1.2 migration 语义必须保持
```

尚未冻结：

```text
reference header最终目录和命名
是否最终拆出独立 GeometricCascade 类
rate-thread支持方案
Terrain reference正式表示
通用多controller注册机制
上游提交拆分与命名
```

这些开放项不得阻塞 C0–C7 的小步迁移，但也不得被 Codex 擅自“顺便决定”。

---

## 21. Definition of Done for the architecture migration stage

架构迁移阶段完成，不等于全模式和真实飞行完成。其完成条件是：

```text
1. Guided/Loiter现有功能通过新reference与integration bridge运行；
2. Native disabled/observer/active路径边界清楚且可测试；
3. Ownership与handoff从散落逻辑收敛为vehicle-level协议；
4. Compatibility publication拥有明确owner/timestamp/reset语义；
5. AUTO-WP完成observer-only并证明零电机写入；
6. 全部已有unit/SITL regression不退化；
7. 目标板固件可编译；
8. 每项支持范围按Mode+SubMode/Primitive记录；
9. 日志可证明每帧owner与handoff；
10. 没有修改AP_Motors、EKF、HAL和Native PID控制公式。
```

达到这些条件后，才进入 AUTO-WP active ownership、RTL phase-by-phase 和 L1/adaptive augmentation 的后续开发。
