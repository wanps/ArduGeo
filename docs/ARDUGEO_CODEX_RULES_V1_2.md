# ArduGeo Codex Project Rules v1.2

本文件是仓库现有根目录 `AGENTS.md` 的 ArduGeo 项目补充规则，不替代 ArduPilot 通用贡献规范。任何 AI Agent、Codex 或人工开发者在修改 ArduGeo 相关代码前，都必须同时阅读：

- 根目录 `AGENTS.md`
- `docs/ARDUGEO_ARCHITECTURE_V1_2.md`
- 本文件

本项目不是“把几何控制器尽快塞进更多 Mode”的功能堆叠任务，而是把 ArduGeo 逐步迁移成一套**可扩展、可验证、可回退、默认安全**的 ArduPilot 内部控制架构。

---

## 1. 工作基线

期望研究基线：

- 工作 fork：`https://github.com/wanps/ArduGeo`
- 基线分支：`ardugeo-pid`
- ArduGeo PID v1.2.0 冻结版本：`697c4da33c0cf0d7b888e9eb6ec2645b6749e91c`
- 上一发布版本：`7a907217175ace2842e2e0390a8f976ec6482d04`
- 预期 ArduPilot 基线：`48644320dc6c4a19cc20c44051e56908eee97f4e`
- 预期 `origin`：`https://github.com/wanps/ArduGeo.git`
- 预期 `upstream`：`https://github.com/TianhuaGao/ArduGeo.git`

Agent 必须先在本地仓库验证：

```text
当前分支
当前 HEAD
冻结版本是否存在
上游基线是否存在
工作区是否干净
现有构建和测试入口
```

若本地事实与上述记录不一致，先报告差异，不得自行猜测或静默换基线。

---

## 2. 不可违反的控制安全约束

以下约束优先级高于代码整洁、性能优化和功能扩展。

### I-01：Geo Core 不直接写电机

`AC_GeometricControl`、位置控制器、SO(3) 姿态控制器及未来 L1/自适应模块，只能产生 controller output 或 normalized actuator intent。

禁止它们直接调用或绕过边界写入：

```text
AP_Motors
SRV_Channel
HAL PWM/DShot output
具体机架 mixer
```

### I-02：每个 main-rate frame 只能有一个 actuator-intent writer

当前阶段只允许二选一：

```text
Native writer
或
Geometric writer
```

禁止同一帧双写、依赖调用顺序覆盖、隐式 blend 或“两个都算完后最后一个生效”。

### I-03：Controller selection 不是 blending

除非以后有单独评审通过的设计，本阶段不得实现 Native/Geo 输出渐变混合。

### I-04：Geo → Native 必须同帧原子回退

当 Geo ownership 失效时，必须在 Native rate controller 本帧运行之前完成：

```text
collective/throttle cache 同步
Native attitude/rate state 重建或放松
owner 清除
handoff reason 记录
```

不得留下“本帧无人写输出”或“下一帧才回退”的空窗。

### I-05：所有异常均 fail closed

至少包括：

```text
unsupported mode/submode
未 armed / interlock 不满足
rate-thread path
controller disabled
output stale
NaN / Inf / 非有限输出
mode-specific unsafe
reference invalid / frame 不匹配
```

失效时回到 Native；hard fault 应保持 latch，直到通过明确操作或模式生命周期重新确认，禁止在边界上高速抖动切换。

### I-06：Compatibility publication 不等于运行 Native feedback

向 `AC_PosControl`、`AC_AttitudeControl` 等发布 external/controller reference 时，只能更新公开 target/cache/bookkeeping。

禁止为了更新遥测、landing logic 或 heading state 而偷偷调用 Native position/attitude/rate feedback。

### I-07：第一阶段禁止修改底层安全链

除非任务中明确批准，禁止修改：

```text
AP_Motors 及各机架 mixer
EKF / AHRS 算法
HAL / PWM / DShot
Native position PID 数学
Native attitude/rate PID 数学
WPNav 的 S-Curve / Spline 数学
AutoYaw 的 ROI / FIXED / LOOK_AHEAD 等语义
rate-thread 输出路径
```

---

## 3. 目标责任边界

### Flight Mode 负责

```text
命令语义
Mode 生命周期
reference generation/composition
Mode-specific authorization
Mode-specific fallback cleanup
```

### Copter vehicle orchestration 负责

```text
状态采样
controller 执行
reference/output freshness 与 finite 检查
actuator ownership
same-frame handoff
最终选择谁写 AP_Motors
```

### Geometric controller 负责

```text
PVA/heading 或 attitude reference 跟踪
specific force / commanded attitude / moment 构造
normalized actuator intent 计算
内部积分器、滤波器与算法状态
```

### AP_Motors 继续负责

```text
spool state
output limiting
frame mixing
per-motor command
hardware output path
```

---

## 4. 依赖方向

必须保持：

```text
Mode / WPNav / AutoYaw
          ↓
controller-neutral reference
          ↓
Copter orchestration
       ↙       ↘
 Native       Geo
       ↘       ↙
 ownership selection
          ↓
       AP_Motors
```

禁止出现：

```text
AC_WPNav → AC_GeometricControl
AutoYaw → AC_GeometricControl
AC_GeometricControl → Mode
AC_GeometricControl → AP_Motors
AP_Motors → 某个具体 controller
```

---

## 5. 接口与实现原则

1. 新的 reference 应当 controller-neutral，不以 `Geo` 命名。
2. Full trajectory、attitude-only、rate/direct control 是不同 capability，不强行塞进一个万能虚接口。
3. 第一版优先使用静态组合、普通结构体和明确调用；不要引入动态插件加载。
4. 控制主路径禁止新增无必要的动态内存、异常、RTTI 或不可预测阻塞。
5. 第一版只允许明确支持的 reference frame；不能把 Terrain、offset frame 或缺失字段静默假装成 Local NED 完整 PVA。
6. 新 API 必须有清楚的 owner、validity、timestamp/freshness 与 reset/clear 语义。
7. 现有 `GEO_*` AP_Param ID、类型和持久化布局不得随意重排或复用。
8. 不得为了减少 diff 而牺牲显式的安全状态或错误报告。

### v1.2.0 参数规则

v1.2.0 已将 43 个参数迁入 Position PID、Attitude PID、Output Mapper、Setpoint Shaper 和 Yaw Shaper 等消费模块，同时保持 57 个公开 `GEO_` 参数名称和编译默认值不变。

以下操作禁止与 controller reference 施工混合，且未经单独批准不得执行：

```text
重新扁平化参数定义
移动或复用现有 AP_Param ID
改变 GEO_OUT_EN 或 GEO_LREF_ storage identity
删除或绕过 v1.1→v1.2 migration
修改迁移后幂等重启行为
声称固件降级会自动把新存储镜像回旧存储
```

必须保留并在相关变更后执行或审计：

```text
Tools/autotest/ardugeo_parameter_module_upgrade.py
57/57 public parameter name/default checks
45/45 old-to-new EEPROM migration checks
45/45 post-migration idempotent restart checks
```

固件降级测试必须遵循：导出参数 → 降级 → wipe → 重导入；不能把旧扁平存储中可能残留的值当成可靠回滚。

---

## 6. 当前文件处理策略

### KEEP / 保护行为

```text
libraries/AC_GeometricControl/AC_Geometric_Position_PID.*
libraries/AC_GeometricControl/AC_Geometric_Attitude_PID.*
libraries/AC_GeometricControl/AC_Geometric_OutputMapper.*
ArduCopter/Attitude.cpp 中的 frame-exclusive ownership 时序
现有 Geo→Native same-frame handoff 语义
现有 compatibility publication 语义
Guided/Loiter 已验证的生命周期行为
现有 DataFlash ownership/lifecycle 证据
```

### GENERALIZE / 逐步收敛

```text
AC_Geometric_Target
Mode 基类中的 Geo-specific hooks
Mode::run_geometric_observer()
Mode::handle_geometric_motor_output_fallback()
AC_PosControl external-reference API 的命名与 owner 元数据
AC_AttitudeControl external-target API 的命名与 owner 元数据
Copter.h 中散落的 controller-selection 状态
```

### DO NOT TOUCH / 首轮禁止

```text
AP_Motors/*
EKF/AHRS/*
HAL/*
WPNav trajectory equations
AutoYaw mission/pilot semantics
Native PID 控制公式
rate-thread control path
v1.2 参数 module ownership、storage identity 与 migration logic
```

---

## 7. 默认安全策略

正式产品基线应满足：

```text
Native controller 默认拥有输出
实验 controller 默认不主动接管电机
Observer-only 与 Active output 分开授权
```

Active Geo 至少需要同时满足：

```text
mode-level request
controller-level enable
reference valid/fresh
output valid/fresh/finite
armed/interlock/mode-safe
main-thread supported path
```

如果当前源码默认会主动启用实验输出，Agent 必须先报告现状，并将安全默认修改作为独立、可审查的变更，不要混入接口重构。

---

## 8. 开发流程

每次只执行一个小任务。默认流程：

```text
阅读本文件与架构文档
→ 检查本地事实
→ 输出精确计划
→ 等待批准
→ 实施一个逻辑变更
→ 构建/测试
→ 提交 diff、测试结果、风险和未解决项
→ 停止
```

除非用户明确授权，不得一次性完成多个迁移阶段。

### 每个任务开始前必须报告

```text
当前分支与 HEAD
工作区状态
本次允许修改的文件
明确禁止修改的文件
预期不变的飞行行为
计划运行的构建/测试
```

### 每个任务结束时必须报告

```text
修改文件清单
关键控制流变化
git diff --stat
实际运行的命令
通过/失败的测试
未运行测试及原因
对 actuator ownership 的影响
对 AP_Param 的影响
对 Native disabled/observer/active 三种路径的影响
已知风险与下一步建议
```

不得用“应该通过”“看起来没问题”代替真实执行结果。

---

## 9. 提交纪律

1. 一个 diff/commit 只解决一个边界。
2. 不混入格式化、无关重命名或大面积机械改动。
3. 不修改生成文件、第三方依赖或无关板级代码。
4. 创建 Git commit 仅在用户明确要求时进行；否则提供可审查工作区 diff。
5. 每个阶段必须保持仓库可构建，或明确说明尚未完成且不得进入下一阶段。
6. 任何改变飞行行为、默认参数、ownership 或 handoff 的变更必须独立提交。

---

## 10. 迁移顺序

严格按架构文档中的阶段推进。默认顺序：

```text
C0  v1.2.0 baseline freeze / remotes / 参数迁移事实核验
C1  Safe-default audit（如需改变默认，独立实施）
C2  controller-neutral reference types
C3  compatibility publication 泛化
C4  state sampling / Geo integration bridge 从 Mode 移到 Copter
C5  selection/handoff helper 收敛，仍保留在 vehicle-level
C6  Guided/Loiter 迁移到新 adapter，行为必须等价
C7  AUTO-WP observer-only
C8  AUTO-WP active ownership（需单独批准）
```

不得跳过 observer-only，直接给新 Mode 电机 ownership。

---

## 11. 新 Mode 的最低验证顺序

任何新 Mode/SubMode 必须依次完成：

```text
reference extraction
→ reference validity/frame/freshness tests
→ observer-only
→ no-motor-write proof
→ Native/Geo diagnostic comparison
→ fault and mode-exit tests
→ active ownership
→ same-frame handoff tests
→ SITL lifecycle regression
→ HIL/physical-flight plan
```

AUTO、RTL 等必须按 SubMode/Control Primitive 声明支持范围，禁止只写“支持整个 Mode”。

---

## 12. 立即执行的第一个任务

首次启动时只做 C0，不修改代码：

1. 验证 fork URL、origin/upstream、`ardugeo-pid`、v1.2.0 release HEAD 与 ArduPilot base。
2. 列出当前分支、tag、submodule 和工作区相对冻结基线的状态。
3. 核对 v1.2.0 的参数 module ownership、storage identity、迁移脚本和测试入口。
4. 找到仓库已有构建、91 个 geometric unit tests、16 项 SITL/autotest 与参数迁移测试入口。
5. 核对架构文档中列出的关键类、函数和文件是否存在，记录名称差异。
6. 输出下一阶段 controller-neutral reference types 的精确实施计划和风险。
7. 证明没有修改文件，然后停止等待批准。
