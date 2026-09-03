# Codex C0 Start Prompt — ArduGeo v1.2.0

Paste the following into Codex from the repository root:

```text
请先读取并遵守：
1. 仓库根目录 AGENTS.md；
2. docs/ARDUGEO_CODEX_RULES_V1_2.md；
3. docs/ARDUGEO_ARCHITECTURE_V1_2.md。

当前不要修改任何文件，也不要创建 branch、commit 或 PR。

只执行 C0 Baseline Freeze：
- 报告仓库 URL、origin/upstream、当前分支、HEAD、tag、工作区和 submodule 状态；
- 验证 fork 为 wanps/ArduGeo，基线分支为 ardugeo-pid；
- 验证 v1.2.0 release commit 为 697c4da33c0cf0d7b888e9eb6ec2645b6749e91c；
- 验证 ArduPilot base 为 48644320dc6c4a19cc20c44051e56908eee97f4e；
- 核对 v1.2.0 的参数模块 ownership、43 个迁移参数、57 个公开参数名称/默认值、
  GEO_OUT_EN 与 GEO_LREF_ storage identity、迁移与幂等重启测试；
- 找出并列出实际可运行的 Copter SITL build、geometric unit tests、
  geometric SITL regression、参数 metadata/default、EEPROM migration 测试命令；
- 核对架构文档列出的关键源码文件、类、方法和调用时序，记录不一致；
- 审计当前 Geo 默认输出策略，但不要修改；
- 输出下一阶段 controller-neutral reference types 的文件级、类型级和测试级计划；
- 最后用 git status 和 git diff 证明没有修改文件，然后停止等待批准。

禁止顺手修复、重构、格式化或声称未运行的测试通过。
```
