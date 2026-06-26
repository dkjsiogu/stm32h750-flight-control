# Changelog

## [0.2.0] - 2026-06-26

### Added

- **[eval]**: 新增 host 闭环评估 runner、`flight_control_eval` CLI、系统评估 CTest。
- **[eval]**: 增加悬停风偏、前向巡航、阵风恢复、重物慢电机、上升转弯巡航五个场景。
- **[docs]**: 新增 `helloagents/` 知识库和 `output/flight_evaluation_report.md` 评估产物。
- **[docs]**: 补齐飞控公共头文件的中文注释，覆盖类型、成员、函数参数和关键局部变量。

### Changed

- **[control]**: 速度控制器加入无爬升指令时的内部高度保持目标。
- **[control]**: 优化水平/竖直速度外环参数，提高风扰和负载下的闭环恢复能力。
- **[model]**: 生成模型 torque scale 从 `0.10Nm` 调整为 `0.15Nm`，在当前评估集里提升巡航和上升转弯响应。
