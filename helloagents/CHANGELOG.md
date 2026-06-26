# Changelog

## [0.3.0] - 2026-06-26

### Added

- **[estimation]**: 新增固件内 `StateEstimator`，融合 IMU 和可选外部姿态/速度/位置观测，输出控制器使用的估计状态。
- **[boards]**: 新增 `boards/stm32h743` 可烧录工程骨架，包含 ARM toolchain、启动文件、linker script 和真实板级 hook 接入点。
- **[boards]**: 新增 H743 newlib 系统调用桩、堆边界符号和 hex/bin 镜像生成 target。
- **[tests]**: 扩展 `firmware_boundary_tests`，检查旧 host/eval 目录、旧评估 target 和训练工具不再存在于飞控仓库。

### Changed

- **[firmware]**: `Stm32SensorSource` 改为读取真实原始传感器和外部观测，由状态估计器生成 `VehicleState`，不再接受仿真真值。
- **[docs]**: 更新 README 和知识库，明确飞控仓库只保留真机固件，PC 侧飞机/环境/训练/评估位于相邻仿真仓库 `../stm32h750-flight-sim`。

### Removed

- **[simulation]**: 从飞控仓库移除 host 环境、闭环评估器、host demo、策略搜索工具和评估输出产物；这些内容迁移到独立仿真仓库。

## [0.2.0] - 2026-06-26

### Added

- **[eval]**: 新增 host 闭环评估 runner、`flight_control_eval` CLI、系统评估 CTest。
- **[eval]**: 增加悬停风偏、前向巡航、阵风恢复、重物慢电机、上升转弯巡航五个场景。
- **[docs]**: 新增 `helloagents/` 知识库和 `output/flight_evaluation_report.md` 评估产物。
- **[docs]**: 补齐飞控公共头文件的中文注释，覆盖类型、成员、函数参数和关键局部变量。
- **[host]**: 新增 host 真实环境替身，显式建模真值状态、传感器延迟/漂移、PWM 非线性、电机差异、阵风和电池压降。

### Changed

- **[control]**: 速度控制器加入无爬升指令时的内部高度保持目标。
- **[control]**: 优化水平/竖直速度外环参数，提高风扰和负载下的闭环恢复能力。
- **[control]**: 速度外环加入目标加速度斜率限制，力矩控制器加入推力曲线标定。
- **[model]**: 生成模型 torque scale 从 `0.10Nm` 调整为 `0.15Nm`，在当前评估集里提升巡航和上升转弯响应。
- **[model]**: 将姿态静态 MLP 从 36-64-64-3 扩展为 36-128-128-3，并加入可导出的分段 ReLU 历史非线性特征。
- **[model]**: 仅重新搜索 NN 姿态模型权重，速度外环和力矩混控参数保持不变；闭环评估提升到 `84.5/100`，5 个场景全部稳定。
- **[tests]**: 将系统评估回归门槛提高到单场景 `77.0`、平均分 `84.0`。
- **[firmware]**: 将固件核心拆为 `flight_control_firmware_core`，host 环境、闭环评估器、文件输出和 host 线程运行器全部移出固件核心。
- **[firmware]**: 新增 STM32/FreeRTOS 装配入口和板级 hook 端口，固件侧只接入真实传感器、指令、PWM 和临界区。
- **[data]**: 从核心 `FlightTelemetry` 移除 host 仿真真值字段，仿真真值只允许由 host 评估环境单独读取。

### Added

- **[tests]**: 新增 `firmware_boundary_tests`，回归检查固件核心 target 不包含 host/eval/demo 源文件，核心头文件不暴露 host runner 或仿真真值。
- **[model]**: 新增 `flight_control_policy_search` 和 `tools/export_linear_policy.py`，用于闭环搜索并导出静态 MLP 姿态模型权重。
- **[model]**: 仅更新 NN 姿态模型权重，闭环评估从 `75.6/100` 提升到 `84.2/100`，5 个场景全部稳定。
