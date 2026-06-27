# Changelog

## [0.9.0] - 2026-06-28

### Added

- **[training]**: 仿真仓库新增静态姿态反解 PyTorch 反向传播训练器，用随机化姿态/角速度/执行器滞后样本直接计算 teacher 力矩并训练 363 维候选权重。
- **[export]**: 导出器新增 `POLICY_PARAM_PATH` 覆盖入口，可临时导出候选权重到飞控生成代码进行闭环验证。

### Changed

- **[model]**: 本轮反向传播候选没有超过当前正式权重的独立闭环验证分，因此飞控正式模型权重保持不变，避免用静态监督结果破坏 payload/motor-lag 场景。

## [0.8.0] - 2026-06-28

### Added

- **[control]**: `SpeedController` 新增水平/竖直速度微分增益 `kd_xy`/`kd_z`，使用基于测量的低通滤波微分（derivative-on-measurement），对突发风扰提供即时加速度修正，在指令跳变时不会产生微分尖峰。
- **[training]**: `flight_control_control_param_search` 参数扩展到 22 维，新增 `kd_xy`/`kd_z` 搜索；`flight_control_policy_search` 扩展训练/开发评估到 25 个随机化场景（5 个 seed），增强 INDI teacher 的误差积分、二阶微分项和阶跃风扰样本。

### Changed

- **[evaluation]**: 固定五场景保持 `5/5` 稳定，平均分提升到 `90.667/100`；gust_recovery 从 `81.922` 提升到 `83.316`；随机验证 seed `20260629` 保持 `5/5` 稳定，平均分提升到 `88.066/100`，gust_recovery 验证分从 `80.807` 提升到 `82.975`。
- **[firmware]**: 速度外环 PID 增益由 `control_param_search` 全量重调（`kp_xy` 4.11、`max_accel_xy` 10.5、`kd_z` 1.01 等）；H743 stub 构建通过，`.text` 为 `80,744 bytes`。

## [0.7.0] - 2026-06-28

### Added

- **[model]**: `AdaptiveTcnPolicy` 参数扩展到 363 维，新增四层 dilated residual TCN 分支，膨胀率为 `1/2/4/8`，用于在 64ms 历史窗口内增强多时间尺度扰动响应。
- **[training]**: `flight_control_policy_search` 扩展训练/开发评估到 15 个随机化场景，并增强 INDI teacher 的目标加速度与角加速度项；最终采用 160 轮搜索中独立验证不退化的候选。

### Changed

- **[evaluation]**: 固定五场景保持 `5/5` 稳定，平均分为 `90.209/100`；随机验证 seed `20260629` 保持 `5/5` 稳定，平均分提升到 `87.679/100`，最弱场景分 `80.807`。
- **[firmware]**: H743 stub 构建通过，当前 `flight_control_h743.elf` 的 `.text` 为 `79,264 bytes`。

## [0.6.0] - 2026-06-28

### Added

- **[model]**: `ModelAdapter` 单帧输入扩展为 16 维，加入姿态误差 log map、目标加速度、collective 和角加速度历史；`AdaptiveTcnPolicy` 参数扩展到 288 维，并加入学习式 RMA summary encoder。
- **[safety]**: 姿态模型输出后新增 CBF 风格安全滤波，对接近倾角和角速度边界的同向力矩做软衰减。
- **[estimation]**: `StateEstimatorObservation` 新增 yaw 观测入口；状态估计器新增误差状态协方差对角线、NIS 风格创新降权和机体系加速度慢偏学习。
- **[tests]**: 新增 yaw 观测融合和异常位置观测降权回归测试。

### Changed

- **[training]**: 仿真训练改为 288 维策略搜索，训练 seed `20260628/20260630`、开发 seed `20260631`、验证 seed `20260629` 分离，降低固定场景过拟合。
- **[control]**: 速度外环、模型力矩缩放和混控参数使用训练/开发随机化场景重新联合优化。
- **[evaluation]**: 固定五场景保持 `5/5` 稳定，平均分提升到 `90.088/100`；随机验证 seed `20260629` 保持 `5/5` 稳定，平均分提升到 `87.462/100`，最弱场景分 `80.768`。
- **[firmware]**: H743 stub 构建通过，当前 `flight_control_h743.elf` 的 `.text` 为 `78,496 bytes`。

## [0.5.0] - 2026-06-27

### Added

- **[model]**: 新增 `AdaptiveTcnPolicy`，用 16 帧因果时间卷积核和 RMA latent 替代导出静态 MLP 作为主姿态策略。
- **[training]**: 策略搜索加入鲁棒 INDI teacher 蒸馏项，用局部稳定控制律约束 TCN/RMA 搜索方向。
- **[safety]**: `TorqueController` 新增立即 failsafe PWM 输出，disarm 或状态不健康时不再沿用上一帧控制 PWM。
- **[tests]**: 新增 failsafe PWM 单元测试，覆盖最小 PWM 立即输出行为。
- **[tests]**: 新增速度外环爬升目标回归测试，覆盖爬升结束后继续跟踪内部高度目标。
- **[training]**: 仿真仓库新增 `flight_control_control_param_search`，用于搜索速度外环、模型力矩缩放和 PWM 混控参数。

### Changed

- **[model]**: `ModelAdapter` 新增姿态误差、角速度和上一动作的分组归一化，并将目标姿态历史重置阈值收回到随机化训练得到的有效范围。
- **[control]**: `TorqueController` 新增电池电压补偿、非对称 PWM slew 和饱和时的力矩比例分配，减少低电压和电机限幅下的力矩失真。
- **[control]**: 速度外环参数按随机化训练场景重新精修，当前独立仿真固定报告 `5/5` 稳定、平均 `84.819/100`，随机验证 seed `20260629` 为 `5/5` 稳定、平均 `82.402/100`。
- **[firmware]**: `FlightApplication` 在传感器不健康或指令 disarm 时重置速度外环、力矩控制器和模型历史，并发布 failsafe 控制解。
- **[control]**: 速度外环将爬升率积分为内部高度目标，停止爬升后不再用当前高度覆盖目标高度。
- **[model]**: 历史输入窗口从 4 帧扩展到 8 帧，静态 MLP 输入从 36 维扩展到 72 维，姿态策略搜索参数从 70 维扩展到 86 维。
- **[control]**: 联合优化 8 帧 NN 姿态策略、速度外环、模型力矩缩放和 PWM 混控参数；闭环评估提升到 `90.499/100`，5 个场景全部稳定。
- **[tests]**: 仿真仓库系统评估门槛提高到单场景 `83.0`、平均分 `90.0`。
- **[model]**: 历史输入窗口从 8 帧扩展到 16 帧，静态 MLP 扩展到 144-256-256-3，姿态策略搜索参数扩展到 152 维，并加入一阶变化 ReLU 门控。
- **[control]**: 联合精修宽历史 NN 姿态策略、速度外环、模型力矩缩放和 PWM 混控参数；闭环评估提升到 `90.873/100`，最低场景分提升到 `85.223`。
- **[tests]**: 仿真仓库系统评估门槛提高到单场景 `85.0`、平均分 `90.8`。
- **[firmware]**: 移除飞控仓库中过期的训练参数文件，并将其加入固件边界测试禁止清单；训练参数只保留在独立仿真仓库。
- **[estimation]**: `StateEstimator` 升级为误差状态式融合，外部姿态观测修正姿态和陀螺零偏，速度观测学习世界系加速度慢偏。
- **[model]**: 生成策略入口从 `StaticMlpPolicy` 切换为 `AdaptiveTcnPolicy`，保留 152 维历史策略参数并启用 RMA latent 耦合。
- **[control]**: 联合精修 Adaptive TCN/RMA 姿态策略、速度外环和混控参数；闭环评估提升到 `90.894/100`，5 个场景全部稳定。
- **[tests]**: 仿真仓库系统评估门槛提高到单场景 `85.1`、平均分 `90.88`。

## [0.4.0] - 2026-06-27

### Added

- **[boards]**: H743 工程新增 `H743_DRIVER_MODE=stub|real`，真实模式必须提供 `drivers/real` 或 `H743_DRIVER_SOURCES`，并可自动获取 FreeRTOS-Kernel。
- **[boards]**: 新增 `FreeRTOSConfig.h`、Cortex-M7 SVC/PendSV/SysTick 映射和 `SystemCoreClock`，减少真实上板前的工程胶水缺口。
- **[drivers]**: 新增 `drivers/stub` 链接验证驱动和 `drivers/real` 真实驱动目录契约。
- **[tests]**: `firmware_boundary_tests` 新增 H743 真实驱动模式约束检查。

### Changed

- **[firmware]**: 固件入口启动任务前调用 `flight_control_board_initialize`，H743 hook 在驱动初始化失败时保持 failsafe/disarm。
- **[docs]**: README 和知识库改为明确说明：上层飞控已完整，真实烧录只剩 drivers/BSP 和 FreeRTOS Kernel 需要补齐。

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
