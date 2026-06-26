# 闭环飞控链路

## 固件边界

- `flight_control_firmware_core`: 真实飞控核心 target，只编译控制器、模型、数据结构、应用调度抽象和同步抽象。
- `flight_control_stm32_port`: STM32/FreeRTOS 适配层，通过板级 hook 读取真实传感器/指令并写入 PWM。
- `flight_control_host_support`: PC 侧验证层，包含 host 环境和评估器，不允许进入固件核心。

## 控制层

- `SpeedController`: 速度外环，输出目标姿态、目标加速度和 collective。无上升/下降指令时会锁定当前高度，避免负载或电机延迟导致静态下沉后失去恢复驱动。
- `ModelAdapter`: 将姿态误差、角速度和上一帧动作组成 4 帧历史窗口，喂给生成的 MLP 姿态策略。
- `TorqueController`: 将 collective 和 NN 输出力矩混控为四路 PWM，并带 PWM slew 限制，模拟实际链路不能瞬时响应的约束。

## 固件端口

- `src/main.cpp`: STM32 固件装配入口，注入 `FreertosTaskRunner`、`Stm32SensorSource`、`Stm32CommandSource`、`Stm32PwmOutput`、`Stm32CriticalSection` 和生成的静态 MLP。
- `Stm32SensorSource`: 通过 `flight_control_board_read_telemetry` 读取真实板级遥测，核心 `FlightTelemetry` 不包含仿真真值。
- `Stm32PwmOutput`: 通过 `flight_control_board_write_pwm` 写真实电调/电机 PWM。

## 评估层

- `ClosedLoopEvaluator`: 确定性闭环评估入口。
- `flight_control_eval`: 生成 CSV 指标和 Markdown 报告。
- `flight_control_policy_search`: 只搜索姿态模型权重等价参数，不调速度外环和力矩混控参数。
- `tools/export_linear_policy.py`: 将搜索得到的历史姿态策略导出成 `StaticMlpPolicy` 的静态 MLP 权重。
- `flight_control_system_tests`: 把五个评估场景作为 CTest 回归门槛。
- `firmware_boundary_tests`: 检查固件核心 target 不包含 host/eval/demo 源文件，核心头文件不暴露 host runner 或仿真真值。
