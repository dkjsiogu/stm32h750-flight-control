# 闭环飞控链路

## 固件边界

- `flight_control_firmware_core`: 真实飞控核心 target，只编译控制器、模型、数据结构、应用调度抽象和同步抽象。
- `flight_control_stm32_port`: STM32/FreeRTOS 适配层，通过板级 hook 读取真实传感器/指令并写入 PWM。
- `boards/stm32h743`: H743 板级可烧录骨架，真实外设驱动通过 `drivers/real` 和 `board_io.hpp` 接入；`H743_DRIVER_MODE=real` 会强制要求真实驱动，并默认自动获取 FreeRTOS Kernel。
- `../stm32h750-flight-sim`: 独立 PC 侧仿真仓库，提供飞机动力学、风场、载荷、电机/电调链路、传感器模型和闭环评估；不属于飞控固件仓库。

## 控制层

- `SpeedController`: 速度外环，输出目标姿态、目标加速度和 collective。内部将爬升率积分为目标高度，并用高度误差修正竖直速度目标，避免爬升结束或负载扰动后锁定到错误高度。
- `ModelAdapter`: 将姿态误差、角速度和上一帧动作组成 16 帧历史窗口，喂给生成的自适应 TCN/RMA 姿态策略。
- `TorqueController`: 将 collective 和 NN 输出力矩混控为四路 PWM，并带 PWM slew 限制，模拟实际链路不能瞬时响应的约束。

## 固件端口

- `src/main.cpp`: STM32 固件装配入口，注入 `FreertosTaskRunner`、`Stm32SensorSource`、`Stm32CommandSource`、`Stm32PwmOutput`、`Stm32CriticalSection` 和生成的自适应 TCN 策略。
- `Stm32SensorSource`: 通过 `flight_control_board_read_sensors` 读取真实原始传感器和可选外部观测，由 `StateEstimator` 生成控制状态，核心 `FlightTelemetry` 不包含仿真真值。
- `Stm32PwmOutput`: 通过 `flight_control_board_write_pwm` 写真实电调/电机 PWM。
- `boards/stm32h743/include/board_io.hpp`: 真实底层驱动契约，覆盖系统早期初始化、驱动初始化、failsafe 输出、IMU、外部观测、指令输入、PWM 输出、风估计、链路延迟和临界区。

## 独立仿真层

- `../stm32h750-flight-sim/flight_control_eval`: 生成 CSV 指标和 Markdown 报告。
- `../stm32h750-flight-sim/flight_control_policy_search`: 搜索姿态模型权重等价参数；当前搜索 152 维 TCN/RMA 参数，并加入鲁棒 INDI teacher 蒸馏项。
- `../stm32h750-flight-sim/flight_control_control_param_search`: 搜索速度外环、模型力矩缩放和 PWM 混控参数，当前闭环评估为 5/5 稳定、平均分 `90.894/100`。
- `../stm32h750-flight-sim/tools/export_linear_policy.py`: 将搜索得到的 TCN/RMA 参数导出成 `AdaptiveTcnPolicy` 的静态参数，并写回飞控仓库 `src/model/generated_policy.cpp`。
- `../stm32h750-flight-sim/flight_control_system_tests`: 把五个评估场景作为 CTest 回归门槛。
- `firmware_boundary_tests`: 在飞控仓库内检查固件核心 target、旧 host/eval 目录和核心头文件，确保仿真环境不回流到真机固件工程。
