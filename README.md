# STM32H750/H743 Flight Control

这是一个面向 `STM32H750VBT6` / `STM32H743` 的 C++ 飞控固件工程。当前边界是：

- `flight_control_firmware_core`: 可烧录飞控核心，不编译 host 仿真、评估器、文件输出或 `std::thread` runner。
- `flight_control_stm32_port`: STM32/FreeRTOS 端口，负责接入真实传感器、速度指令、PWM 输出和临界区。
- `flight_control_host_support`: PC 侧验证工具，包含 host 真实环境替身和闭环评估器，只用于训练/验证，不属于固件核心。

## 固件控制链

`src/main.cpp` 是 STM32 固件装配入口，流程是：

1. `Stm32SensorSource` 通过板级 hook 读取真实传感器和状态估计结果。
2. `Stm32CommandSource` 读取目标速度、爬升率和 yaw 角速度。
3. `SpeedController` 将目标速度转换为目标姿态和 collective。
4. `ModelAdapter` 将真实历史状态滑动窗口喂给静态 MLP 姿态模型，输出目标力矩。
5. `TorqueController` 将目标力矩混控为四路 PWM。
6. `Stm32PwmOutput` 将 PWM 写入真实电机/电调链路。

固件核心的遥测结构只包含真实飞控能获得的原始传感器、估计状态和链路诊断量；仿真真值只存在于 host 评估环境。

## STM32 板级 hook

移植到 STM32 工程时需要在板级代码中实现这些 C hook：

```cpp
void flight_control_board_read_telemetry(
    flight_control::SensorPacket* packet,
    flight_control::VehicleState* state,
    float* estimated_wind_x_m_s,
    float* estimated_wind_y_m_s,
    float* control_latency_ms);

void flight_control_board_read_command(flight_control::GuidanceCommand* command);
void flight_control_board_write_pwm(const flight_control::MotorPwmFrame* frame);
void flight_control_board_enter_critical();
void flight_control_board_exit_critical();
```

## Host 验证

PC 侧工具只用于验证同一套控制链路：

```text
SpeedController -> ModelAdapter -> StaticMlpPolicy -> TorqueController -> PWM -> host dynamics
```

`HostFlightEnvironment` 会模拟电机一阶滞后、PWM 非线性、电机个体差异、风场/阵风、阻力、电池压降、传感器延迟、陀螺仪 bias/drift/jitter、姿态/速度/位置估计偏差。飞控应用只能通过 `ISensorSource` 和 `IPwmOutput` 与它闭环。

## 姿态模型优化

当前姿态模型仍然是固件内的静态 MLP 权重，不依赖运行时训练框架。优化流程是：

```bash
cmake --build build --target flight_control_policy_search
./build/flight_control_policy_search
python3 tools/export_linear_policy.py
```

`flight_control_policy_search` 只搜索姿态模型权重等价参数，速度外环和力矩混控参数不参与调参。`export_linear_policy.py` 将最优历史策略精确导出为 `StaticMlpPolicy` 可执行的 36-64-64-3 权重。

## 构建与验证

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
./build/flight_control_eval
```

固件入口链接验证：

```bash
cmake -S . -B build-firmware-entry \
  -DFLIGHT_CONTROL_BUILD_HOST_TOOLS=OFF \
  -DFLIGHT_CONTROL_BUILD_TESTS=OFF \
  -DFLIGHT_CONTROL_BUILD_FIRMWARE_ENTRY=ON
cmake --build build-firmware-entry
```

接入真实 FreeRTOS 头文件时再显式打开：

```bash
-DFLIGHT_CONTROL_USE_FREERTOS=ON
-DFLIGHT_CONTROL_FREERTOS_INCLUDE_DIRS="path/to/FreeRTOS/include;path/to/portable"
```

验证输出：

- `output/flight_evaluation_metrics.csv`
- `output/flight_evaluation_report.md`

`firmware_boundary_tests` 会回归检查固件核心 target 不包含 host/eval/demo 源文件，避免仿真代码再次混入飞控核心。
