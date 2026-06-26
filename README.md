# STM32H750/H743 Flight Control

这是一个面向 `STM32H750VBT6` / `STM32H743` 的 C++ 飞控固件工程。当前边界是：

- `flight_control_firmware_core`: 可烧录飞控核心，不编译 PC 侧仿真、评估器、文件输出或 `std::thread` runner。
- `flight_control_stm32_port`: STM32/FreeRTOS 端口，负责接入真实传感器、速度指令、PWM 输出和临界区。
- `boards/stm32h743`: STM32H743 可烧录工程骨架，包含 ARM toolchain、启动文件、linker script、FreeRTOS 配置、newlib 桩和 drivers/BSP 接入点。

仿真、飞机动力学、风场、传感器延迟和训练评估已经剥离到独立仓库 `../stm32h750-flight-sim`。

## 固件控制链

`src/main.cpp` 是 STM32 固件装配入口，流程是：

1. `Stm32SensorSource` 通过板级 hook 读取真实 IMU、电池和可选外部观测，并由 `StateEstimator` 生成控制状态。
2. `Stm32CommandSource` 读取目标速度、爬升率和 yaw 角速度。
3. `SpeedController` 将目标速度转换为目标姿态和 collective。
4. `ModelAdapter` 将真实历史状态滑动窗口喂给静态 MLP 姿态模型，输出目标力矩。
5. `TorqueController` 将目标力矩混控为四路 PWM。
6. `Stm32PwmOutput` 将 PWM 写入真实电机/电调链路。

固件核心的遥测结构只包含真实飞控能获得的原始传感器、估计状态和链路诊断量；仿真真值只存在于独立仿真工程。

## H743 drivers/BSP 缺口

H743 工程现在有两个明确模式：

- `H743_DRIVER_MODE=stub`: 默认链接验证模式，只证明上层飞控、启动文件、linker script 和镜像生成能通过；驱动固定返回无效数据和 disarm。
- `H743_DRIVER_MODE=real`: 真实烧录模式，必须补齐 `boards/stm32h743/drivers/real/` 下的真实驱动源文件；FreeRTOS Kernel 默认由 CMake 自动获取，也可以指定本地路径。

真实驱动只需要实现 `boards/stm32h743/include/board_io.hpp` 里的底层函数：

```cpp
void board_system_preinit();
bool board_drivers_initialize();
void board_write_failsafe_outputs();
bool board_read_imu_sample(flight_control::SensorPacket* packet);
bool board_read_external_observation(flight_control::StateEstimatorObservation* observation);
bool board_read_guidance_command(flight_control::GuidanceCommand* command);
void board_write_motor_pwm(const flight_control::MotorPwmFrame* frame);
void board_read_estimated_wind(float* wind_x_m_s, float* wind_y_m_s);
float board_control_latency_ms();
void board_enter_critical();
void board_exit_critical();
```

补齐这些 drivers/BSP 后，飞控入口会先调用板级初始化，再启动采集、控制和执行器任务；生成的 `.hex/.bin` 就是实际烧录产物。

## 独立仿真

PC 侧仿真不在本仓库内。使用相邻仓库：

```bash
cd ../stm32h750-flight-sim
cmake -S . -B build
cmake --build build
./build/flight_control_eval
```

仿真仓库提供飞机环境和闭环评估，飞控仓库只作为被链接的控制器依赖。

## 姿态模型优化

当前姿态模型仍然是固件内的静态 MLP 权重，不依赖运行时训练框架。优化流程是：

```bash
cd ../stm32h750-flight-sim
cmake --build build --target flight_control_policy_search
./build/flight_control_policy_search
FLIGHT_CONTROL_DIR=../stm32h750-flight-control python3 tools/export_linear_policy.py
```

`flight_control_policy_search` 只搜索姿态模型权重等价参数，速度外环和力矩混控参数不参与调参。`export_linear_policy.py` 将最优历史策略精确导出回本仓库的 `StaticMlpPolicy` 36-128-128-3 权重。

当前模型使用分段 ReLU 历史特征表达大扰动下的非线性补偿；搜索得到的 70 维策略参数保存在仿真仓库 `tools/static_policy_params.txt`，导出的固件权重在 `src/model/generated_policy.cpp`。最新闭环验证平均分为 `84.6/100`，五个场景全部稳定。

## 构建与验证

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

固件入口链接验证：

```bash
cmake -S . -B build-firmware-entry \
  -DFLIGHT_CONTROL_BUILD_TESTS=OFF \
  -DFLIGHT_CONTROL_BUILD_FIRMWARE_ENTRY=ON
cmake --build build-firmware-entry
```

H743 链接验证模式：

```bash
cmake -S boards/stm32h743 -B build-h743 \
  -DCMAKE_TOOLCHAIN_FILE=boards/stm32h743/toolchain-arm-none-eabi.cmake
cmake --build build-h743
```

H743 真实烧录模式：

```bash
cmake -S boards/stm32h743 -B build-h743-real \
  -DCMAKE_TOOLCHAIN_FILE=boards/stm32h743/toolchain-arm-none-eabi.cmake \
  -DH743_DRIVER_MODE=real
cmake --build build-h743-real
```

如果真实驱动不放在 `drivers/real/`，用 `H743_DRIVER_SOURCES` 和 `H743_DRIVER_INCLUDE_DIRS` 传入。真实模式会自动启用 FreeRTOS，并自动加入常规 FreeRTOS Kernel 源文件、Cortex-M7 GCC port 和 `heap_4.c`。离线构建时设置 `H743_FREERTOS_KERNEL_DIR` 指向本地 FreeRTOS-Kernel，或关闭 `H743_FETCH_FREERTOS` 后手动传 `H743_FREERTOS_SOURCES`。

`firmware_boundary_tests` 会回归检查固件核心 target 不包含旧 PC/eval/demo 源文件，避免仿真代码再次混入飞控核心。
