# 项目上下文

## 当前目标

构建面向 `STM32H750VBT6` / `STM32H743` 的可烧录飞控固件主体，主链路为：

```text
SensorSource -> SpeedController -> ModelAdapter/StaticMlpPolicy -> TorqueController -> PwmOutput
```

核心模型只负责从姿态误差、角速度和历史动作窗口输出目标力矩；速度外环负责把目标速度、上升下降和偏航速率转换成目标姿态与 collective；力矩控制器负责把目标力矩混控为四路 PWM。

工程边界已经拆成三层：

- `flight_control_firmware_core`: 固件核心，只包含控制器、模型适配、数据结构、应用调度抽象和同步抽象，不编译 PC 侧仿真或评估代码。
- `flight_control_stm32_port`: STM32/FreeRTOS 端口，通过板级 hook 对接真实传感器、速度指令、PWM 输出和临界区。
- `boards/stm32h743`: H743 可烧录骨架，包含 toolchain、启动文件、linker script、FreeRTOS 配置、newlib 桩、stub/real 驱动模式和 drivers/BSP 接入点。

## 当前验证方式

PC 侧仿真已经剥离到相邻仓库 `../stm32h750-flight-sim`。仿真工程提供飞机、风场、传感器延迟、陀螺仪 bias/drift/jitter、电机滞后、PWM 非线性推力曲线、电池压降、载荷和闭环评估场景，角色类似 Gazebo/SITL 环境。飞控仓库只作为被链接的控制器依赖，不保存仿真真值、评估器、PC 线程 runner 或文件输出 target。

飞控核心不能读取仿真真值，只能通过 `ISensorSource` 获取估计遥测，通过 `IPwmOutput` 写 PWM。`firmware_boundary_tests` 会检查旧 host/eval 目录、旧评估 target 和核心头文件暴露，防止仿真代码再次混入固件仓库。

H743 真实烧录模式为 `H743_DRIVER_MODE=real`。该模式必须提供 `drivers/real` 或 `H743_DRIVER_SOURCES`；FreeRTOS Kernel 默认由 CMake 自动获取，也可通过 `H743_FREERTOS_KERNEL_DIR` 指向本地副本。默认 `stub` 模式只用于 ARM 交叉编译、linker script 和 hex/bin 生成验证，固定保持 failsafe/disarm。

## 当前模型状态

姿态模型仍然是 `StaticMlpPolicy` 静态 MLP 权重，当前结构为 36-128-128-3。最近一次优化只调整模型权重等价参数，没有改速度外环和力矩混控参数；独立仿真仓库的 `flight_control_eval` 当前 5/5 场景稳定，平均分 `84.6/100`。
