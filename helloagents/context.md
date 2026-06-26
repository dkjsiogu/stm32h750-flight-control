# 项目上下文

## 当前目标

构建面向 `STM32H750VBT6` / `STM32H743` 的可烧录飞控固件主体，主链路为：

```text
SensorSource -> SpeedController -> ModelAdapter/StaticMlpPolicy -> TorqueController -> PwmOutput
```

核心模型只负责从姿态误差、角速度和历史动作窗口输出目标力矩；速度外环负责把目标速度、上升下降和偏航速率转换成目标姿态与 collective；力矩控制器负责把目标力矩混控为四路 PWM。

工程边界已经拆成三层：

- `flight_control_firmware_core`: 固件核心，只包含控制器、模型适配、数据结构、应用调度抽象和同步抽象，不编译 host 仿真或评估代码。
- `flight_control_stm32_port`: STM32/FreeRTOS 端口，通过板级 hook 对接真实传感器、速度指令、PWM 输出和临界区。
- `flight_control_host_support`: PC 侧验证工具，包含 host 真实环境替身和闭环评估器，不属于固件核心。

## 当前验证方式

host 侧 `HostFlightEnvironment` 只用于 PC 验证，作为真实环境替身包含电机一阶滞后、PWM 非线性推力曲线、电机个体差异、风场/阵风、阻力、电池压降、传感器延迟、陀螺仪 bias/drift/jitter、姿态/速度/位置估计偏差。飞控核心不能读取仿真真值，只能通过 `ISensorSource` 获取估计遥测，通过 `IPwmOutput` 写 PWM。`flight_control_eval` 会运行悬停风偏、前向巡航、阵风恢复、重物慢电机、上升转弯巡航五个场景，并输出 `output/flight_evaluation_report.md`。
