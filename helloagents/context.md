# 项目上下文

## 当前目标

构建面向 `STM32H750VBT6` 的飞控系统骨架，主链路为：

```text
SensorSource -> SpeedController -> ModelAdapter/StaticMlpPolicy -> TorqueController -> PwmOutput
```

核心模型只负责从姿态误差、角速度和历史动作窗口输出目标力矩；速度外环负责把目标速度、上升下降和偏航速率转换成目标姿态与 collective；力矩控制器负责把目标力矩混控为四路 PWM。

## 当前验证方式

host 侧 `SimulatedQuadPlant` 提供带电机一阶滞后、风场、阻力、陀螺仪 bias/jitter 的闭环仿真。`flight_control_eval` 会运行悬停风偏、前向巡航、阵风恢复、重物慢电机、上升转弯巡航五个场景，并输出 `output/flight_evaluation_report.md`。
