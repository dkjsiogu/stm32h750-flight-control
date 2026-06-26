# 闭环飞控链路

## 控制层

- `SpeedController`: 速度外环，输出目标姿态、目标加速度和 collective。无上升/下降指令时会锁定当前高度，避免负载或电机延迟导致静态下沉后失去恢复驱动。
- `ModelAdapter`: 将姿态误差、角速度和上一帧动作组成 4 帧历史窗口，喂给生成的 MLP 姿态策略。
- `TorqueController`: 将 collective 和 NN 输出力矩混控为四路 PWM，并带 PWM slew 限制，模拟实际链路不能瞬时响应的约束。

## 评估层

- `ClosedLoopEvaluator`: 确定性闭环评估入口。
- `flight_control_eval`: 生成 CSV 指标和 Markdown 报告。
- `flight_control_system_tests`: 把五个评估场景作为 CTest 回归门槛。
