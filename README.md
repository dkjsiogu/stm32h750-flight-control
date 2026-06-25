# STM32H750 Flight Control

这是一个面向 `STM32H750VBT6` 的 C++ 飞控工程骨架，分成三层：

- 顶层：任务编排、数据采集、状态流转和运行时管理
- 中间层：速度控制器、神经网络姿态模型适配器、力矩到 PWM 混控
- 底层：传感器、PWM、FreeRTOS 端口和主机模拟端口

## 结构

```text
include/flight_control/   公共类型、数学工具、控制器、模型接口、任务接口
src/                      可编译的主机模拟、控制器和应用实现
tests/                    主机侧验证
```

## 控制链

`main.cpp` 展示完整飞控链路：

1. 数据采集接口获取姿态、角速度、速度、电压等数据
2. 速度控制器把目标速度转换成目标姿态和 collective
3. 模型适配器把历史窗口喂给姿态模型，输出目标力矩
4. 力矩控制器把目标力矩混控成四路 PWM
5. 运行时通过线程或 FreeRTOS 任务持续执行

## 构建

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build
```

## 闭环评估

`flight_control_eval` 在 host 模拟器里复用正式控制链路：

```text
SpeedController -> ModelAdapter -> StaticMlpPolicy -> TorqueController -> PWM -> motor lag -> rigid-body plant
```

运行：

```bash
./build/flight_control_eval
```

评估场景覆盖：

- `hover_wind_bias`: 悬停风偏和陀螺仪 bias/jitter
- `forward_cruise`: 水平巡航速度阶跃
- `gust_recovery`: 两次方向相反的阵风阶跃
- `payload_motor_lag`: 额外负载和更慢电机响应
- `climb_turn_cruise`: 上升、偏航和巡航锁高

输出文件：

- `output/flight_evaluation_metrics.csv`
- `output/flight_evaluation_report.md`

## 备注

- Host 侧带一个简单的四旋翼模拟器，用来验证任务调度和控制链。
- `include/flight_control/platform/freertos/` 预留了 STM32 + FreeRTOS 端口。
- 神经网络策略通过 `IAttitudePolicy` 接口注入，输入维度固定为 4 帧 x 9 维。
- `StaticMlpPolicy` 提供固定 36-64-64-3 MLP 推理后端，当前已接入从 `checkpoints/latest.pt` 导出的 C++ 静态权重。
- 系统评估默认只使用生成的 NN 权重，不使用 PD fallback。
