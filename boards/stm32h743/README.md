# STM32H743 Board Skeleton

这个目录是面向 `STM32H743` 的最小可烧录工程骨架。它只负责把真实板级驱动接到飞控核心：

- `flight_control_board_read_sensors`: 读取 IMU、电池和可选外部姿态/位置/速度观测。
- `flight_control_board_read_command`: 读取速度、爬升率和 yaw 角速度指令。
- `flight_control_board_write_pwm`: 写四路电机 PWM。
- `flight_control_board_enter_critical` / `flight_control_board_exit_critical`: 进入和退出临界区。

低层驱动入口在 `include/board_io.hpp` 中声明。默认实现是 failsafe，占位函数会返回无效传感器和 disarm 指令；接真实硬件时实现这些 `board_*` 函数即可。

构建示例：

```bash
cmake -S boards/stm32h743 -B build-h743 \
  -DCMAKE_TOOLCHAIN_FILE=boards/stm32h743/toolchain-arm-none-eabi.cmake \
  -DH743_FLIGHT_CONTROL_DIR="$(pwd)"
cmake --build build-h743
```

接入真实 FreeRTOS 时显式打开：

```bash
cmake -S boards/stm32h743 -B build-h743 \
  -DCMAKE_TOOLCHAIN_FILE=boards/stm32h743/toolchain-arm-none-eabi.cmake \
  -DH743_FLIGHT_CONTROL_DIR="$(pwd)" \
  -DH743_USE_FREERTOS=ON \
  -DH743_FREERTOS_INCLUDE_DIRS="path/to/FreeRTOS/include;path/to/portable"
cmake --build build-h743
```

实际烧录前还必须补齐真实时钟、IMU、气压计/GPS/光流、接收机、电调 PWM 定时器和 FreeRTOS 移植文件。
