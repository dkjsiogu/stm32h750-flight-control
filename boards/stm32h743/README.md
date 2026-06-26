# STM32H743 Board Skeleton

这个目录是面向 `STM32H743` 的可烧录工程骨架。上层飞控、启动文件、linker script、FreeRTOS 配置、newlib 桩和镜像生成规则已经在这里，真实上板只剩 `drivers/BSP` 需要补齐。

## 驱动模式

- `H743_DRIVER_MODE=stub`: 链接验证模式，使用 `drivers/stub`，固定 failsafe/disarm，不代表可飞。
- `H743_DRIVER_MODE=real`: 真实烧录模式，自动编译 `drivers/real` 下的真实驱动；没有真实驱动源文件时 CMake 直接失败。FreeRTOS Kernel 默认由 CMake 自动获取。

真实驱动入口在 `include/board_io.hpp` 中声明。补齐这些函数后，`src/board_hooks.cpp` 会把它们接到飞控核心的 `Stm32SensorSource`、`Stm32CommandSource`、`Stm32PwmOutput` 和临界区接口。

## 链接验证

```bash
cmake -S boards/stm32h743 -B build-h743 \
  -DCMAKE_TOOLCHAIN_FILE=boards/stm32h743/toolchain-arm-none-eabi.cmake \
  -DH743_FLIGHT_CONTROL_DIR="$(pwd)"
cmake --build build-h743
```

## 真实烧录构建

```bash
cmake -S boards/stm32h743 -B build-h743-real \
  -DCMAKE_TOOLCHAIN_FILE=boards/stm32h743/toolchain-arm-none-eabi.cmake \
  -DH743_FLIGHT_CONTROL_DIR="$(pwd)" \
  -DH743_DRIVER_MODE=real
cmake --build build-h743-real
```

默认 `H743_FETCH_FREERTOS=ON`，未指定本地 FreeRTOS 时会自动获取 FreeRTOS-Kernel，并加入 `tasks.c`、`queue.c`、`timers.c`、Cortex-M7 GCC port 和 `heap_4.c`。离线或定制 FreeRTOS 布局可以改用 `H743_FREERTOS_KERNEL_DIR`，或者传入 `H743_FREERTOS_SOURCES` 和 `H743_FREERTOS_INCLUDE_DIRS`。

真实驱动需要覆盖这些硬件能力：时钟/Cache/MPU/NVIC、GPIO、SPI/I2C/UART、DMA、TIM PWM 或 DShot、ADC 电池采样、IMU、接收机、可选气压计/GPS/光流、failsafe 输出。
