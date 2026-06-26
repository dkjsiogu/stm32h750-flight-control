# H743 Real Drivers

把真实板级驱动源文件放在这个目录，或通过 `H743_DRIVER_SOURCES` 传给 CMake。

`H743_DRIVER_MODE=real` 时，本目录下的 `.c/.cpp/.s/.S` 会被自动加入固件目标；如果没有任何真实驱动源文件，CMake 会直接失败，避免生成只有占位驱动的烧录镜像。

必须实现的接口见 `boards/stm32h743/include/board_io.hpp`：

- `board_system_preinit`
- `board_drivers_initialize`
- `board_write_failsafe_outputs`
- `board_read_imu_sample`
- `board_read_external_observation`
- `board_read_guidance_command`
- `board_write_motor_pwm`
- `board_read_estimated_wind`
- `board_control_latency_ms`
- `board_enter_critical`
- `board_exit_critical`
