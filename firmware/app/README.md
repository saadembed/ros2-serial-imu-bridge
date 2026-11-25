# Firmware app (STM32 Nucleo-L432KC)

`main.c` shows how to wire the portable code in `../protocol` and `../drivers` into an STM32 HAL project: MPU-6050 on I2C1, frames out on USART2 (ST-LINK virtual COM port) at 115200 baud, about 100 Hz.

**Status: `main.c` has not been compiled for the target and has never run on hardware.** It needs the STM32L4 HAL and CubeMX-generated init code, which are not in this repo. Only `protocol/` and `drivers/` are compiled and tested, on a PC (see `../tests`).

## Setup

1. In STM32CubeIDE, create a project for NUCLEO-L432KC.
2. In CubeMX, enable I2C1 (for example PB6 SCL / PB7 SDA; check the pins and solder bridges in the Nucleo-32 user manual) and USART2 asynchronous at 115200 8N1. Generate code.
3. Add `protocol/frame.c` and `drivers/mpu6050.c` to the build, and add `protocol/` and `drivers/` to the include paths.
4. Copy the `USER CODE` parts of `main.c` into the generated `main.c`. `SystemClock_Config`, `MX_*_Init`, `Error_Handler`, `hi2c1` and `huart2` come from the generated code.
5. Wire the MPU-6050 (VCC 3.3 V, GND, SCL, SDA, AD0 low for address 0x68). Many breakout boards already have pull-ups.

## Host side

```bash
ros2 run serial_imu_bridge bridge_node --ros-args -p port:=/dev/ttyACM0 -p baud:=115200
```

## Next steps

- `HAL_UART_Transmit_DMA` instead of blocking `HAL_UART_Transmit`.
- Use the MPU-6050 data-ready interrupt (INT pin) instead of polling on `HAL_GetTick`.
