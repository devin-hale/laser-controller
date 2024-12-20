# laser-controller

Mounted laser pointer with pot controlled yaw/pitch. Laser can be toggled on/off via a push button.

## Materials

| Component     | Model Number  | Quantity |
| ------------- | ------------- | -------- |
| MCU           | STM32F103C8T6 | 1        |
| Servo         | SG-90         | 2        |
| I2C LCD       | LCD1602       | 1        |
| Potentiometer | n/a           | 2        |
| Laser Pointer | n/a           | 1        |
| Push Button   | n/a           | 1        |
| AC Port       | n/a           | 1        |

## Resources

- [STM32 Blue Pill Details](https://stm32-base.org/boards/STM32F103C8T6-Blue-Pill.html)
- [STM32F1 Reference Manual](./resources/stm32f1_reference_manual.pdf)
- [I2C LCD Datasheet](./resources/I2C_1602_LCD.pdf)
- [Servo Motor Datasheet](./resources/sg90_datasheet.pdf)

### How To Build

```
$ cmake -DCMAKE_TOOLCHAIN_FILE=toolchain.cmake -B build
$ cd build
$ make
```
or just use the included `build.sh`
