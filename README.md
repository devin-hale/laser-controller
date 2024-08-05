# laser-controller

Mounted laser pointer with pot controlled yaw/pitch. Laser can be toggled on/off via a push button.

## Materials

| Component     | Model Number  | Quantity |
| ------------- | ------------- | -------- |
| MCU           | STM32F103C8T6 | 1        |
| Servo         | SG-90         | 2        |
| Potentiometer | WH184         | 2        |
| I2C LCD       | 1602          | 1        |
| Laser Pointer | N/A           | 1        |
| Push Button   | N/A           | 1        |
| AC Port       | N/A           | 1        |


### How To Build

```
$ cmake -DCMAKE_TOOLCHAIN_FILE=toolchain.cmake -B build
$ cd build
$ make
```
or just use the included `build.sh`
