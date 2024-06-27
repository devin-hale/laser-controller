# rf-laser-controller
Radio controlled laser pointer using the STM32F103C8T6

## Overview
The main learning goal of this project is the following
1. Learn about the STM32 cpu by doing a project from only the arm gnu toolchain and cmake
    (project template from https://github.com/kxygk/bluepill?tab=readme-ov-file)
2. Learn about RF communication protocols

### The scope of the project will encompass the following:
- The STM32 blue pill will drive two servos acting as pan/tilt for a mounted laser pointer
    - Can also turn on/off
    - Will have an upper limit for rotation values as not to damage wires
    - Instructions will be received via an RF receiver
- A Raspberry Pi W Zero 2 will be wired to an RF transmitter, and connect to my nintendo switch controller via bluetooth
    - Will receive inputs via the controller, and transmit to the laser pointer's RF receiver
    - Unnecessarily convoluted but seems cool to play with

### How to build
```
$ cmake -DCMAKE_TOOLCHAIN_FILE=toolchain.cmake -B build
$ cd build
$ make
```
