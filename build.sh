#! /bin/bash

cmake -DCMAKE_TOOLCHAIN_FILE=./toolchain.cmake -B ./build && \
cd build && \
make && \
openocd && \
cd ..
