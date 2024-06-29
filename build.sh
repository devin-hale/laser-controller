#! /bin/bash

cmake -DCMAKE_TOOLCHAIN_FILE=./toolchain.cmake ./build && \
cd build && \
make && \
openocd && \
cd ..
