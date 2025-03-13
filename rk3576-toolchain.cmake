# rk3576-toolchain.cmake
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

SET(CMAKE_C_COMPILER /home/alex/toolchain/aarch64-buildroot-linux-gnu_sdk-buildroot-rk3576/bin/aarch64-buildroot-linux-gnu-gcc)
SET(CMAKE_CXX_COMPILER /home/alex/toolchain/aarch64-buildroot-linux-gnu_sdk-buildroot-rk3576/bin/aarch64-buildroot-linux-gnu-g++)

# 设置交叉编译环境的根路径
set(CMAKE_FIND_ROOT_PATH /home/alex/toolchain/aarch64-buildroot-linux-gnu_sdk-buildroot-rk3576/aarch64-buildroot-linux-gnu/sysroot)

# 设置查找库和头文件的模式
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# 设置交叉编译环境中的库路径
set(PCAP_ROOT_DIR /home/alex/toolchain/aarch64-buildroot-linux-gnu_sdk-buildroot-rk3576/)
