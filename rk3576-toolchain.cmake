# rk3576-toolchain.cmake

# 设置目标系统
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# 设置交叉编译工具链路径 - 使用绝对路径
set(TOOLCHAIN_PATH /home/alex/toolchain/aarch64-buildroot-linux-gnu_sdk-buildroot-rk3576)

# 强制设置C/C++编译器 (使用FORCE避免被其他值覆盖)
set(CMAKE_C_COMPILER ${TOOLCHAIN_PATH}/bin/aarch64-buildroot-linux-gnu-gcc CACHE FILEPATH "C compiler" FORCE)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PATH}/bin/aarch64-buildroot-linux-gnu-g++ CACHE FILEPATH "C++ compiler" FORCE)

# 设置搜索路径
set(CMAKE_FIND_ROOT_PATH ${TOOLCHAIN_PATH})

# 调整查找行为，避免使用主机系统库
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# 禁用编译器检查（可选，如果上述修改仍然有问题，可以取消下面的注释）
# set(CMAKE_C_COMPILER_WORKS 1)
# set(CMAKE_CXX_COMPILER_WORKS 1)
