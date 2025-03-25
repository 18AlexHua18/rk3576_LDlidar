cmake_minimum_required(VERSION 3.10)

# 设置交叉编译 toolchain 文件
# 注意：这必须在 project() 命令之前
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/rk3576-toolchain.cmake CACHE PATH "toolchain file")

# Set the project name
project(rk3576_LDlidar)

# 设置构建类型，如果未指定则默认为Release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build (Debug or Release)" FORCE)
endif()

# 确保调试标志被正确应用
string(TOUPPER "${CMAKE_BUILD_TYPE}" uppercase_CMAKE_BUILD_TYPE)
if(uppercase_CMAKE_BUILD_TYPE MATCHES "DEBUG")
    # 直接设置完整的调试标志，而不是追加
    set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -ggdb3 -fno-omit-frame-pointer")
    # 额外设置通用标志以确保调试信息被包含
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
    message(STATUS "Debug flags: ${CMAKE_CXX_FLAGS_DEBUG}")
else()
    set(CMAKE_CXX_FLAGS_RELEASE "-O2")
endif()

# 显示当前构建类型和标志
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "CXX flags: ${CMAKE_CXX_FLAGS}")
message(STATUS "CXX ${CMAKE_BUILD_TYPE} flags: ${CMAKE_CXX_FLAGS_${uppercase_CMAKE_BUILD_TYPE}}")

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 确保不会剥离调试符号
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -rdynamic")

# 设置可执行文件输出路径
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# 添加所有源文件 - 通过明确指定源目录来避免包含CMakeCompilerId文件
file(GLOB SOURCES 
     "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

# 打印找到的源文件（用于调试）
message(STATUS "Found source files: ${SOURCES}")

# 确保有源文件
if(NOT SOURCES)
    # 如果没有找到任何源文件，则添加一个默认的main.cpp
    message(WARNING "No source files found! Creating a default main.cpp")
    file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp 
        "#include <iostream>\nint main() {\n  std::cout << \"Hello from LDlidar!\" << std::endl;\n  return 0;\n}")
    set(SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)
endif()

# 点云过滤选项（默认关闭）
option(ENABLE_POINT_FILTERING "Enable point cloud filtering" OFF)

# 如果启用了点云过滤，添加相应的编译定义
if(ENABLE_POINT_FILTERING)
    add_definitions(-DENABLE_POINT_FILTERING=1)
    message(STATUS "Point cloud filtering: ENABLED")
else()
    add_definitions(-DENABLE_POINT_FILTERING=0)
    message(STATUS "Point cloud filtering: DISABLED")
endif()

# Add the executable
add_executable(rk3576_LDlidar ${SOURCES})

# Include directories
include_directories(include)

# Link libraries
target_link_libraries(rk3576_LDlidar pthread)