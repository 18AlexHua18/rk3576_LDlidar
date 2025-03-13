cmake_minimum_required(VERSION 3.10)

# 设置交叉编译 toolchain 文件
# 注意：这必须在 project() 命令之前
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/rk3576-toolchain.cmake CACHE PATH "toolchain file")

# Set the project name
project(rk3576_LDlidar)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

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

# Add the executable
add_executable(rk3576_LDlidar ${SOURCES})

# Include directories
include_directories(include)

# Link libraries
target_link_libraries(rk3576_LDlidar pthread)