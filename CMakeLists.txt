cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME dm_mc02_485_sbus_lkm)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    # Add user defined libraries
)

# stm32_framework：必须在 add_subdirectory(cmake/stm32cubemx) 之后接入（见 Middlewares/Third_Party/stm32-framework/README.md 第 1 节）
add_subdirectory(Middlewares/Third_Party/stm32-framework)
target_include_directories(stm32_framework PRIVATE user/board)  # 库自身也要看到 hardware_config.h
target_link_libraries(${CMAKE_PROJECT_NAME} stm32_framework)
target_compile_features(${CMAKE_PROJECT_NAME} PRIVATE cxx_std_20)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE user/board user/app user/module)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)  # BspUartT::printf 打印浮点需要（nano.specs）

# 业务代码：user/ 下的 .c / .cpp 自动收集（CONFIGURE_DEPENDS：增删业务文件后直接构建，不用改这里）
file(GLOB_RECURSE USER_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/user/*.c ${CMAKE_CURRENT_SOURCE_DIR}/user/*.cpp)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${USER_SOURCES})
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE user)
