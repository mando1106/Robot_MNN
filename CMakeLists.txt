cmake_minimum_required(VERSION VERSION 3.10)
project(main)


# === 控制变量：默认不开启交叉编译（本机编译） ===
option(USE_ARM "Build for ARM architecture (cross-compile)" OFF)
# === 设置交叉编译器 ===
if(USE_ARM)
    message(STATUS "Cross-compiling for ARM")
    # 默认工具链路径（可通过 -DTOOLCHAIN_PATH=xxx 设置）
    set(TOOLCHAIN_PATH /home/mando/Phd/Ec_mater/mando_torch/test_arm_static/gcc-linaro-7.5.0-2019.12-x86_64_arm-linux-gnueabihf)
    set(CMAKE_C_COMPILER ${TOOLCHAIN_PATH}/bin/arm-linux-gnueabihf-gcc)
    set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PATH}/bin/arm-linux-gnueabihf-g++)
    set(CMAKE_FIND_ROOT_PATH ${TOOLCHAIN_PATH}/arm-linux-gnueabihf)   
    set(CMAKE_SYSROOT ${TOOLCHAIN_PATH}/arm-linux-gnueabihf/libc)

# 设置静态链接和相关选项
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static-libgcc -static-libstdc++")

    set(MNN_INCLUDE_DIR /home/mando/Phd/Ec_mater/mando_torch/MNN/build-arm-gcc7/install-arm/include)
    set(MNN_LIB /home/mando/Phd/Ec_mater/mando_torch/MNN/build-arm-gcc7/install-arm/lib/libMNN.a)
else()
    message(STATUS "Building for native (host) architecture")
    set(MNN_INCLUDE_DIR /home/mando/Phd/Ec_mater/mando_torch/MNN/build/install/include)
    set(MNN_LIB         /home/mando/Phd/Ec_mater/mando_torch/MNN/build/install/lib/libMNN.so)
endif()

set(Python3_USE_DEBUG_LIBS FALSE)  # 禁用 debug 库链接
# 找Python解释器、开发包、NumPy（确保你的Anaconda环境激活）
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED )

find_package(Eigen3 CONFIG REQUIRED)
if (Eigen3_FOUND)
    message(STATUS "Eigen3 found: ${Eigen3_DIR}")
endif()

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# # ➕ 添加优化编译选项
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3  ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3  ")


file(GLOB SOURCES src/*.cpp )

# 找Python解释器、开发包、NumPy（确保你的Anaconda环境激活）
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
# find_package(Threads REQUIRED)

add_executable(main ${SOURCES})

# 添加Python头文件和NumPy头文件路径
target_include_directories(main PRIVATE
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}
)

target_include_directories(main PRIVATE ${MNN_INCLUDE_DIR})
target_link_libraries(main PRIVATE ${MNN_LIB} )

target_link_libraries(main PRIVATE ${Python3_LIBRARIES} )
# target_link_libraries(main PRIVATE Eigen3::Eigen Threads::Threads)
target_link_libraries(main PRIVATE Eigen3::Eigen pthread)

set_property(TARGET main PROPERTY CXX_STANDARD 17)
if(USE_ARM)

else()
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
endif()