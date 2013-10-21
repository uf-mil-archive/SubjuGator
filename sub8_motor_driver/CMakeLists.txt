cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_find_ros_package(libopencm3)
rosbuild_find_ros_package(arm_bootloader)

add_custom_command(
    WORKING_DIRECTORY ../firmware
    COMMAND $(MAKE) BUILD_DIR=../build/firmware_out TOOLCHAIN_DIR=${libopencm3_PACKAGE_PATH}/build/libopencm3 ARM_BOOTLOADER_DIR=${arm_bootloader_PACKAGE_PATH}
    OUTPUT firmware_out/firmware.bin.dummy
)
add_custom_command(
    DEPENDS firmware_out/firmware.bin.dummy
    WORKING_DIRECTORY firmware_out
    COMMAND xxd -i firmware.bin > ../firmware_bin.cpp.tmp
    COMMAND cmp -s ../firmware_bin.cpp.tmp ../firmware_bin.cpp || cp ../firmware_bin.cpp.tmp ../firmware_bin.cpp
    OUTPUT firmware_bin.cpp
)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
rosbuild_add_executable(sub8_motor_driver src/sub8_motor_driver.cpp build/firmware_bin.cpp)
rosbuild_link_boost(${PROJECT_NAME} system)
rosbuild_add_compile_flags(sub8_motor_driver -std=c++11)
#target_link_libraries(example ${PROJECT_NAME})
