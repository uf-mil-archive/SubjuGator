# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(sub_launch)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS magnetic_dynamic_compensation adis16400_imu thruster_mapper imagenex_852 leaksensor_driver rise_6dof odometry_utils indirect_kalman_6dof tf magnetic_hardsoft_compensation uf_common thruster_driver rdi_explorer_dvl image_proc c3_trajectory_generator depth_driver object_finder merge_driver)
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

#set the default path for built executables to the "bin" directory

# CATKIN_MIGRATION: removed during catkin migration
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory

# CATKIN_MIGRATION: removed during catkin migration
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
## Generate added messages and services with any dependencies listed here
#generate_messages(
#    DEPENDENCIES 
#)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS  # TODO
    CATKIN_DEPENDS magnetic_dynamic_compensation adis16400_imu thruster_mapper imagenex_852 leaksensor_driver rise_6dof odometry_utils indirect_kalman_6dof tf magnetic_hardsoft_compensation uf_common thruster_driver rdi_explorer_dvl image_proc c3_trajectory_generator depth_driver object_finder merge_driver
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

include_directories( ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})


#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

install(PROGRAMS scripts/record scripts/newshooter_stage2 scripts/odom_to_euler scripts/legacy_debug_images scripts/plot_angle scripts/manipulation scripts/bump_buoy scripts/barrel_roll scripts/show_color scripts/debug_images scripts/grid_pattern scripts/manipulation_wheel scripts/bashrc scripts/bins scripts/bag_to_csv scripts/get_power scripts/dvl_conn scripts/paths scripts/path scripts/depth_conn scripts/imu_conn scripts/get_pose scripts/camera_views scripts/widget scripts/newshooter scripts/send_waypoint scripts/plot_coordinate scripts/shooter DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
