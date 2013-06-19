cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
find_package (Eigen REQUIRED)
include_directories (${Eigen_INCLUDE_DIRS})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#rosbuild_genmsg()
#rosbuild_gensrv()
#include_directories(include)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
rosbuild_add_library(${PROJECT_NAME} 	src/mpc/lbmpc.cpp
										src/mpc/stdmpc.cpp
										src/example_models/tanks_system.cpp
										src/example_models/tanks_system_simulator.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)

rosbuild_add_executable(stdmpc_node test/stdmpc_node.cpp)
#												src/QPOASES.cpp
#												src/Tanksystem.cpp)
target_link_libraries(stdmpc_node ${PROJECT_NAME})





