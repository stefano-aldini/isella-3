cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
rosbuild_gensrv()
rosbuild_genmsg()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)

#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(my_controller_exe src/isella3_controller2.cpp
src/ISELLA3_ArmModule.cpp
src/ISELLA3_Process_Monitor.cpp
src/ISELLA3_Position_Control.cpp)

rosbuild_add_executable(manual src/manual_mode.cpp
src/ISELLA3_ArmModule.cpp
src/ISELLA3_Process_Monitor.cpp
src/ISELLA3_Position_Control.cpp)

rosbuild_add_library(my_controller_lib src/ISELLA3_ArmModule.cpp
src/ISELLA3_Process_Monitor.cpp
src/ISELLA3_Position_Control.cpp
src/isella3_controller2.cpp)

rosbuild_add_library(manual_lib src/manual_mode.cpp
src/ISELLA3_ArmModule.cpp
src/ISELLA3_Process_Monitor.cpp
src/ISELLA3_Position_Control.cpp)

link_directories(/usr/local/lib)

target_link_libraries(my_controller_exe EPOS)

target_link_libraries(manual EPOS)
