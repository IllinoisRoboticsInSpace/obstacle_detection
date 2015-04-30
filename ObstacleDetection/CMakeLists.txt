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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)



#opengl_exe
rosbuild_add_executable(opengl_exe src/main_opengl.cpp src/CoordSystemKinect.cpp src/TrigLookup.cpp)
rosbuild_add_compile_flags(opengl_exe -lfreenect -lGL -lGLU -lglut)
rosbuild_add_link_flags(opengl_exe -lfreenect -lGL -lGLU -lglut)
target_link_libraries(opengl_exe -lfreenect -lGL -lGLU -lglut)



#simple_exe
rosbuild_add_executable(simple_exe src/main_simple.cpp src/CoordSystemKinect.cpp src/TrigLookup.cpp)
rosbuild_add_compile_flags(simple_exe -lfreenect -lGL -lGLU -lglut)
rosbuild_add_link_flags(simple_exe -lfreenect -lGL -lGLU -lglut)
target_link_libraries(simple_exe -lfreenect -lGL -lGLU -lglut)
