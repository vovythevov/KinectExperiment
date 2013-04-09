cmake_minimum_required (VERSION 2.6)

project (K)

set (K_VERSION_MAJOR 1)
set (K_VERSION_MINOR 0)
 
include_directories("${PROJECT_BINARY_DIR}")

# Kinect dir
set(KINECT_DIR $ENV{KINECTSDK10_DIR}) #installed in the path

set(KINECT_INCLUDE_DIRS
  ${KINECT_DIR}/inc)

# VTK Dir
find_package(VTK REQUIRED)
if(VTK_NOT_FOUND)
  FATAL_ERROR("VTK not found")
else()
  include( ${VTK_USE_FILE} )
endif()

# Include dirs
include_directories(
  ${PROJECT_BINARY_DIR}
  ${KINECT_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIR})

# Kinect Lib
set(KINECT_LIB ${KINECT_DIR}/lib/x86/Kinect10.lib) #32 bit windows

add_executable(K main.cxx)
target_link_libraries(K ${KINECT_LIB} ${VTK_LIBRARIES})
