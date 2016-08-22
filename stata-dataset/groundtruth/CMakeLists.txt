cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(../isam_pod/include)
link_directories(../isam_pod/lib)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
find_package(CHOLMODinSuiteSparse REQUIRED)
include_directories(${CHOLMODINSUITESPARSE_INCLUDES})
link_directories(${suitesparse_dir}/CHOLMOD/Lib)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME groundtruth)
include(cmake/pods.cmake)


# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

# Core library
add_subdirectory(src/plan_utils)

# simple floorplan visualiser
add_subdirectory(src/xml2lcm)

# LCM log application
# Currently requires [non-essential] kinect pod:
#add_subdirectory(src/match_lcm)
# ROS bag application
add_subdirectory(src/match_ros)

# Tool to convert dense laser GT file to dense camera GT file
#add_subdirectory(src/laser2camera_ros)

# Explore the file images:
add_subdirectory(src/explode_bag)
add_subdirectory(src/facedetect)

# Tool to visualize tf messages in a bag file by publishing them to LCM
#add_subdirectory(src/tf2lcm_ros)

# Wall Filtering Tool:
#add_subdirectory(src/wallfilter)

# Tools
# - reproject the results of the ground truth tools:
# - carry out smoothing
# - tweak smoothed results
add_subdirectory(src/tools)

# Tool to convert Openni ROS data to PCD files [Required PCL]
add_subdirectory(src/writepcd_ros)


# Occupancy Map creator - requires part of envoy/carmen codebase:
#add_subdirectory(src/gridmap_creator)
#add_subdirectory(src/gridmap_to_3dmap)

# Tool to convert walls to 3d model. Requires KMCL/PCL:
# Would not compile in this newer project
#add_subdirectory(src/walls_to_3dmodel)

# Unnecessary Demo utilities
#add_subdirectory(src/raytrace)
#add_subdirectory(src/match)

# Viewer:
add_subdirectory(src/groundtruth_renderer)
add_subdirectory(src/viewer)

