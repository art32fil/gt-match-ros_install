#--------------------------------------------------------
# The CMakeLists.txt for:                 mapdb-lcm
# Author(s):                          Hordur Johannsson
#--------------------------------------------------------

cmake_minimum_required(VERSION 2.6)

#set(CMAKE_BUILD_TYPE debug)
#set(CMAKE_BUILD_TYPE release)
#message(${CMAKE_BUILD_TYPE})

# glib
add_definitions(`pkg-config --cflags glib-2.0 gmodule-2.0`)
add_definitions(`pkg-config --libs glib-2.0 gmodule-2.0 gthread-2.0 gobject-2.0`)

link_libraries(isam scanmatch)

link_libraries(opencv_core opencv_imgproc opencv_highgui opencv_ml 
      opencv_video opencv_features2d opencv_calib3d opencv_objdetect 
      opencv_contrib opencv_legacy lcmtypes_visualization )

add_executable(gt-writepcd-ros writepcd_ros.cpp )
target_link_libraries(gt-writepcd-ros )
pods_use_pkg_config_packages(gt-writepcd-ros lcm eigen3 opencv bot2-core groundtruth_utils
    lcmtypes_visualization)
pods_install_executables(gt-writepcd-ros)
target_link_libraries(gt-writepcd-ros ${CHOLMODINSUITESPARSE_LIBRARIES} boost_system)


#add_executable(gt-read-match-then-smooth read_match_then_smooth.cpp )
#target_link_libraries(gt-read-match-then-smooth isam cholmod)
#pods_use_pkg_config_packages(gt-read-match-then-smooth lcm eigen3 opencv bot2-core groundtruth_utils
#    lcmtypes_visualization  lcmtypes_kinect )
#pods_install_executables(gt-read-match-then-smooth)
