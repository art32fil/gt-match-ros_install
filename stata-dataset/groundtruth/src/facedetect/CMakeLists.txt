#--------------------------------------------------------
# The CMakeLists.txt for:                 mapdb-lcm
# Author(s):                          Hordur Johannsson
#--------------------------------------------------------

cmake_minimum_required(VERSION 2.6)

add_definitions(`pkg-config --cflags glib-2.0 gmodule-2.0`)
add_definitions(`pkg-config --libs glib-2.0 gmodule-2.0 gthread-2.0 gobject-2.0`)

link_libraries(opencv_core3 opencv_imgproc3 opencv_highgui3 opencv_ml3 
      opencv_video3 opencv_features2d3 opencv_calib3d3 opencv_objdetect3 
      opencv_contrib opencv_legacy lcmtypes_visualization opencv_imgproc3 opencv_imgcodecs3)

add_executable(gt-facedetect facedetect.cpp )
pods_use_pkg_config_packages(gt-facedetect opencv )
pods_install_executables(gt-facedetect)

