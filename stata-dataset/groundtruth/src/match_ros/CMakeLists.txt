rosbuild_add_executable(gt-match-ros match_ros.cpp)
rosbuild_add_executable(gt-compute-transforms-ros compute_transforms_ros.cpp)

# glib
add_definitions(`pkg-config --cflags glib-2.0 gmodule-2.0`)
add_definitions(`pkg-config --libs glib-2.0 gmodule-2.0 gthread-2.0 gobject-2.0`)

link_libraries(opencv_core opencv_imgproc opencv_highgui opencv_ml 
      opencv_video opencv_features2d opencv_calib3d opencv_objdetect 
      opencv_contrib opencv_legacy lcmtypes_visualization )

target_link_libraries(gt-match-ros boost_thread 
                                   boost_program_options )

target_link_libraries(gt-compute-transforms-ros boost_thread 
                                                boost_program_options )

set(GT_PACKAGES lcm eigen3 opencv bot2-core groundtruth_utils
                lcmtypes_bot2-core
                lcmtypes_visualization )

if ($ENV{ROS_DISTRO} STREQUAL fuerte)
  pods_use_pkg_config_packages(gt-match-ros ${GT_PACKAGES} rosbag)
  pods_use_pkg_config_packages(gt-compute-transforms-ros ${GT_PACKAGES} rosbag)
else ($ENV{ROS_DISTRO} STREQUAL fuerte)
  pods_use_pkg_config_packages(gt-match-ros ${GT_PACKAGES})
  pods_use_pkg_config_packages(gt-compute-transforms-ros ${GT_PACKAGES})
endif ($ENV{ROS_DISTRO} STREQUAL fuerte)

pods_install_executables(gt-match-ros)
pods_install_executables(gt-compute-transforms-ros)
target_link_libraries(gt-match-ros ${CHOLMODINSUITESPARSE_LIBRARIES} boost_system)
target_link_libraries(gt-compute-transforms-ros ${CHOLMODINSUITESPARSE_LIBRARIES} boost_system)
