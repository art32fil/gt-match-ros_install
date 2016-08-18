#include_directories(../lib)
#ADD_EXECUTABLE(APITest main.cpp)

#TARGET_LINK_LIBRARIES(APITest floorplan)
#install(TARGETS APITest DESTINATION ${OUTPUT})

#ADD_EXECUTABLE(read_xml read_xml.cpp)

#TARGET_LINK_LIBRARIES(read_xml floorplan)
#install(TARGETS read_xml DESTINATION ${OUTPUT})

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(xml2lcm)

add_executable(gt-xml2lcm xml2lcm.cpp)
target_link_libraries(gt-xml2lcm jpeg) 
pods_use_pkg_config_packages(gt-xml2lcm eigen3 groundtruth_utils
		  path-util visualization lcmtypes_visualization opencv)
pods_install_executables(gt-xml2lcm)
target_link_libraries(gt-xml2lcm ${CHOLMODINSUITESPARSE_LIBRARIES} boost_system)

add_executable(gt-xml2lcm-multiple xml2lcm_multiple.cpp)
target_link_libraries(gt-xml2lcm-multiple jpeg) 
pods_use_pkg_config_packages(gt-xml2lcm-multiple eigen3 groundtruth_utils
		  path-util visualization lcmtypes_visualization opencv)
pods_install_executables(gt-xml2lcm-multiple)
target_link_libraries(gt-xml2lcm-multiple ${CHOLMODINSUITESPARSE_LIBRARIES} boost_system)

