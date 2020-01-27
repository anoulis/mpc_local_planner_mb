cmake_minimum_required(VERSION 2.8.3)
project(mpc_local_planner_mb)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
option(BUILD_EXAMPLE "Whether or not building the CppAD & Ipopt example" OFF)


find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  nav_core
  nav_msgs
  roscpp
  tf
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS
    base_local_planner
    costmap_2d
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    tf
    tf2_geometry_msgs
    tf2_ros
  INCLUDE_DIRS include
  LIBRARIES mpc_local_planner_mb
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library

add_library(mpc_local_planner_mb src/mpc_local_planner_mb.cpp
                                  src/MPC.cpp
 )
add_dependencies(mpc_local_planner_mb ${catkin_EXPORTED_TARGETS})



target_link_libraries(mpc_local_planner_mb ipopt ${catkin_LIBRARIES})


install(TARGETS mpc_local_planner_mb
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES mpclp_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(BUILD_EXAMPLE)
    ADD_EXECUTABLE( CppAD_started example/CppAD_started.cpp )
    TARGET_LINK_LIBRARIES(CppAD_started)

    ADD_EXECUTABLE( CppAD_Ipopt example/CppAD_Ipopt.cpp )
    TARGET_LINK_LIBRARIES(CppAD_Ipopt ipopt)
endif(BUILD_EXAMPLE)
