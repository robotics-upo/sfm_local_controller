cmake_minimum_required(VERSION 3.0.2)
project(sfm_local_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  dynamic_reconfigure
  nav_core
  nav_msgs
  pluginlib
  roscpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  dynamic_obstacle_detector
)


## Generate dynamic reconfigure parameters in the 'cfg' folder
 generate_dynamic_reconfigure_options(
   cfg/SFMLocalController.cfg
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
    sfm_local_controller 
    sfm_controller_ros
  CATKIN_DEPENDS geometry_msgs lightsfm nav_core nav_msgs pluginlib roscpp sensor_msgs tf2 tf2_geometry_msgs tf2_ros visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  /usr/local/include  #to find lightsfm
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
    src/sensor_interface.cpp
    src/collision_checker.cpp
    src/sfm_controller.cpp
 )

 add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )

 ## Declare a C++ library
 add_library(sfm_controller_ros
    src/sfm_controller_ros.cpp
 )

 add_dependencies(sfm_controller_ros ${sfm_controller_ros_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(sfm_controller_ros
   ${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )


install(TARGETS
            ${PROJECT_NAME}
            sfm_controller_ros
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       #RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES sfm_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

