cmake_minimum_required(VERSION 2.8.3)
project(ur5_moveit_simple)

# C++ 14
add_compile_options(-std=c++14)

# Warnings
add_definitions(-W -Wall -Wextra
  -Wwrite-strings -Wunreachable-code -Wpointer-arith
  -Winit-self -Wredundant-decls
  -Wno-unused-parameter -Wno-unused-function)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  rosparam_shortcuts
  rviz_visual_tools
  moveit_simple
  moveit_visual_tools
  tf
  moveit_core
  robotiq_2f_gripper_control
)

# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

###################################
## Catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    tf
    moveit_core
    robotiq_2f_gripper_control
  INCLUDE_DIRS
  LIBRARIES
    ${PROJECT_NAME}
  DEPENDS
    Boost
    EIGEN3
)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  SYSTEM
    ${Boost_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ executable
add_executable(
  ${PROJECT_NAME}_ur5_demo
  src/ur5_demo.cpp
)

# Rename C++ executable without namespace
set_target_properties(
  ${PROJECT_NAME}_ur5_demo
  PROPERTIES
    OUTPUT_NAME ur5_demo PREFIX ""
)

# Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}_ur5_demo
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS
    ${PROJECT_NAME}_ur5_demo
  ARCHIVE DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark roslaunch files for installation
install(
  DIRECTORY
    launch
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Mark config files for installation
install(
  DIRECTORY
    config
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Test for correct C++ source code
roslint_cpp()
