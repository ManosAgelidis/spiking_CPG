cmake_minimum_required(VERSION 2.8.3)
project(lamprey_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  roscpp
)

find_package(gazebo REQUIRED)

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
  LIBRARIES lamprey_control
  CATKIN_DEPENDS gazebo_msgs roscpp
  DEPENDS gazebo 
)

###########
## Build ##
###########

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/lamprey_control_plugin.cpp
)

add_library(lamprey_simple_fluids_controller SHARED
  src/lamprey_simple_fluids_controller.cpp
  src/Plotter.cpp
)

add_library(lamprey_drag_force SHARED
  src/lamprey_drag_force.cpp
)

add_library(camera_move SHARED
  src/camera_move.cpp
)

find_package(PythonLibs 2.7)
find_package(gram_savitzky_golay REQUIRED)
target_include_directories(lamprey_simple_fluids_controller PRIVATE ${CMAKE_SOURCE_DIR}/lamprey_control/external_libraries/matplotlib-cpp ${PYTHON_INCLUDE_DIRS})
target_link_libraries(lamprey_simple_fluids_controller ${PYTHON_LIBRARIES} gram_savitzky_golay)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

target_link_libraries(lamprey_simple_fluids_controller
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  yaml-cpp
)

target_link_libraries(lamprey_drag_force
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  yaml-cpp
)

target_link_libraries(camera_move
  ${GAZEBO_LIBRARIES}
)
## Install
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS lamprey_simple_fluids_controller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS lamprey_drag_force
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS camera_move
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)