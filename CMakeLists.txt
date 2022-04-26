cmake_minimum_required(VERSION 2.8.3)
project(mav_simple_planner)

add_definitions(-std=c++14)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
)

## Declare a C++ library
cs_add_library(${PROJECT_NAME}_library
  src/polynomial_interpolator.cpp
  src/simple_planner.cpp
)

cs_add_executable(mav_simple_planner src/simple_planner_node.cpp)
target_link_libraries(mav_simple_planner ${catkin_LIBRARIES} ${PROJECT_NAME}_library)

##########
# EXPORT #
##########
cs_install()
cs_export()
