cmake_minimum_required(VERSION 3.0.2)
project(planning)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if (COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_FLAGS} doesn't have C++11 support.
                         Please use a different C++ compiler")
endif ()

set(ADDITIONAL_CXX_FLAG "-Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")

# Eigen
include(cmake/FindEigen.cmake)
include_directories(${EIGEN_INCLUDE_DIR})

# glog
include(cmake/glog.cmake)
include_directories(${GLOG_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  map_server
  roscpp
  std_msgs
  tf
  nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES planning
CATKIN_DEPENDS map_server roscpp std_msgs tf nav_msgs
#  DEPENDS system_lib
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_library(head_src
  include/${PROJECT_NAME}/type.h
  include/${PROJECT_NAME}/state_node.h
  include/${PROJECT_NAME}/timer.h
  
  include/${PROJECT_NAME}/init_pose_subscriber.h
  src/init_pose_subscriber.cpp
  
  include/${PROJECT_NAME}/goal_pose_subscriber.h
  src/goal_pose_subscriber.cpp
  
  include/${PROJECT_NAME}/planning_method_flow.h
  src/planning_method_flow.cpp
  
  include/${PROJECT_NAME}/costmap_subscriber.h
  src/costmap_subscriber.cpp
  
  include/${PROJECT_NAME}/planning_method.h
  src/planning_method.cpp
  
  include/${PROJECT_NAME}/rs_path.h
  src/rs_path.cpp
)


add_executable(run_planning app/run_planning.cpp)

add_dependencies(head_src ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(run_planning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(head_src
  ${catkin_LIBRARIES}
  ${GLOG_LIBRARIES}
)
target_link_libraries(run_planning
  head_src
  ${catkin_LIBRARIES}
)
