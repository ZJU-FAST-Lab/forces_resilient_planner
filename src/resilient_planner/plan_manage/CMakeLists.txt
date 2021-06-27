cmake_minimum_required(VERSION 2.8.3)
project(resilient_planner)

add_definitions(-std=c++11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -w -g -fPIC")

find_package(catkin_simple REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories(${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} include)
include_directories(${EIGEN3_INCLUDE_DIRS} ${DECOMP_UTIL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}")

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    rospy
    std_msgs
    geometry_msgs
    quadrotor_msgs
    sensor_msgs
    visualization_msgs
    pcl_ros
    decomp_ros_utils
    path_searching
    cv_bridge
    mav_msgs
    message_generation
    occ_grid
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES resilient_planner
 CATKIN_DEPENDS occ_grid path_searching
)


include_directories(
  include
  SYSTEM 
  ${DECOMP_UTIL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/solver/normal/FORCESNLPsolver_normal/include/
  ${PROJECT_SOURCE_DIR}/solver/normal/
  ${PROJECT_SOURCE_DIR}/solver/final/FORCESNLPsolver_final/include/
  ${PROJECT_SOURCE_DIR}/solver/final/
)

link_directories(${PROJECT_SOURCE_DIR}/solver/normal/FORCESNLPsolver_normal/lib/)
link_directories(${PROJECT_SOURCE_DIR}/solver/final/FORCESNLPsolver_final/lib/)


add_executable(resilient_planner_node 
  src/resilient_planner_node.cpp 
  src/nmpc_manage.cpp
  src/nmpc_solver.cpp
  src/forces_normal.cpp
  src/forces_final.cpp
)



target_link_libraries(resilient_planner_node 
  ${catkin_LIBRARIES}
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
  libFORCESNLPsolver_normal.a
  libFORCESNLPsolver_final.a
)

add_dependencies(resilient_planner_node   
   ${${PROJECT_NAME}_EXPORTED_TARGETS} 
   ${catkin_EXPORTED_TARGETS}
)

