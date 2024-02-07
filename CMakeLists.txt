cmake_minimum_required(VERSION 3.0.2)
project(multi_dynamic)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  tf
  laser_geometry
)

# SIN ESTO NO FUNCIONA (???????????)
catkin_package(
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Las librerías que se usan
set(multi_util src/multi_util.cpp src/path_follower.cpp)
add_library(multi_util ${multi_util})

#set(util src/util/funciones.cpp)
set(util src/util/funciones.cpp  src/util/useless.cpp src/util/fmm_2.cpp src/util/generic_fmm.cpp src/util/bresenham.cpp src/util/dynamic.cpp src/util/formation.cpp src/util/geometry.cpp   src/util/eq_segm.cpp src/util/grad_segm_def.cpp src/util/it_grad_segm.cpp    src/util/basic_hungarian.cpp src/util/hungarian.cpp src/util/nearest_neighbor.cpp  src/util/cluster.cpp  src/util/chain_plan.cpp)
add_library(util ${util})

# Ejecutables
add_executable(explorer_simple src/explorer_simple.cpp)
target_link_libraries(explorer_simple
  LINK_PUBLIC multi_util
  LINK_PUBLIC util
  ${catkin_LIBRARIES}
)

add_executable(explorer_dynamic src/explorer_dynamic.cpp)
target_link_libraries(explorer_dynamic
  LINK_PUBLIC multi_util
  LINK_PUBLIC util
  ${catkin_LIBRARIES}
)

add_executable(explorer_vertex src/explorer_vertex.cpp)
target_link_libraries(explorer_vertex
  LINK_PUBLIC multi_util
  LINK_PUBLIC util
  ${catkin_LIBRARIES}
)

add_executable(explorer_edge src/explorer_edge.cpp)
target_link_libraries(explorer_edge
  LINK_PUBLIC multi_util
  LINK_PUBLIC util
  ${catkin_LIBRARIES}
)

add_executable(explorer_static src/explorer_static.cpp)
target_link_libraries(explorer_static
  LINK_PUBLIC multi_util
  LINK_PUBLIC util
  ${catkin_LIBRARIES}
)

add_executable(path_follower src/path_follower_ex.cpp)
target_link_libraries(path_follower
  LINK_PUBLIC multi_util
  LINK_PUBLIC util
  ${catkin_LIBRARIES}
)

add_executable(obstacles src/obstacles.cpp)
target_link_libraries(obstacles
  LINK_PUBLIC multi_util
  LINK_PUBLIC util
  ${catkin_LIBRARIES}
)