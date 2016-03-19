cmake_minimum_required(VERSION 2.8.3)
project(proxemics_anytimerrts)

SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  message_generation
  nav_core
  nav_msgs
  navfn
  pluginlib
  roscpp
  tf
  visualization_msgs
)

find_package(cmake_modules REQUIRED)

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
  message("OPENMP FOUND")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


# add qt4 to the project
FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(Boost REQUIRED)

 add_message_files(
   FILES
   Path.msg
 )

 generate_messages(
   DEPENDENCIES
   geometry_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}

  CATKIN_DEPENDS
    geometry_msgs
    nav_core
    nav_msgs
    roscpp
    rospy
    std_msgs
    tf
    visualization_msgs
    costmap_2d
    pluginlib

  LIBRARIES
    ${PROJECT_NAME}
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/
  ${CMAKE_CURRENT_SOURCE_DIR}/src/
  ${CMAKE_CURRENT_SOURCE_DIR}
)
set(SOCNAV 
    include/${PROJECT_NAME}/pose.h
    include/${PROJECT_NAME}/velocity.h
    include/${PROJECT_NAME}/discrete_state.h
    include/${PROJECT_NAME}/state.h
    include/${PROJECT_NAME}/heuristics.h
    include/${PROJECT_NAME}/state_discretizer.h
    include/${PROJECT_NAME}/trajectory_rollout.h
    include/${PROJECT_NAME}/hasher.h
#    include/${PROJECT_NAME}/Path.h
    include/${PROJECT_NAME}/cost_manager.h
    include/${PROJECT_NAME}/dynamic_costmap.h
   )
set(SOURCES_RRT
#    src/
     include/${PROJECT_NAME}/anytimerrts_planner.h
   )
set(SOURCES
    src/thetastar_leading_rrt.cpp
    include/${PROJECT_NAME}/thetastar_leading_rrt.h 
    include/${PROJECT_NAME}/costmap_model.cpp 
    include/${PROJECT_NAME}/trajectory.cpp 
    include/${PROJECT_NAME}/grid.cpp 
    include/${PROJECT_NAME}/cell.cpp    
   )

# RRT PLANNER
## Declare KdTree lib
add_library(SRLGLOBALPLANNER_KDTREE_LIB ${CMAKE_CURRENT_SOURCE_DIR}/include/smp/external_libraries/kdtree/kdtree.c)

# Plugin
add_library(
  ${PROJECT_NAME}
    src/planner_core.cpp
#    src/tb_lattice.cpp
    src/cost_manager.cpp
    src/trajectory_rollout.cpp
    src/heuristics.cpp
    src/dijkstra.cpp
    src/state_discretizer.cpp
    ${SOURCES_RRT}
    ${SOURCES}
    ${SOCNAV}
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${BOOST_LIBRARIES}
    SRLGLOBALPLANNER_KDTREE_LIB
)


########
# TODO
########
# Add install items if it is needed.
