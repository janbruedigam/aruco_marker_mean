cmake_minimum_required(VERSION 3.0.2)
project(aruco_marker_mean)


find_package(
    catkin 
    REQUIRED 
    COMPONENTS 
    roscpp 
    std_msgs 
    geometry_msgs
    aruco_msgs
    tf2_ros
    message_generation
)

find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

file(GLOB SOURCES_CPP src/*.cpp)
set(SOURCE_FILES
  ${SOURCES_CPP}
)

add_executable(${PROJECT_NAME}_node ${SOURCE_FILES})

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
