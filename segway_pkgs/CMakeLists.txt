cmake_minimum_required(VERSION 2.8.3)

project(segway_pkgs)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
)

find_package(gazebo REQUIRED)


catkin_package(
        INCLUDE_DIRS
        LIBRARIES segway_pkgs
        DEPENDS gazebo_ros roscpp rospy std_msgs
        DEPENDS system_lib
)

include_directories(
                    ${Boost_INCLUDE_DIR} 
                    ${catkin_INCLUDE_DIRS} 
                    ${GAZEBO_INCLUDE_DIRS} 
                    ${PCL_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME} src/plugin.cc)

target_link_libraries(
                    ${PROJECT_NAME} 
                    ${catkin_LIBRARIES} 
                    ${GAZEBO_LIBRARIES}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

