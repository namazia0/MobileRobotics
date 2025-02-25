cmake_minimum_required(VERSION 2.8.3)
project(unit_05)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)


catkin_package(
CATKIN_DEPENDS cv_bridge roscpp 
#DEPENDS system_lib

)
find_package( OpenCV 3 REQUIRED )

include_directories(
  ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS}
)


add_executable(guess_01 src/guess_01.cpp)
target_link_libraries (guess_01 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )

add_executable(guess_02 src/guess_02.cpp)
target_link_libraries (guess_02 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )



