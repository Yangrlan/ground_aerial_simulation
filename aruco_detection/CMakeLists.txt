cmake_minimum_required(VERSION 3.0.2)
project(aruco_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
)

find_package(nlohmann_json REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(aruco_detection_node src/aruco_detection_node.cpp)
target_link_libraries(aruco_detection_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
