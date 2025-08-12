cmake_minimum_required(VERSION 3.0.2)
project(constrained_icp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
)
find_package(Eigen3 REQUIRED)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(constrained_icp_node src/constrained_icp_node.cpp)
target_link_libraries(constrained_icp_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
