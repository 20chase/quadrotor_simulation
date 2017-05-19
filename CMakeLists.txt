cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_simulation)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  urdf
)

find_package(Eigen3 REQUIRED)

catkin_package(
)



include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  /usr/include
)

add_executable(quadrotor src/compute.cpp src/param.cpp src/run_sim.cpp)
target_link_libraries(quadrotor ${catkin_LIBRARIES}
${EIGEN3_LIBS}
)

