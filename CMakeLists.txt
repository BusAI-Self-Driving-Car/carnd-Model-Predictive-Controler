cmake_minimum_required (VERSION 3.5)
project(MPC)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CXX_FLAGS "-Wall")

find_package(Eigen3 REQUIRED)

set(sources src/mpc.cpp src/main.cpp)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(mpc ${sources})

target_link_libraries(mpc
  ipopt
  z
  ssl
  uv
  uWS
)
