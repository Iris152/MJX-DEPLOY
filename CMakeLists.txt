cmake_minimum_required(VERSION 3.16)
project(mjx_deploy_go2_ahac LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(MJX_DEPLOY_ENABLE_ROS2 "Enable ROS2 /velocity_command support" OFF)
option(MJX_DEPLOY_BUILD_STAND_EXAMPLE "Build Unitree Go2 stand example" ON)

set(OPEN_DIFFLOCO_ENABLE_ROS2 ${MJX_DEPLOY_ENABLE_ROS2} CACHE BOOL "" FORCE)

add_subdirectory(cpp/deploy_blind_nolinvel_nokinref)

if(MJX_DEPLOY_BUILD_STAND_EXAMPLE)
  find_package(unitree_sdk2 REQUIRED)
  add_executable(go2_stand_example examples/unitree_sdk2/go2_stand_example.cpp)
  target_link_libraries(go2_stand_example PRIVATE unitree_sdk2)
endif()

