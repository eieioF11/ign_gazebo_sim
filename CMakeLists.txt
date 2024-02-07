cmake_minimum_required(VERSION 3.8)
project(ign_gazebo_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_compile_options(-O3)

# ament_auto_add_executable(${PROJECT_NAME} src/main.cpp)

install(DIRECTORY
  config
  launch
  rviz
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_auto_package()