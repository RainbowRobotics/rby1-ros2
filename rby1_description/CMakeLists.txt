cmake_minimum_required(VERSION 3.8)
project(rby1_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(
  DIRECTORY urdf meshes launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES urdf/initial_positions.yaml
  DESTINATION share/${PROJECT_NAME}/urdf
)


ament_package()
