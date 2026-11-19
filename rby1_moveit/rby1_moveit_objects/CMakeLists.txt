cmake_minimum_required(VERSION 3.10)
project(rby1_moveit_objects)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)

# The library: object specs, YAML files, the planning-scene client and the modules.
# The executables are thin; other packages can link it too.
add_library(${PROJECT_NAME} SHARED
  src/object_spec.cpp
  src/scene.cpp
  src/command.cpp
  src/modules.cpp
  src/point_object.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME} PRIVATE ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(${PROJECT_NAME} rclcpp moveit_msgs shape_msgs geometry_msgs geometric_shapes)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

add_executable(scene src/scene_cli.cpp)
target_link_libraries(scene ${PROJECT_NAME})

add_executable(publish_objects src/publish_objects.cpp)
target_link_libraries(publish_objects ${PROJECT_NAME})
ament_target_dependencies(publish_objects ament_index_cpp sensor_msgs)

add_executable(object_at_point src/object_at_point.cpp)
target_link_libraries(object_at_point ${PROJECT_NAME})
target_include_directories(object_at_point PRIVATE ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(object_at_point tf2 tf2_ros)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS scene publish_objects object_at_point DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scene test/test_scene.cpp)
  target_link_libraries(test_scene ${PROJECT_NAME})
  target_compile_definitions(test_scene PRIVATE
    EXAMPLE_SCENE="${CMAKE_CURRENT_SOURCE_DIR}/config/example_scene.yaml")
  ament_add_gtest(test_modules test/test_modules.cpp)
  target_link_libraries(test_modules ${PROJECT_NAME})
  target_compile_definitions(test_modules PRIVATE
    EXAMPLE_OBJECTS="${CMAKE_CURRENT_SOURCE_DIR}/config/objects.yaml"
    GRIPPER_OBJECTS="${CMAKE_CURRENT_SOURCE_DIR}/../../rby1_tools/model/rby1_gripper/gripper.yaml")
  ament_add_gtest(test_point_object test/test_point_object.cpp)
  target_link_libraries(test_point_object ${PROJECT_NAME})
  target_compile_definitions(test_point_object PRIVATE
    POINT_OBJECT="${CMAKE_CURRENT_SOURCE_DIR}/config/point_object.yaml")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp moveit_msgs shape_msgs geometry_msgs geometric_shapes yaml-cpp)
ament_package()
