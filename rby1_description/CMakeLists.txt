cmake_minimum_required(VERSION 3.8)
project(rby1_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(
  DIRECTORY launch urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Linters (ament_lint_common); C++, Python and CMake use a 120 column limit.
  # cppcheck is skipped: Humble ships a slow version and it misreads the gtest macros.
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cppcheck ament_cmake_cpplint ament_cmake_flake8 ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
  ament_flake8(MAX_LINE_LENGTH 120)
endif()

ament_package()
