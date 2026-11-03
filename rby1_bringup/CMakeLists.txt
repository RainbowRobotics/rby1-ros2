cmake_minimum_required(VERSION 3.8)
project(rby1_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Linters (ament_lint_common); C++, Python and CMake use a 120 column limit.
  # cppcheck is skipped: Humble ships a slow version and it misreads the gtest macros.
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cppcheck ament_cmake_cpplint ament_cmake_flake8 ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
  ament_flake8(MAX_LINE_LENGTH 120)

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_isaac_ros_contract.py TIMEOUT 180)
endif()

ament_package()
