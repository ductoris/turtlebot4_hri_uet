cmake_minimum_required(VERSION 3.8)
project(turbot4_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  maps
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install executable scripts with execution permissions
install(PROGRAMS
  scripts/install_dependencies.sh
  scripts/save_map.sh
  scripts/teleop.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
