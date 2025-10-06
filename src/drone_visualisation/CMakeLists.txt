cmake_minimum_required(VERSION 3.12)
project(drone_visualisation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install launch/URDF/meshes (ok if some folders are empty)
install(DIRECTORY
  launch
  urdf
  meshes
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Register the package in the ament index (so package:// resolves)
install(FILES
  resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

# Ship package.xml alongside the assets (handy for inspection)
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
