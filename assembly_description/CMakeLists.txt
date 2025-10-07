cmake_minimum_required(VERSION 3.10)
project(assembly_description)

find_package(ament_cmake REQUIRED)

# Install directories so resources resolve via package share
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_package()