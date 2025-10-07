cmake_minimum_required(VERSION 3.10)
project(connectors_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

ament_package()



