cmake_minimum_required(VERSION 3.5)
project(tmc_manipulation_types)

find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include ${EIGEN3_INCLUDE_DIRS})
ament_export_libraries()
ament_export_dependencies()

ament_package()