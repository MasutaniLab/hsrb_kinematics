cmake_minimum_required(VERSION 3.5)
project(tmc_robot_kinematics_model)

find_package(ament_cmake REQUIRED)
find_package(pinocchio REQUIRED)
find_package(tmc_manipulation_types REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(pinocchio_wrapper SHARED src/pinocchio_wrapper.cpp)
target_include_directories(pinocchio_wrapper PUBLIC include ${pinocchio_INCLUDE_DIRS})
target_link_libraries(pinocchio_wrapper ${pinocchio_LIBRARIES})
ament_target_dependencies(pinocchio_wrapper tmc_manipulation_types)

add_library(numeric_ik_solver src/numeric_ik_solver.cpp)
target_include_directories(numeric_ik_solver PUBLIC include)
ament_target_dependencies(numeric_ik_solver tmc_manipulation_types)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_numeric_ik_urdf test/test_numeric_ik_urdf.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(test_numeric_ik_urdf pinocchio_wrapper numeric_ik_solver)

  ament_add_gtest(pinocchio_wrapper_test test/pinocchio_wrapper-test.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(pinocchio_wrapper_test pinocchio_wrapper)
endif()

install(TARGETS numeric_ik_solver pinocchio_wrapper
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include ${pinocchio_INCLUDE_DIRS})
ament_export_libraries(numeric_ik_solver pinocchio_wrapper)
ament_export_dependencies(tmc_manipulation_types)

ament_package()
