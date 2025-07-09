cmake_minimum_required(VERSION 3.8)
project(semantic_voxel_mapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(assimp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_definitions(${PCL_DEFINITIONS})
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp src/file_parser.cpp src/semantic_map.cpp)
target_include_directories(${PROJECT_NAME}_node PRIVATE include/semantic_voxel_mapper ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_node tinyxml2::tinyxml2 ${PCL_LIBRARIES} ${ASSIMP_LIBRARIES})

ament_target_dependencies(${PROJECT_NAME}_node rclcpp visualization_msgs)


install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

ament_package()
