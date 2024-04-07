cmake_minimum_required(VERSION 3.5)
project(dynrclcpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(dynmsg REQUIRED)
find_package(rcl REQUIRED)
find_package(rcl_action REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_library(dynrclcpp STATIC
  src/node_base.cpp
  src/timer_base.cpp
  src/publisher_base.cpp
  src/subscription_base.cpp
  src/typesupport_utils.cpp
)

ament_export_targets(dynrclcpp HAS_LIBRARY_TARGET) # to export the library

target_include_directories(dynrclcpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  dynrclcpp
  "dynmsg"
  "rcl"
  "rcl_action"
  "yaml_cpp_vendor"
  "ament_index_cpp"
)


target_link_libraries(dynrclcpp
  ${rcl_LIBRARIES}
  pthread # for std::thread
  yaml-cpp 
)

install(
  DIRECTORY include/dynrclcpp
  DESTINATION include
)

install(
  TARGETS dynrclcpp
  EXPORT dynrclcpp
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
