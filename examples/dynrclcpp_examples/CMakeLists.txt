cmake_minimum_required(VERSION 3.5)
project(dynrclcpp_examples)

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
find_package(rcl REQUIRED)
find_package(rcl_action REQUIRED)
find_package(dynmsg REQUIRED)
find_package(dynrclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(string_publisher src/string_publisher.cpp)
target_include_directories(string_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  string_publisher
  "rcl"
  "rcl_action"
  "dynmsg"
  "dynrclcpp"
  "ament_index_cpp"
)

add_executable(int_publisher src/int_publisher.cpp)
target_include_directories(int_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  int_publisher
  "rcl"
  "rcl_action"
  "dynmsg"
  "dynrclcpp"
  "ament_index_cpp"
)

add_executable(multi_int32_publisher src/multi_int32_publisher.cpp)
target_include_directories(multi_int32_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  multi_int32_publisher
  "rcl"
  "rcl_action"
  "dynmsg"
  "dynrclcpp"
  "ament_index_cpp"
)

add_executable(point_publisher src/point_publisher.cpp)
target_include_directories(point_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  point_publisher
  "rcl"
  "rcl_action"
  "dynmsg"
  "dynrclcpp"
  "ament_index_cpp"
)


add_executable(string_subscriber src/string_subscriber.cpp)
target_include_directories(string_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  string_subscriber
  "rcl"
  "rcl_action"
  "dynmsg"
  "dynrclcpp"
  "ament_index_cpp"
)

add_executable(int_subscriber src/int_subscriber.cpp)
target_include_directories(int_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  int_subscriber
  "rcl"
  "rcl_action"
  "dynmsg"
  "dynrclcpp"
  "ament_index_cpp"
)

add_executable(multi_float64_subscriber src/multi_float64_subscriber.cpp)
target_include_directories(multi_float64_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  multi_float64_subscriber
  "rcl"
  "rcl_action"
  "dynmsg"
  "dynrclcpp"
  "ament_index_cpp"
)


add_executable(add_two_ints_client src/add_two_ints_client.cpp)
target_include_directories(add_two_ints_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  add_two_ints_client
  "rcl"
  "rcl_action"
  "dynmsg"
  "dynrclcpp"
  "ament_index_cpp"
)

add_executable(add_two_ints_service src/add_two_ints_service.cpp)
target_include_directories(add_two_ints_service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  add_two_ints_service
  "rcl"
  "rcl_action"
  "dynmsg"
  "dynrclcpp"
  "ament_index_cpp"
)

install(TARGETS 
  string_publisher
  int_publisher
  multi_int32_publisher
  point_publisher
  string_subscriber
  int_subscriber
  multi_float64_subscriber
  add_two_ints_client
  add_two_ints_service
  DESTINATION lib/${PROJECT_NAME})

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
