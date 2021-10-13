cmake_minimum_required(VERSION 3.5)
project(inst_pkg)

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

# **********************************************
# CKim - Dependencies (headers and lib files) should be 
#        added here. It can be conveniently set by using 
#        CMake's find_package macro or include_directories
#        and link_directories
# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)

include_directories("include/inst_pkg/")
# link_directories("${CMAKE_BINARY_DIR}/my_lib/install/usr/local/lib")
# **********************************************

# **********************************************
# CKim - Files to be compiled and built
add_executable(inst_node src/main.cpp src/DxlMaster.cpp)
ament_target_dependencies(inst_node rclcpp std_msgs dynamixel_sdk)
# **********************************************

# **********************************************
# CKim - Installation of built node
install(TARGETS
  inst_node
  DESTINATION lib/${PROJECT_NAME})
# **********************************************

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
