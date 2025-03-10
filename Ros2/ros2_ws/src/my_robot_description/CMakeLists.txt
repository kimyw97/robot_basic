cmake_minimum_required(VERSION 3.5)
project(my_robot_description)

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED) # xacro 패키지 추가
find_package(sensor_msgs REQUIRED)

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

# urdf 파일 존재 여부 확인 (선택 사항)
if(EXISTS "${CMAKE_SOURCE_DIR}/urdf")
  install(DIRECTORY urdf/
          DESTINATION share/${PROJECT_NAME}/urdf
          FILES_MATCHING PATTERN "*.urdf")
else()
  message(WARNING "urdf directory not found in ${CMAKE_SOURCE_DIR}")
endif()

install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)
        
install(FILES rviz/testbot_edu.rviz
        DESTINATION share/${PROJECT_NAME}/rviz)
        
ament_package()
