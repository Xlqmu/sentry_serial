cmake_minimum_required(VERSION 3.8)
project(rm_serial)

# 使用 c++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 查找所需的包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)

# 添加头文件目录
include_directories(
  include
  ${CMAKE_BINARY_DIR}/rosidl_generator_cpp
)

# 添加可执行文件
add_executable(serial_driver 
  src/serial.cpp
)

# 链接依赖库（先为 target 指定链接库）
target_link_libraries(serial_driver
  ${nav2_msgs_LIBRARIES}
)

# 添加其它依赖
ament_target_dependencies(serial_driver
  rclcpp
  std_msgs
  geometry_msgs
  tf2_ros
  tf2
  rm_interfaces
  nav2_msgs
)

# 安装可执行文件
install(TARGETS
  serial_driver
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装 launch 文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()