cmake_minimum_required(VERSION 3.8)
project(egocar_base_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 尋找 colcon 和 ament 的相關套件
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# 新增要編譯的執行檔
# 名稱為 base_node，原始碼是 src/base_node.cpp
add_executable(base_node src/base_node.cpp)
add_executable(base_node_X3 src/base_node_X3.cpp)

# 連結執行檔所需的依賴項
ament_target_dependencies(base_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
)

ament_target_dependencies(base_node_X3
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
)

# 安裝規則：將編譯好的執行檔安裝到 lib/ego_robot_base/ 目錄下
install(TARGETS
  base_node
  base_node_X3
  DESTINATION lib/${PROJECT_NAME}
)

# 安裝 launch 檔案和 Rviz 設定檔 (如果有的話)
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}
# )
# install(DIRECTORY
#   rviz
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()
