cmake_minimum_required(VERSION 3.8)
project(bringup)  # 包名严格=bringup

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 仅保留核心依赖（避免launch_ros导致的CMake错误）
find_package(ament_cmake REQUIRED)

# 核心：将launch目录安装到ROS2标准路径
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch  # 安装到share/bringup/launch
  PATTERN "*.py"  # 仅安装py文件
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
              GROUP_READ GROUP_EXECUTE
              WORLD_READ WORLD_EXECUTE
)

# 必需的ament打包指令
ament_package()
