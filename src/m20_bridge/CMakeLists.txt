cmake_minimum_required(VERSION 3.8)
project(m20_bridge)

find_package(ament_cmake REQUIRED)

# Python bridge node (self-contained; invoked as `ros2 run m20_bridge m20_udp_node.py`)
install(PROGRAMS
  scripts/m20_udp_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
