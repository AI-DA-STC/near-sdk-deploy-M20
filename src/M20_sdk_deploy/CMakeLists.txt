cmake_minimum_required(VERSION 3.10)

project(rl_deploy)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 默认构建类型
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
# 警告忽略和编译选项
add_definitions(-w)
add_compile_options(-fPIC)

# 配置选项
# Auto-detect the target platform from the actual target arch so the correct
# prebuilt third_party libs (onnxruntime) AND link flags are chosen WITHOUT
# relying on -DBUILD_PLATFORM being threaded through colcon --cmake-args (which
# silently fell back to x86 and linked the x86 libonnxruntime.so). On the arm64
# deploy image (native on the robot, or QEMU on the workstation)
# CMAKE_SYSTEM_PROCESSOR is aarch64; an ordinary x86 dev build stays x86.
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(BUILD_PLATFORM "arm" CACHE STRING "Target platform (x86 or arm)" FORCE)
else()
  set(BUILD_PLATFORM "x86" CACHE STRING "Target platform (x86 or arm)")
endif()
# NB: no CMAKE_C/CXX_COMPILER override here — setting it after project() (line 3)
# is a no-op, and the deploy image compiles natively (real gcc; under QEMU when
# building arm64 on an x86 workstation), so no cross toolchain is needed.

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(drdds REQUIRED)

set(THIRD_PARTY ${CMAKE_CURRENT_SOURCE_DIR}/third_party)

include_directories(
  include/types
  include/utils
  interface/robot
  interface/user_command
  state_machine
  run_policy
  ${THIRD_PARTY}/eigen/
  ${THIRD_PARTY}/gamepad/include
  ${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM}/include
)

# 子目录
add_subdirectory(interface)

# 运行时路径设置
set(CMAKE_EXE_LINKER_FLAGS "-Wl,--disable-new-dtags")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


set(CMAKE_INSTALL_RPATH 
  ${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM}/lib/
  ../lib/onnxruntime/
)

file(GLOB_RECURSE STATE_MACHINE_SRC "state_machine/*.c*")

add_executable(rl_deploy main.cpp ${STATE_MACHINE_SRC})

add_library(onnxruntime SHARED IMPORTED)
set_target_properties(onnxruntime PROPERTIES IMPORTED_LOCATION "${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM}/lib/libonnxruntime.so")

ament_target_dependencies(rl_deploy
  rclcpp
  geometry_msgs
  std_msgs
  drdds
)
if(BUILD_PLATFORM STREQUAL "arm")
  target_link_libraries(rl_deploy
    interface
    onnxruntime
    -lpthread
    -lm
    -lrt
    -ldl
    -lstdc++
  )
else()
  target_link_libraries(rl_deploy
    interface
    onnxruntime
    -lpthread
    -lm
    -lrt
    -ldl
    -lstdc++
    -levdev
  )
endif()


install(TARGETS rl_deploy
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts as executables
install(PROGRAMS
  interface/robot/simulation/gazebo_controller_ros2.py
  interface/robot/simulation/mujoco_simulation_ros2.py
  scripts/ply_to_2d_map.py
  scripts/pose_ape_evaluator.py
  scripts/route_server.py
  scripts/path_follower.py
  scripts/m20_imu_adapter.py
  scripts/m20_cmd_vel_bridge.py
  scripts/linear_orchestrator.py
  scripts/velodyne_timestamp_injector.py
  scripts/restamp_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch model config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()

