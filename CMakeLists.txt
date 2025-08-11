cmake_minimum_required(VERSION 3.8)
project(robofer)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)

add_library(robo_eyes_lib
  src/eyes.cpp
)

ament_target_dependencies(robo_eyes_lib rclcpp)


add_executable(robo_eyes_sim src/eyes_node.cpp)
ament_target_dependencies(robo_eyes_sim rclcpp)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EyesCommand.msg"
  "msg/ServoGoal.msg"
  "msg/SoundRequest.msg"
  DEPENDENCIES std_msgs
)

target_link_libraries(robo_eyes_lib ${OpenCV_LIBS})
target_link_libraries(robo_eyes_sim robo_eyes_lib ${OpenCV_LIBS})

install(TARGETS robo_eyes_sim robo_eyes_lib
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_package()

