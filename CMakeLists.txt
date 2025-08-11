cmake_minimum_required(VERSION 3.8)
project(robofer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)

# 1) Librería compartida (sin prefijo "lib" en el nombre del target)
add_library(robo_eyes_lib SHARED
  src/eyes.cpp
)
ament_target_dependencies(robo_eyes_lib rclcpp)
target_link_libraries(robo_eyes_lib ${OpenCV_LIBS})
target_include_directories(robo_eyes_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 2) Ejecutable
add_executable(robo_eyes_sim src/eyes_node.cpp)
ament_target_dependencies(robo_eyes_sim rclcpp)
target_link_libraries(robo_eyes_sim robo_eyes_lib ${OpenCV_LIBS})

# 3) Mensajes
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EyesCommand.msg"
  "msg/ServoGoal.msg"
  "msg/SoundRequest.msg"
  DEPENDENCIES std_msgs
)

# 4) Instalación correcta: .so a lib/, ejecutable a lib/${PROJECT_NAME}
install(TARGETS
  robo_eyes_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib          # (por si generas .a)
  LIBRARY DESTINATION lib          # <-- la .so va aquí
  RUNTIME DESTINATION bin
)

install(TARGETS
  robo_eyes_sim
  DESTINATION lib/${PROJECT_NAME}   # <-- el binario va aquí
)

install(DIRECTORY include/
  DESTINATION include)

# 5) Export para que otros paquetes puedan linkar correctamente (y buen RPATH)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp OpenCV)
ament_package()