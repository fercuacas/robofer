cmake_minimum_required(VERSION 3.8)
project(robofer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Dependencias ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)

# ---- Mensajes (si no los usas en los nodos, igualmente exportamos runtime) ----
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EyesCommand.msg"
  "msg/ServoGoal.msg"
  "msg/SoundRequest.msg"
  DEPENDENCIES std_msgs
)

# =========================================
# 1) Librería de lógica de ojos
# =========================================
add_library(robo_eyes_lib SHARED
  src/eyes.cpp
)
# Headers públicos para quien linke esta lib
target_include_directories(robo_eyes_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# ROS y demás (usar ament_target_dependencies para ROS)
ament_target_dependencies(robo_eyes_lib rclcpp)
# OpenCV va por linkeo plano (firma sin PUBLIC/PRIVATE)
target_link_libraries(robo_eyes_lib
  ${OpenCV_LIBS}
)

# =========================================
# 2) Librería de displays unificada (SIM + ST7735)
# =========================================
add_library(robo_display
  src/display.cpp
)
target_include_directories(robo_display PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(robo_display rclcpp)
# OpenCV y gpiod en firma "plana"
target_link_libraries(robo_display
  ${OpenCV_LIBS}
  gpiod
)

# =========================================
# Servos library
# =========================================
add_library(robo_servos
  src/servos.cpp
)
target_include_directories(robo_servos PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(robo_servos rclcpp)
target_link_libraries(robo_servos gpiod)
rosidl_target_interfaces(robo_servos
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# =========================================
# 3) Ejecutables
# =========================================

# 3.1) Simulador OpenCV clásico (si quieres mantenerlo)
add_executable(robo_eyes_sim
  src/eyes_node.cpp
)
ament_target_dependencies(robo_eyes_sim rclcpp)
target_link_libraries(robo_eyes_sim
  robo_eyes_lib
  ${OpenCV_LIBS}
)

# 3.2) Nodo ST7735 clásico (si quieres mantenerlo)
#     OJO: NO añadimos src/eyes.cpp aquí (ya está en la librería)
add_executable(robo_eyes_st7735_node
  src/eyes_st7735_node.cpp
)
ament_target_dependencies(robo_eyes_st7735_node rclcpp)
target_include_directories(robo_eyes_st7735_node PRIVATE include)
target_link_libraries(robo_eyes_st7735_node
  robo_eyes_lib
  ${OpenCV_LIBS}
  gpiod
)

# 3.3) Nodo unificado (elige backend por parámetro: sim | st7735)
add_executable(eyes_unified_node
  src/eyes_unified_node.cpp
  src/ui_menu.cpp
)
ament_target_dependencies(eyes_unified_node rclcpp std_msgs)
target_link_libraries(eyes_unified_node
  robo_eyes_lib
  robo_display
  ${OpenCV_LIBS}
)

# --- Botones físicos ---
add_executable(buttons_node
  src/buttons.cpp
)
ament_target_dependencies(buttons_node rclcpp std_msgs)
target_link_libraries(buttons_node gpiod)

# --- Botones por teclado (simulación) ---
add_executable(keyboard_buttons_node
  src/keyboard_buttons.cpp
)
ament_target_dependencies(keyboard_buttons_node rclcpp std_msgs)

# --- State handler (modes + servos) ---
add_executable(state_handler_node
  src/state_handler_node.cpp
)
ament_target_dependencies(state_handler_node rclcpp std_msgs)
target_link_libraries(state_handler_node
  robo_servos
)

add_executable(servo_monitor_node
  src/servo_monitor_node.cpp
)
ament_target_dependencies(servo_monitor_node rclcpp)
rosidl_target_interfaces(servo_monitor_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# =========================================
# 4) Instalación
#    - Librerías a lib/
#    - Ejecutables a lib/${PROJECT_NAME}/
#    - Headers a include/
# =========================================

# Librerías
install(TARGETS
  robo_eyes_lib
  robo_display
  robo_servos
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Ejecutables
install(TARGETS
  robo_eyes_sim
  robo_eyes_st7735_node
  eyes_unified_node
  buttons_node
  keyboard_buttons_node
  state_handler_node
  servo_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Headers
install(DIRECTORY include/
  DESTINATION include
)

# Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# =========================================
# 5) Export para consumidores
# =========================================
ament_export_include_directories(include)
ament_export_libraries(robo_eyes_lib robo_display robo_servos)
ament_export_dependencies(rclcpp OpenCV rosidl_default_runtime)

ament_package()
