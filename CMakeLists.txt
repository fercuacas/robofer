cmake_minimum_required(VERSION 3.8)
project(robofer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Dependencias ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

# ---- Mensajes (si no los usas en los nodos, igualmente exportamos runtime) ----
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EyesCommand.msg"
  "msg/ServoGoal.msg"
  "msg/SoundRequest.msg"
  "msg/WifiStatus.msg"
  "msg/WifiNetwork.msg"
  "srv/WifiGetStatus.srv"
  "srv/WifiSetCredentials.srv"
  "srv/WifiScan.srv"
  "srv/SetBoolWithCode.srv"
  DEPENDENCIES std_msgs
)

# =========================================
# Bluetoothctl agent library
# =========================================
add_library(robofer_bt_agent
  src/bluetoothctl_agent.cpp
)
target_include_directories(robofer_bt_agent PUBLIC include)
target_link_libraries(robofer_bt_agent PUBLIC pthread)

# =========================================
# 1) Librería de lógica de ojos
# =========================================
add_library(robo_eyes_lib SHARED
  src/screen/Eyes.cpp
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
  src/screen/Display.cpp
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
  src/actuators/ControlServo.cpp
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
# Audio library
# =========================================
add_library(robo_audio
  src/audio/AudioPlayer.cpp
)
target_include_directories(robo_audio PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# =========================================
# 3) Ejecutables
# =========================================

# 3.1) Simulador OpenCV clásico (si quieres mantenerlo)
add_executable(robo_eyes_sim
  src/screen/EyesNode.cpp
)
ament_target_dependencies(robo_eyes_sim rclcpp)
target_link_libraries(robo_eyes_sim
  robo_eyes_lib
  ${OpenCV_LIBS}
)

# 3.2) Nodo ST7735 clásico (si quieres mantenerlo)
#     OJO: NO añadimos src/screen/Eyes.cpp aquí (ya está en la librería)
add_executable(robo_eyes_st7735_node
  src/screen/EyesSt7735Node.cpp
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
  src/screen/EyesUnifiedNode.cpp
  src/screen/UiMenu.cpp
)
ament_target_dependencies(eyes_unified_node rclcpp std_msgs std_srvs)
target_link_libraries(eyes_unified_node
  robo_eyes_lib
  robo_display
  robofer_bt_agent
  ${OpenCV_LIBS}
)
rosidl_target_interfaces(eyes_unified_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# --- Botones físicos ---
add_executable(buttons_node
  src/input/ButtonsNode.cpp
)
ament_target_dependencies(buttons_node rclcpp std_msgs)
target_link_libraries(buttons_node gpiod)

# --- Botones por teclado (simulación) ---
add_executable(keyboard_buttons_node
  src/input/KeyboardButtonsNode.cpp
)
ament_target_dependencies(keyboard_buttons_node rclcpp std_msgs)

# --- State handler (modes + servos) ---
add_executable(state_handler_node
  src/control/StateHandler.cpp
)
ament_target_dependencies(state_handler_node rclcpp std_msgs)
target_link_libraries(state_handler_node
  robo_servos
  robo_audio
)

add_executable(servo_monitor_node
  src/actuators/ServoMonitorNode.cpp
)
ament_target_dependencies(servo_monitor_node rclcpp)
rosidl_target_interfaces(servo_monitor_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(wifi_manager_node
  src/network/WifiManagerNode.cpp
)
ament_target_dependencies(wifi_manager_node rclcpp)
rosidl_target_interfaces(wifi_manager_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(bluetooth_manager_node
  src/network/BluetoothManagerNode.cpp
)
ament_target_dependencies(bluetooth_manager_node rclcpp std_msgs std_srvs)
rosidl_target_interfaces(bluetooth_manager_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(bt_provision_node
  src/network/BtProvisionNode.cpp
)
ament_target_dependencies(bt_provision_node rclcpp  std_msgs std_srvs)
target_link_libraries(bt_provision_node bluetooth)
rosidl_target_interfaces(bt_provision_node
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
  robo_audio
  robofer_bt_agent
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
  wifi_manager_node
  bluetooth_manager_node
  bt_provision_node
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
ament_export_libraries(robo_eyes_lib robo_display robo_servos robo_audio)
ament_export_dependencies(rclcpp OpenCV rosidl_default_runtime)

ament_package()
