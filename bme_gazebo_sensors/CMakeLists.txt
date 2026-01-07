cmake_minimum_required(VERSION 3.8)
project(bme_gazebo_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED) 
# CORRETTO: Il nome del pacchetto è quello definito nel package.xml dell'altro pacchetto
find_package(assignment2_msgs REQUIRED) 

# Executable 1: Movement
add_executable(movement src/movement.cpp)
ament_target_dependencies(movement rclcpp geometry_msgs std_msgs nav_msgs)

# Executable 2: Distance
add_executable(distance src/distance.cpp)
ament_target_dependencies(distance rclcpp sensor_msgs nav_msgs)

# Executable 3: Service Node
# CORRETTO: Sintassi add_executable(nome_eseguibile sorgente)
# Assumo che il file cpp sia dentro la cartella src/ e si chiami Assignment2_service.cpp
add_executable(assignment_service_node src/Assignment2_service.cpp)
ament_target_dependencies(assignment_service_node rclcpp assignment2_msgs)

install(TARGETS 
  movement
  distance 
  assignment_service_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  worlds
  rviz
  urdf
  meshes
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
