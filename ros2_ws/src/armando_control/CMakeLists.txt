cmake_minimum_required(VERSION 3.5)
project(armando_control)

# Utiliser C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Trouver les packages ROS nécessaires
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Inclure le dossier include pour les headers
include_directories(include)

# Ajouter l’exécutable
add_executable(joint_position_publisher
  src/joint_position_publisher.cpp
)

# Lier les bibliothèques ROS nécessaires
ament_target_dependencies(joint_position_publisher
  rclcpp
  sensor_msgs
)

# Installer l’exécutable
install(TARGETS joint_position_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Déclarer le package comme ament
ament_package()

