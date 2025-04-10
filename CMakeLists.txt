cmake_minimum_required(VERSION 3.5)
project(q_simulation_interfaces)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
SET(CMAKE_AUTOUIC ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(simulation_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)

# Add UI file

set(SOURCES src/main.cpp
            src/my_widget.cpp)
add_executable(${PROJECT_NAME} ${SOURCES}
)
qt_add_resources(${PROJECT_NAME} "resources/lightstyle.qrc")
ament_target_dependencies(${PROJECT_NAME}
  rclcpp rclcpp_action simulation_interfaces tf2
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
)


install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

