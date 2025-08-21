cmake_minimum_required(VERSION 3.5)
project(q_simulation_interfaces)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
SET(CMAKE_AUTOUIC ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(simulation_interfaces 1 REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)

# Add UI file

qt5_wrap_cpp(MOC_FILES
        include/q_simulation_interfaces/simulation_panel.h
)

set(SOURCES src/simulation_widget.cpp src/simulation_panel.cpp)

add_library(${PROJECT_NAME} SHARED ${SOURCES} ${MOC_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    rclcpp_action
    simulation_interfaces
    pluginlib rviz_common  interactive_markers visualization_msgs
    tf2 tf2_ros tf2_geometry_msgs
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
)

# Create standalone executable
add_executable(${PROJECT_NAME}_standalone src/standalone_main.cpp src/simulation_widget.cpp ${MOC_FILES})
target_link_libraries(${PROJECT_NAME}_standalone ${PROJECT_NAME} Qt5::Widgets)
ament_target_dependencies(${PROJECT_NAME}_standalone rclcpp)

install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(TARGETS ${PROJECT_NAME}_standalone
        DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
        DESTINATION include
)

install(DIRECTORY icons/
        DESTINATION share/${PROJECT_NAME}/icons/classes
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
pluginlib_export_plugin_description_file(rviz_common q_simulation_plugins.xml)

ament_package()

