# Copyright 2025, Robotec.ai sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

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

