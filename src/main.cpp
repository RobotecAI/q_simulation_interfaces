/* Copyright 2025, Robotec.ai sp. z o.o.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 
#include <QApplication>
#include <QStyleFactory>
#include <rclcpp/rclcpp.hpp>
#include "my_widget.hpp"
#include <QFile>
#include <QTextStream>


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    MyWidget widget;
    widget.show();

    int ret = app.exec();
    rclcpp::shutdown();
    return ret;
}

