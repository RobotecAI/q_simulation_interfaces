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
#include <QFile>
#include <rclcpp/rclcpp.hpp>
#include "simulation_widget.h"

class StandaloneWidget : public QWidget
{
public:
    StandaloneWidget(QWidget* parent = nullptr) : QWidget(parent)
    {
        // Initialize ROS node
        node_ = rclcpp::Node::make_shared("standalone_simulation_widget");
        setWindowTitle("Simulation Interface");
        setMinimumSize(800, 600);

        auto layout = new QVBoxLayout(this);

        // Create a mock display context for the panel
        panel_ = new q_simulation_interfaces::SimulationWidget(this);
        panel_->initialize(node_);
        layout->addWidget(panel_);
        node_thread_ = std::thread([this]() { rclcpp::spin(node_); });
    }

    ~StandaloneWidget()
    {
        if (node_thread_.joinable())
        {
            node_thread_.join();
        }
        rclcpp::shutdown();
    }

private:
    q_simulation_interfaces::SimulationWidget* panel_;
    rclcpp::Node::SharedPtr node_;
    std::thread node_thread_;
};

int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    StandaloneWidget widget;
    widget.show();

    int ret = app.exec();
    rclcpp::shutdown();
    return ret;
}
