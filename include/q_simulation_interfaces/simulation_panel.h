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
 
#pragma once

#include <QVBoxLayout>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>

namespace q_simulation_interfaces
{
    class SimulationWidget;
    class SimulationPanel : public rviz_common::Panel
    {
        Q_OBJECT
    public:
        explicit SimulationPanel(QWidget* parent = nullptr);

        ~SimulationPanel() override;
        void onInitialize() override;
        QString getName() const override;
        void hideEvent(QHideEvent* event) override;

    private:
        SimulationWidget* simulationWidget_;
        rviz_common::Display* im_display_{nullptr};
        std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
    };
} // namespace q_simulation_interfaces
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(q_simulation_interfaces::SimulationPanel, rviz_common::Panel)
