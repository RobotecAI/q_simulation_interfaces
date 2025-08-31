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

#include <QScrollArea>
#include <QVBoxLayout>
#include <q_simulation_interfaces/simulation_panel.h>
#include <rviz_common/display_group.hpp>
#include <rviz_common/panel.hpp>
#include "simulation_widget.h"


namespace q_simulation_interfaces
{
    namespace
    {
        constexpr char InteractiveMarkerClassId[] = "rviz_default_plugins/InteractiveMarkers";
        constexpr char InteractiveMarkerNamespacePropertyName[] = "Interactive Markers Namespace";
    } // namespace
    SimulationPanel::SimulationPanel(QWidget* parent)
    {
        simulationWidget_ = new SimulationWidget(this);

        auto* scrollArea = new QScrollArea(this);
        scrollArea->setWidget(simulationWidget_);
        scrollArea->setWidgetResizable(true);
        scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);

        QVBoxLayout* layout = new QVBoxLayout(this);
        layout->addWidget(scrollArea);
        setLayout(layout);
    }

    SimulationPanel::~SimulationPanel() { delete simulationWidget_; }

    //! Recursively iterate through all displays in a display group and apply a function to each display.
    void iterateAllDisplays(rviz_common::DisplayGroup* group, std::function<bool(rviz_common::Display*)> func)
    {
        for (int i = 0; i < group->numChildren(); ++i)
        {
            auto display = group->getDisplayAt(i);
            if (display)
            {
                if (func(display))
                {
                    return; // stop iterating if the function returns true
                }
            }
        }
        for (int i = 0; i < group->numChildren(); ++i)
        {
            auto sub_group = group->getGroupAt(i);
            if (sub_group)
            {
                iterateAllDisplays(sub_group, func);
            }
        }
    }

    void SimulationPanel::onInitialize()
    {
        node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();
        if (!node_ptr_)
        {
            throw std::runtime_error("Failed to get ROS node abstraction");
        }
        auto context = this->getDisplayContext();
        simulationWidget_->SetFixedFrame(context->getFixedFrame());
        simulationWidget_->initialize(node_ptr_->get_raw_node());

        auto display_group = context->getRootDisplayGroup();
        assert(display_group);

        // check if the InteractiveMarkers display already exists

        const auto isInteractiveMarkerDisplay = [this](rviz_common::Display* display)
        {
            if (display->getClassId() == InteractiveMarkerClassId)
            {
                auto property = display->findProperty(InteractiveMarkerNamespacePropertyName);
                if (property &&
                    property->getValue().toString() == QString::fromStdString(InteractiveMarkerNamespaceValue))
                {
                    std::cout << "Found InteractiveMarkers display with namespace: "
                              << property->getValue().toString().toStdString() << std::endl;
                    im_display_ = display; // Store the found display
                    return true; // Found the InteractiveMarkers display with the correct namespace
                }
            }
            return false;
        };

        iterateAllDisplays(display_group, isInteractiveMarkerDisplay);

        if (!im_display_)
        {
            im_display_ = display_group->createDisplay(InteractiveMarkerClassId);
            assert(im_display_);
            std::cout << "InteractiveMarkers display created" << std::endl;

            im_display_->initialize(context);

            im_display_->setEnabled(true);
            im_display_->setShouldBeSaved(false);
            im_display_->setName("Simulation Interactive Markers");
            auto property = im_display_->findProperty(InteractiveMarkerNamespacePropertyName);
            assert(property);
            property->setValue(QString::fromStdString(InteractiveMarkerNamespaceValue));
            display_group->addDisplay(im_display_);
        }
    };

    void SimulationPanel::hideEvent(QHideEvent* event)
    {
        if (im_display_)
        {
            im_display_ = nullptr;
        }
    }


    QString SimulationPanel::getName() const { return "Simulation Panel"; }

    void SimulationPanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);
        if (simulationWidget_)
        {
            const_cast<SimulationWidget*>(simulationWidget_)->getServiceDiscovery().saveConfig(config);
        }
    }

    void SimulationPanel::load(const rviz_common::Config& config)
    {
        rviz_common::Panel::load(config);
        if (simulationWidget_)
        {
            simulationWidget_->getServiceDiscovery().loadConfig(config);
        }
    }

} // namespace q_simulation_interfaces
