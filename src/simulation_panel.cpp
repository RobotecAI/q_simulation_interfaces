#include <q_simulation_interfaces/simulation_panel.hpp>
#include <QVBoxLayout>
#include "simulation_widget.hpp"
#include <rviz_common/panel.hpp>
#include <rviz_common/display_group.hpp>
namespace q_simulation_interfaces
{

    SimulationPanel::SimulationPanel(QWidget* parent)
    {
        simulationWidget_ = new SimulationWidget(this);
        QVBoxLayout* layout = new QVBoxLayout(this);
        layout->addWidget(simulationWidget_);
        setLayout(layout);
    }

    SimulationPanel::~SimulationPanel() { delete simulationWidget_; }

    void SimulationPanel::onInitialize()
    {
        if (!im_display_)
        {
            auto context = this->getDisplayContext();
            auto display_group = context->getRootDisplayGroup();
            im_display_ = display_group->createDisplay("rviz_default_plugins/InteractiveMarkers");
            if (!im_display_)
            {
                throw std::runtime_error("Failed to create InteractiveMarkers display");
            }
        }
        simulationWidget_->intiliaze();
        // add marker to the interactive marker server
    };

    QString SimulationPanel::getName() const { return "Simulation Panel"; }

} // namespace q_simulation_interfaces
