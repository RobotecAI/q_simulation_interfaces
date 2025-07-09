#pragma once


#include <QVBoxLayout>
#include <rviz_common/panel.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>

namespace q_simulation_interfaces
{
    class SimulationWidget;
    class SimulationPanel : public rviz_common::Panel
    {
    public:
        explicit SimulationPanel(QWidget* parent = nullptr);

        ~SimulationPanel() override;
        void onInitialize() override;
        QString getName() const;
        void hideEvent(QHideEvent* event) override;

    private:
        SimulationWidget* simulationWidget_;
        rviz_common::Display *im_display_ {nullptr};
        std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
    };
} // namespace q_simulation_interfaces
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(q_simulation_interfaces::SimulationPanel, rviz_common::Panel)
