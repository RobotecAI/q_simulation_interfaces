#pragma once

#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <thread>

namespace Ui {
class simWidgetUi;
}

namespace q_simulation_interfaces
{

class SimulationPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit SimulationPanel(QWidget *parent = nullptr);
  ~SimulationPanel() override;

  void onInitialize() override;
  
  // Method for standalone mode to set ROS node directly
  void setRosNode(rclcpp::Node::SharedPtr node);

private:
  void GetSpawnables();
  void SpawnButton();
  void GetAllEntities();
  void GetEntityState(bool silent = false);
  void SetEntityState();
  void DespawnButton();
  void GetSimFeatures();
  void ResetSimulation();
  void StepSimulation();
  void GetSimulationState();
  void SetSimulationState();
  void StepSimulationService();

  void ActionThreadWorker(int steps);
  std::thread actionThread_;

  Ui::simWidgetUi *ui_;
  rclcpp::Node::SharedPtr node_;
};

}  // namespace q_simulation_interfaces