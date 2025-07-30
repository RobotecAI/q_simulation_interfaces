#pragma once

#include <QWidget>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <thread>
#include <memory>
#include <interactive_markers/interactive_marker_server.hpp>


// Forward declarations for service types
template<typename T> class Service;
namespace simulation_interfaces {
  namespace srv {
    class GetSpawnables;
    class SpawnEntity;
    class GetEntities;
    class GetEntityState;
    class SetEntityState;
    class DeleteEntity;
    class GetSimulatorFeatures;
    class ResetSimulation;
    class GetSimulationState;
    class SetSimulationState;
    class StepSimulation;
  }
}

namespace Ui {
class simWidgetUi;
}
namespace q_simulation_interfaces
{

class SimulationWidget : public QWidget
{
  Q_OBJECT

public:
  explicit SimulationWidget(QWidget *parent = nullptr);
  ~SimulationWidget() override;


  void intiliaze(rclcpp::Node::SharedPtr node = nullptr);

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
  
  // Service member variables
  std::unique_ptr<Service<simulation_interfaces::srv::GetSpawnables>> getSpawnablesService_;
  std::unique_ptr<Service<simulation_interfaces::srv::SpawnEntity>> spawnEntityService_;
  std::unique_ptr<Service<simulation_interfaces::srv::GetEntities>> getEntitiesService_;
  std::unique_ptr<Service<simulation_interfaces::srv::GetEntityState>> getEntityStateService_;
  std::unique_ptr<Service<simulation_interfaces::srv::SetEntityState>> setEntityStateService_;
  std::unique_ptr<Service<simulation_interfaces::srv::DeleteEntity>> deleteEntityService_;
  std::unique_ptr<Service<simulation_interfaces::srv::GetSimulatorFeatures>> getSimFeaturesService_;
  std::unique_ptr<Service<simulation_interfaces::srv::ResetSimulation>> resetSimulationService_;
  std::unique_ptr<Service<simulation_interfaces::srv::GetSimulationState>> getSimulationStateService_;
  std::unique_ptr<Service<simulation_interfaces::srv::SetSimulationState>> setSimulationStateService_;
  std::unique_ptr<Service<simulation_interfaces::srv::StepSimulation>> stepSimulationService_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> interactiveMarkerServer_;
};

class SimulationPanel : public rviz_common::Panel {
public:
  explicit SimulationPanel(QWidget *parent = nullptr) {
    simulationWidget_ = new SimulationWidget(this);
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addWidget(simulationWidget_);
    setLayout(layout);
  }
  ~SimulationPanel() override{
    delete simulationWidget_;
  }
  void onInitialize() override {
    simulationWidget_->intiliaze();
  };

  QString getName() const override { return "Simulation Panel"; }
private:
  SimulationWidget *simulationWidget_;
};
}  // namespace q_simulation_interfaces