#pragma once

#include "service.h"
#include <QVBoxLayout>
#include <QWidget>
#include <interactive_markers/interactive_marker_server.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <thread>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// Forward declarations for service types
template <typename T>
class Service;
namespace simulation_interfaces
{
    namespace srv
    {
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
    } // namespace srv
} // namespace simulation_interfaces

namespace Ui
{
    class simWidgetUi;
}
namespace q_simulation_interfaces
{
    const char InteractiveMarkerNamespaceValue[] = "simulation_interfaces_panel";
    class SimulationWidget : public QWidget
    {
        Q_OBJECT

    public:
        explicit SimulationWidget(QWidget* parent = nullptr);
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

        //! Called periodically to update the state of the services
        void UpdateServices();

        //! Create and update spawn point interactive marker
        void CreateSpawnPointMarker();
        void UpdateSpawnPointMarker();

        std::thread actionThread_;
        std::atomic<bool> actionThreadRunning_{false}; //! Flag to control the action thread
        std::atomic<float> actionThreadProgress_{0.0f}; //! Progress of the simulation step, used for UI updates

        Ui::simWidgetUi* ui_;
        rclcpp::Node::SharedPtr node_;

        // Service member variables
        std::shared_ptr<Service<simulation_interfaces::srv::GetSpawnables>> getSpawnablesService_;
        std::shared_ptr<Service<simulation_interfaces::srv::SpawnEntity>> spawnEntityService_;
        std::shared_ptr<Service<simulation_interfaces::srv::GetEntities>> getEntitiesService_;
        std::shared_ptr<Service<simulation_interfaces::srv::GetEntityState>> getEntityStateService_;
        std::shared_ptr<Service<simulation_interfaces::srv::SetEntityState>> setEntityStateService_;
        std::shared_ptr<Service<simulation_interfaces::srv::DeleteEntity>> deleteEntityService_;
        std::shared_ptr<Service<simulation_interfaces::srv::GetSimulatorFeatures>> getSimFeaturesService_;
        std::shared_ptr<Service<simulation_interfaces::srv::ResetSimulation>> resetSimulationService_;
        std::shared_ptr<Service<simulation_interfaces::srv::GetSimulationState>> getSimulationStateService_;
        std::shared_ptr<Service<simulation_interfaces::srv::SetSimulationState>> setSimulationStateService_;
        std::shared_ptr<Service<simulation_interfaces::srv::StepSimulation>> stepSimulationService_;

        // Vector to hold all service interfaces - this allows for easy management and iteration
        std::vector<std::shared_ptr<ServiceInterface>> serviceInterfaces_;
        QTimer* timer_; //! Timer for periodic updates

        std::shared_ptr<interactive_markers::InteractiveMarkerServer> interactiveMarkerServer_;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    };
}
