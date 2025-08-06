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


#include <simulation_interfaces/srv/spawn_entity.hpp>
#include <simulation_interfaces/srv/get_entities.hpp>
#include <simulation_interfaces/srv/get_entity_state.hpp>
#include <simulation_interfaces/srv/set_entity_state.hpp>
#include <simulation_interfaces/srv/delete_entity.hpp>
#include <simulation_interfaces/srv/reset_simulation.hpp>
#include <simulation_interfaces/srv/get_simulation_state.hpp>
#include <simulation_interfaces/srv/set_simulation_state.hpp>
#include <simulation_interfaces/srv/step_simulation.hpp>

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


        void initialize(rclcpp::Node::SharedPtr node = nullptr);

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

        //! The thread with own ROS 2 node that will run the action client
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

        // Vector to hold all service interfaces of created services
        std::vector<std::shared_ptr<ServiceInterface>> serviceInterfaces_;
        QTimer* timer_; //! Timer for periodic updates

        std::shared_ptr<interactive_markers::InteractiveMarkerServer> interactiveMarkerServer_;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    };
}
