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

#include <QCheckBox>
#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <array>
#include <qpushbutton.h>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/config.hpp>
#include <set>

namespace q_simulation_interfaces
{

    enum ServiceType : unsigned int
    {
        SERVICE_GET_SPAWNABLES,
        SERVICE_SPAWN_ENTITY,
        SERVICE_GET_ENTITIES,
        SERVICE_GET_ENTITY_STATE,
        SERVICE_SET_ENTITY_STATE,
        SERVICE_DELETE_ENTITY,
        SERVICE_GET_SIM_FEATURES,
        SERVICE_RESET_SIMULATION,
        SERVICE_STEP_SIMULATION,
        SERVICE_GET_SIM_STATE,
        SERVICE_SET_SIM_STATE,
        SERVICE_GET_CURRENT_WORLD,
        SERVICE_GET_AVAILABLE_WORLDS,
        SERVICE_LOAD_WORLD,
        SERVICE_UNLOAD_WORLD,
        ACTION_SIMULATE_STEPS,
        SUPPORTED_SERVICE_IDL_COUNT
    };

    struct ServiceInfo
    {
        ServiceType type;
        const char* type_string;
        const char* friendly_name;
        bool is_action;
    };

    constexpr std::array<ServiceInfo, static_cast<unsigned int>(ServiceType::SUPPORTED_SERVICE_IDL_COUNT)>
        SUPPORTED_SERVICE_IDL_TYPES = {
            {{ServiceType::SERVICE_GET_SPAWNABLES, "simulation_interfaces/srv/GetSpawnables", "Get Spawnables", false},
             {ServiceType::SERVICE_SPAWN_ENTITY, "simulation_interfaces/srv/SpawnEntity", "Spawn Entity", false},
             {ServiceType::SERVICE_GET_ENTITIES, "simulation_interfaces/srv/GetEntities", "Get Entities", false},
             {ServiceType::SERVICE_GET_ENTITY_STATE, "simulation_interfaces/srv/GetEntityState", "Get Entity State",
              false},
             {ServiceType::SERVICE_SET_ENTITY_STATE, "simulation_interfaces/srv/SetEntityState", "Set Entity State",
              false},
             {ServiceType::SERVICE_DELETE_ENTITY, "simulation_interfaces/srv/DeleteEntity", "Delete Entity", false},
             {ServiceType::SERVICE_GET_SIM_FEATURES, "simulation_interfaces/srv/GetSimulatorFeatures",
              "Get Sim Features", false},
             {ServiceType::SERVICE_RESET_SIMULATION, "simulation_interfaces/srv/ResetSimulation", "Reset Simulation",
              false},
             {ServiceType::SERVICE_STEP_SIMULATION, "simulation_interfaces/srv/StepSimulation", "Step Simulation",
              false},
             {ServiceType::SERVICE_GET_SIM_STATE, "simulation_interfaces/srv/GetSimulationState", "Get Sim State",
              false},
             {ServiceType::SERVICE_SET_SIM_STATE, "simulation_interfaces/srv/SetSimulationState", "Set Sim State",
              false},
             {ServiceType::SERVICE_GET_CURRENT_WORLD, "simulation_interfaces/srv/GetCurrentWorld", "Get Current World",
              false},
             {ServiceType::SERVICE_GET_AVAILABLE_WORLDS, "simulation_interfaces/srv/GetAvailableWorlds",
              "Get Available Worlds", false},
             {ServiceType::SERVICE_LOAD_WORLD, "simulation_interfaces/srv/LoadWorld", "Load World", false},
             {ServiceType::SERVICE_UNLOAD_WORLD, "simulation_interfaces/srv/UnloadWorld", "Unload World", false},
             {ServiceType::ACTION_SIMULATE_STEPS, "simulation_interfaces/action/SimulateSteps", "Simulate Steps",
              true}}};

    class ServiceDiscovery : public QObject
    {
        Q_OBJECT

    public:
        ServiceDiscovery();
        ~ServiceDiscovery();

        // Initializes the service tab UI
        void initializeServiceUI(QWidget* parent);

        void initializeServices(rclcpp::Node::SharedPtr node);

        void saveConfig(rviz_common::Config config) const;
        void loadConfig(const rviz_common::Config& config);

    signals:
        void serviceComboBoxChanged(ServiceType idlType, const QString& selectedService);

    private slots:
        void onAutoSelectToggled(bool enabled);

    private:
        void createServiceComboBox(const ServiceInfo& idlType, QVBoxLayout* layout);

        void onServiceComboBoxChanged(const QString& selectedService);

        void discoverServices();
        void startAutoService();
        void stopAutoService();

        void autoSelectServices();

        rclcpp::Node::SharedPtr node_;
        QWidget* parent_;

        // Buttons for controlling the service discovery
        QPushButton* discoverButton_;
        QCheckBox* autoDiscoveryCheckBox_;

        // Automatic service discovery
        QTimer* autoDiscoveryTimer_;

        // Comboboxes for each service type
        std::array<QComboBox*, SUPPORTED_SERVICE_IDL_COUNT> serviceComboBoxes_;
        std::array<QLabel*, SUPPORTED_SERVICE_IDL_COUNT> serviceLabels_;
    };
} // namespace q_simulation_interfaces
