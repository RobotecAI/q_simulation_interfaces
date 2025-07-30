#include "q_simulation_interfaces/simulation_panel.hpp"
#include <QDebug>
#include <QMessageBox>
#include <rclcpp_action/create_client.hpp>
#include <rviz_common/display_context.hpp>
#include <simulation_interfaces/action/simulate_steps.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "service.h"
#include "stringToKeys.h"
#include "ui_sim_widget.h"
#include "vector_utils.hpp"

#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>

namespace q_simulation_interfaces
{

    SimulationWidget::SimulationWidget(QWidget* parent) : QWidget(parent), ui_(new Ui::simWidgetUi)
    {
        ui_->setupUi(this);

        for (const auto& [name, _] : ScopeNameToId)
        {
            ui_->resetModeCombo->addItem(QString::fromStdString(name));
        }
        for (const auto& [name, _] : SimStateNameToId)
        {
            ui_->simStateToSetComboBox->addItem(QString::fromStdString(name));
        }

        connect(ui_->PushButtonRefresh, &QPushButton::clicked, this, &SimulationWidget::GetSpawnables);
        connect(ui_->SpawnButton, &QPushButton::clicked, this, &SimulationWidget::SpawnButton);
        connect(ui_->getAllEntitiesButton, &QPushButton::clicked, this, &SimulationWidget::GetAllEntities);
        connect(ui_->getEntityStateButton, &QPushButton::clicked, this, &SimulationWidget::GetEntityState);
        connect(ui_->setEntityStateButton, &QPushButton::clicked, this, &SimulationWidget::SetEntityState);
        connect(ui_->despawnButton, &QPushButton::clicked, this, &SimulationWidget::DespawnButton);
        connect(ui_->GetSimCapabilites, &QPushButton::clicked, this, &SimulationWidget::GetSimFeatures);
        connect(ui_->resetSimButton, &QPushButton::clicked, this, &SimulationWidget::ResetSimulation);
        connect(ui_->stepSimButtonAction, &QPushButton::clicked, this, &SimulationWidget::StepSimulation);
        connect(ui_->getSimStateBtn, &QPushButton::clicked, this, &SimulationWidget::GetSimulationState);
        connect(ui_->setSimStateButton, &QPushButton::clicked, this, &SimulationWidget::SetSimulationState);
        connect(ui_->stepSimServiceButton, &QPushButton::clicked, this, &SimulationWidget::StepSimulationService);
        connect(ui_->ComboEntities, &QComboBox::currentTextChanged, this, [this]() { this->GetEntityState(true); });
    }

    SimulationWidget::~SimulationWidget()
    {
        if (actionThread_.joinable())
        {
            actionThread_.join();
        }
        delete ui_;
    }

    void SimulationWidget::intiliaze(rclcpp::Node::SharedPtr node) {
        if (!node)
        {
            node_ = rclcpp::Node::make_shared("qt_gui_node");
        }
        else
        {
            node_ = node;
        }

        // Initialize service objects
        getSpawnablesService_ = std::make_unique<Service<simulation_interfaces::srv::GetSpawnables>>("/get_spawnables", node_);
        spawnEntityService_ = std::make_unique<Service<simulation_interfaces::srv::SpawnEntity>>("/spawn_entity", node_);
        getEntitiesService_ = std::make_unique<Service<simulation_interfaces::srv::GetEntities>>("/get_entities", node_);
        getEntityStateService_ = std::make_unique<Service<simulation_interfaces::srv::GetEntityState>>("/get_entity_state", node_);
        setEntityStateService_ = std::make_unique<Service<simulation_interfaces::srv::SetEntityState>>("/set_entity_state", node_);
        deleteEntityService_ = std::make_unique<Service<simulation_interfaces::srv::DeleteEntity>>("/delete_entity", node_);
        getSimFeaturesService_ = std::make_unique<Service<simulation_interfaces::srv::GetSimulatorFeatures>>("/get_simulation_features", node_);
        resetSimulationService_ = std::make_unique<Service<simulation_interfaces::srv::ResetSimulation>>("/reset_simulation", node_);
        getSimulationStateService_ = std::make_unique<Service<simulation_interfaces::srv::GetSimulationState>>("/get_simulation_state", node_);
        setSimulationStateService_ = std::make_unique<Service<simulation_interfaces::srv::SetSimulationState>>("/set_simulation_state", node_);
        stepSimulationService_ = std::make_unique<Service<simulation_interfaces::srv::StepSimulation>>("/step_simulation", node_);
        interactiveMarkerServer_ = std::make_unique<interactive_markers::InteractiveMarkerServer>("/simulation_interfaces_panel", node_);

    }

     void SimulationWidget::GetSimulationState()
    {
        auto response = getSimulationStateService_->call_service_sync();
        if (!response)
        {
            QMessageBox::warning(this, "Error",
                                 "Failed to get simulation features : " + QString::fromStdString(response.error()));
            return;
        }

        int stateId = response->state.state;
        QString stateName;
        auto it = SimStateIdToName.find(stateId);
        if (it == SimStateIdToName.end())
        {
            stateName = QString::asprintf("Unknow state %d", stateId);
        }
        else
        {
            stateName = QString::fromStdString(it->second);
        }
        ui_->simStateLabel->setText(stateName);
    }

    void SimulationWidget::SetSimulationState()
    {
        simulation_interfaces::srv::SetSimulationState::Request request;
        auto selectedMode = ui_->simStateToSetComboBox->currentText();
        auto it = SimStateNameToId.find(selectedMode.toStdString());
        Q_ASSERT(it != SimStateNameToId.end());
        request.state.state = it->second;
        auto response = setSimulationStateService_->call_service_sync(request);
        if (!response)
        {
            QMessageBox::warning(this, "Error",
                                 "Failed to set simulation state : " + QString::fromStdString(response.error()));
            return;
        }
        if (response)
        {
            GetSimulationState();
        }
    }

    void SimulationWidget::ActionThreadWorker(int steps)
    {
        // create node
        auto node = rclcpp::Node::make_shared("qt_gui_action_node");
        using SimulateSteps = simulation_interfaces::action::SimulateSteps;
        auto client = rclcpp_action::create_client<SimulateSteps>(node, "/simulate_steps");

        auto send_goal_options = rclcpp_action::Client<SimulateSteps>::SendGoalOptions();
        auto goal = std::make_shared<SimulateSteps::Goal>();
        goal->steps = steps;
        send_goal_options.feedback_callback =
            [this](rclcpp_action::ClientGoalHandle<SimulateSteps>::SharedPtr goal_handle,
                   const std::shared_ptr<const SimulateSteps::Feedback> feedback)
        {
            float progress = static_cast<float>(feedback->completed_steps) / feedback->remaining_steps;
            ui_->simProgressBar->setValue(static_cast<int>(progress * 100));
        };
        send_goal_options.goal_response_callback =
            [this](rclcpp_action::ClientGoalHandle<SimulateSteps>::SharedPtr goal_handle)
        {
            if (!goal_handle)
            {
                RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by the action server");
            }
            else
            {
                RCLCPP_INFO(node_->get_logger(), "Goal accepted by the action server");
            }
        };
        send_goal_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<SimulateSteps>::WrappedResult& result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(node_->get_logger(), "Simulation completed successfully");
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "Simulation failed");
            }
        };
        auto goal_handle = client->async_send_goal(*goal, send_goal_options);
        if (rclcpp::spin_until_future_complete(node, goal_handle) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to call action");
            return;
        }
        auto goal_handle_result = goal_handle.get();
        auto result_future = client->async_get_result(goal_handle_result);
        if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to get action result");
            return;
        }
    }

    void SimulationWidget::StepSimulation()
    {
        int steps = ui_->stepsSpinBox->value();

        if (actionThread_.joinable())
        {
            actionThread_.join();
        }
        actionThread_ = std::thread(&SimulationWidget::ActionThreadWorker, this, steps);
    }

    void SimulationWidget::ResetSimulation()
    {
        simulation_interfaces::srv::ResetSimulation::Request request;
        auto selectedMode = ui_->resetModeCombo->currentText();
        auto it = ScopeNameToId.find(selectedMode.toStdString());
        Q_ASSERT(it != ScopeNameToId.end());
        request.scope = it->second;
        auto response = resetSimulationService_->call_service_sync(request);
    }

    void SimulationWidget::GetSimFeatures()
    {
        auto response = getSimFeaturesService_->call_service_sync();
        ui_->listCapabilities->clear();
        std::set<int> features;
        if (!response)
        {
            QMessageBox::warning(this, "Error",
                                 "Failed to get get sim features : " + QString::fromStdString(response.error()));
            return;
        }
        for (auto& feature : response->features.features)
        {
            features.emplace(feature);
            QString capabilityName;
            auto it = FeatureToName.find(feature);
            if (it == FeatureToName.end())
            {
                capabilityName = QString::asprintf("Unknow feature %d", feature);
                ui_->listCapabilities->addItem(capabilityName);
            }
        }
        for (auto& [featrueId, featureName] : FeatureToName)
        {
            const QString labelSupported = u8"✔️";
            const QString labelNotSupported = u8"❌";
            bool isSupported = features.find(featrueId) != features.end();
            QString label = QString(featureName.c_str()) + " : " + (isSupported ? labelSupported : labelNotSupported);
            QListWidgetItem* item = new QListWidgetItem(label);
            item->setTextAlignment(Qt::AlignLeft);
            item->setText(label);
            item->setToolTip(QString::fromStdString(FeatureDescription.at(featrueId)));
            ui_->listCapabilities->addItem(item);
        }
    }

    void SimulationWidget::StepSimulationService()
    {
        simulation_interfaces::srv::StepSimulation::Request request;
        request.steps = ui_->stepsSpinBox->value();
        auto response = stepSimulationService_->call_service_sync(request);
        if (!response)
        {
            QMessageBox::warning(this, "Error",
                                 "Failed to get step simulation : " + QString::fromStdString(response.error()));
            return;
        }
    }

    void SimulationWidget::DespawnButton()
    {
        simulation_interfaces::srv::DeleteEntity::Request request;
        request.entity = ui_->ComboEntities->currentText().toStdString();
        auto response = deleteEntityService_->call_service_sync(request);
        if (!response)
        {
            QMessageBox::warning(this, "Error",
                                 "Failed to get simulation features : " + QString::fromStdString(response.error()));
        }
    }

    geometry_msgs::msg::Quaternion CreateQuaternion(double w, double x, double y, double z)
    {
        geometry_msgs::msg::Quaternion q;
        q.w = w;
        q.x = x;
        q.y = y;
        q.z = z;
        return q;
    }

    void AddControlToInteractiveMarker(
        visualization_msgs::msg::InteractiveMarker& interactive_marker)
    {

        // move axis x
        const std::vector<geometry_msgs::msg::Quaternion> axesT = {
          CreateQuaternion(1, 1, 0, 0), // x-axis
          CreateQuaternion(1, 0, 1, 0), // y-axis
          CreateQuaternion(1, 0, 0, 1)  // z-axis
        };
        const std::vector<geometry_msgs::msg::Quaternion> axesR = {
          CreateQuaternion(1, 0, 0, 1)  // z-axis
        };

        for (const auto & axis : axesT)
        {
            visualization_msgs::msg::InteractiveMarkerControl control;
            control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
            control.orientation = axis;
            control.name = "move_" + std::to_string(axis.w);
            control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
            interactive_marker.controls.push_back(control);
        }
        for (const auto & axis : axesR)
        {
            visualization_msgs::msg::InteractiveMarkerControl control;
            control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
            control.orientation = axis;
            control.name = "rotate_" + std::to_string(axis.w);
            control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
            interactive_marker.controls.push_back(control);
        }


    }

    void SimulationWidget::GetAllEntities()
    {
        auto response = getEntitiesService_->call_service_sync();
        if (!response)
        {
            QMessageBox::warning(this, "Error",
                                 "Failed to get all entities : " + QString::fromStdString(response.error()));
            return;
        }

        ui_->ComboEntities->clear();
        for (const auto& entity : response->entities)
        {
            ui_->ComboEntities->addItem(QString(entity.c_str()));
        }
    }

    void SimulationWidget::GetEntityState(bool silent)
    {
        simulation_interfaces::srv::GetEntityState::Request request;
        request.entity = ui_->ComboEntities->currentText().toStdString();

        auto response = getEntityStateService_->call_service_sync(request, silent);
        if (!response)
        {
            QMessageBox::warning(this, "Error",
                                 "Failed to get entity state : " + QString::fromStdString(response.error()));
            return;
        }

        ui_->StatePosX->setValue(response->state.pose.position.x);
        ui_->StatePosY->setValue(response->state.pose.position.y);
        ui_->StatePosZ->setValue(response->state.pose.position.z);

        tf2::Quaternion q = tf2::Quaternion(response->state.pose.orientation.x, response->state.pose.orientation.y,
                                            response->state.pose.orientation.z, response->state.pose.orientation.w);

        const auto axis = q.getAxis();
        const auto angle = q.getAngle();
        ui_->RotVector->setText(VectorToQstring(axis));
        ui_->RotAngle->setValue(angle);

        ui_->StateVelX->setValue(response->state.twist.linear.x);
        ui_->StateVelY->setValue(response->state.twist.linear.y);
        ui_->StateVelZ->setValue(response->state.twist.linear.z);
        ui_->StateVelRotX->setValue(response->state.twist.angular.x);
        ui_->StateVelRotY->setValue(response->state.twist.angular.y);
        ui_->StateVelRotZ->setValue(response->state.twist.angular.z);

        // update marker server

        if (interactiveMarkerServer_) {
          visualization_msgs::msg::InteractiveMarker interactive_marker;
          interactive_marker.header.frame_id = "map";
          interactive_marker.name = request.entity;
          interactive_marker.description = "Manipulate entity " + request.entity;
          interactive_marker.pose.position = response->state.pose.position;
          interactive_marker.pose.orientation = response->state.pose.orientation;
          interactive_marker.scale = 1.0;

          AddControlToInteractiveMarker(interactive_marker);

          interactive_markers::InteractiveMarkerServer::FeedbackCallback cb =[this](const auto & feedback)
          {
            const auto& pose = feedback->pose;
            ui_->StatePosX->setValue(pose.position.x);
            ui_->StatePosY->setValue(pose.position.y);
            ui_->StatePosZ->setValue(pose.position.z);

            tf2::Quaternion q = tf2::Quaternion(pose.orientation.x, pose.orientation.y,
                                                pose.orientation.z, pose.orientation.w);

            const auto axis = q.getAxis();
            const auto angle = q.getAngle();
            ui_->RotVector->setText(VectorToQstring(axis));
            ui_->RotAngle->setValue(angle);
          };
          interactiveMarkerServer_->clear();
          interactiveMarkerServer_->insert(interactive_marker, cb);
          interactiveMarkerServer_->applyChanges();
        }
    }

    void SimulationWidget::SetEntityState()
    {
        simulation_interfaces::srv::SetEntityState::Request request;
        request.entity = ui_->ComboEntities->currentText().toStdString();
        request.state.pose.position.x = ui_->StatePosX->value();
        request.state.pose.position.y = ui_->StatePosY->value();
        request.state.pose.position.z = ui_->StatePosZ->value();

        const QString vectorStr = ui_->RotVector->text();
        const double angle = ui_->RotAngle->value();
        const auto vector = QStringToVector(vectorStr);
        tf2::Quaternion q(vector, angle);

        request.state.pose.orientation.x = q.x();
        request.state.pose.orientation.y = q.y();
        request.state.pose.orientation.z = q.z();
        request.state.pose.orientation.w = q.w();

        request.state.twist.linear.x = ui_->StateVelX->value();
        request.state.twist.linear.y = ui_->StateVelY->value();
        request.state.twist.linear.z = ui_->StateVelZ->value();
        request.state.twist.angular.x = ui_->StateVelRotX->value();
        request.state.twist.angular.y = ui_->StateVelRotY->value();
        request.state.twist.angular.z = ui_->StateVelRotZ->value();

        auto response = setEntityStateService_->call_service_sync(request);
        if (!response)
        {
            QMessageBox::warning(this, "Error",
                                 "Failed to get simulation features : " + QString::fromStdString(response.error()));
        }
    }

    void SimulationWidget::SpawnButton()
    {
        simulation_interfaces::srv::SpawnEntity::Request request;
        request.name = ui_->lineEditName->text().toStdString();
        request.uri = ui_->ComboSpawables->currentText().toStdString();
        request.entity_namespace = ui_->lineEditNamespace->text().toStdString();
        request.allow_renaming = ui_->checkBoxAllowRename->isChecked();

        request.initial_pose.pose.position.x = ui_->doubleSpinBoxX->value();
        request.initial_pose.pose.position.y = ui_->doubleSpinBoxY->value();
        request.initial_pose.pose.position.z = ui_->doubleSpinBoxZ->value();
        auto response = spawnEntityService_->call_service_sync(request);
        if (!response)
        {
            QMessageBox::warning(this, "Error",
                                 "Failed to get simulation features : " + QString::fromStdString(response.error()));
        }
        if (response && response->result.result == simulation_interfaces::msg::Result::RESULT_OK)
        {
            QString message = QString::asprintf("Spawned as %s", response->entity_name.c_str());
            QMessageBox::information(this, "Success", message);
        }
    }

    void SimulationWidget::GetSpawnables()
    {

        const auto response = getSpawnablesService_->call_service_sync();
        if (!response)
        {
            QString errorMessage = QString::fromStdString(response.error());
            QMessageBox::warning(this, "Error", "Failed to get spawnables : " + errorMessage);
            ui_->ComboSpawables->clear();
            return;
        }
        QString selectedSpawnable = ui_->ComboSpawables->currentText();
        ui_->ComboSpawables->clear();
        auto spawnables = response->spawnables;
        std::sort(spawnables.begin(), spawnables.end(), [](const auto& a, const auto& b) { return a.uri < b.uri; });
        for (const auto& spawnable : spawnables)
        {
            ui_->ComboSpawables->addItem(QString::fromStdString(spawnable.uri));
        }
        if (ui_->ComboSpawables->findText(selectedSpawnable) != -1)
        {
            ui_->ComboSpawables->setCurrentText(selectedSpawnable);
        }
        else
        {
            ui_->ComboSpawables->setCurrentIndex(0);
        }
    }

} // namespace q_simulation_interfaces

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(q_simulation_interfaces::SimulationPanel, rviz_common::Panel)
