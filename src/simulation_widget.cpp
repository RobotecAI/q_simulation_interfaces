#include "simulation_widget.hpp"
#include <QMessageBox>
#include <QTimer>
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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace q_simulation_interfaces
{
    namespace
    {
        geometry_msgs::msg::Quaternion CreateQuaternion(double w, double x, double y, double z)
        {
            geometry_msgs::msg::Quaternion q;
            q.w = w;
            q.x = x;
            q.y = y;
            q.z = z;
            return q;
        }

        void AddControlToInteractiveMarker(visualization_msgs::msg::InteractiveMarker& interactive_marker, bool rotationZ)
        {
            // move axis x
            const std::vector<geometry_msgs::msg::Quaternion> axesT = {
                CreateQuaternion(1, 1, 0, 0), // x-axis
                CreateQuaternion(1, 0, 1, 0), // y-axis
                CreateQuaternion(1, 0, 0, 1) // z-axis
            };
            std::vector<geometry_msgs::msg::Quaternion> axesR;
            if (rotationZ)
            {
                CreateQuaternion(1, 0, 1, 0); // z-axis
            }

            for (const auto& axis : axesT)
            {
                visualization_msgs::msg::InteractiveMarkerControl control;
                control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
                control.orientation = axis;
                control.name = "move_" + std::to_string(axis.w);
                control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
                interactive_marker.controls.push_back(control);
            }
            for (const auto& axis : axesR)
            {
                visualization_msgs::msg::InteractiveMarkerControl control;
                control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
                control.orientation = axis;
                control.name = "rotate_" + std::to_string(axis.w);
                control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
                interactive_marker.controls.push_back(control);
            }
        }

    } // namespace

    SimulationWidget::SimulationWidget(QWidget* parent) : QWidget(parent), ui_(new Ui::simWidgetUi)
    {
        std::cout << "Simulation widget created" << std::endl;
        ui_->setupUi(this);
        ui_->frameStateLineEdit->setText("map");
        ui_->spawnFrameLineEdit->setText("map");
        for (const auto& [name, _] : ScopeNameToId)
        {
            ui_->resetModeCombo->addItem(QString::fromStdString(name));
        }
        for (const auto& [name, _] : SimStateNameToId)
        {
            ui_->simStateToSetComboBox->addItem(QString::fromStdString(name));
        }

        timer_ = new QTimer(this);
        connect(timer_, &QTimer::timeout, this, &SimulationWidget::UpdateServices);
        timer_->setSingleShot(false);
        timer_->start(100);
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

        // Connect spawn position spin boxes to update marker
        connect(ui_->doubleSpinBoxX, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
                &SimulationWidget::UpdateSpawnPointMarker);
        connect(ui_->doubleSpinBoxY, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
                &SimulationWidget::UpdateSpawnPointMarker);
        connect(ui_->doubleSpinBoxZ, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
                &SimulationWidget::UpdateSpawnPointMarker);
    }

    SimulationWidget::~SimulationWidget()
    {
        if (actionThread_.joinable())
        {
            actionThread_.join();
        }
        delete ui_;
    }

    void SimulationWidget::intiliaze(rclcpp::Node::SharedPtr node)
    {
        node_ = node;
        // tf transform listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        // Initialize service objects
        getSpawnablesService_ =
            std::make_shared<Service<simulation_interfaces::srv::GetSpawnables>>("/get_spawnables", node);
        serviceInterfaces_.push_back(getSpawnablesService_);

        spawnEntityService_ =
            std::make_shared<Service<simulation_interfaces::srv::SpawnEntity>>("/spawn_entity", node);
        serviceInterfaces_.push_back(spawnEntityService_);

        getEntitiesService_ =
            std::make_shared<Service<simulation_interfaces::srv::GetEntities>>("/get_entities", node);
        serviceInterfaces_.push_back(getEntitiesService_);

        getEntityStateService_ =
            std::make_shared<Service<simulation_interfaces::srv::GetEntityState>>("/get_entity_state", node);
        serviceInterfaces_.push_back(getEntityStateService_);

        setEntityStateService_ =
            std::make_shared<Service<simulation_interfaces::srv::SetEntityState>>("/set_entity_state", node);
        serviceInterfaces_.push_back(setEntityStateService_);


        deleteEntityService_ =
            std::make_shared<Service<simulation_interfaces::srv::DeleteEntity>>("/delete_entity", node);
        serviceInterfaces_.push_back(getEntityStateService_);


        getSimFeaturesService_ = std::make_shared<Service<simulation_interfaces::srv::GetSimulatorFeatures>>(
            "/get_simulation_features", node);
        serviceInterfaces_.push_back(getSimFeaturesService_);

        resetSimulationService_ =
            std::make_shared<Service<simulation_interfaces::srv::ResetSimulation>>("/reset_simulation", node);
        serviceInterfaces_.push_back(resetSimulationService_);

        getSimulationStateService_ =
            std::make_shared<Service<simulation_interfaces::srv::GetSimulationState>>("/get_simulation_state", node);
        serviceInterfaces_.push_back(getSimulationStateService_);

        setSimulationStateService_ =
            std::make_shared<Service<simulation_interfaces::srv::SetSimulationState>>("/set_simulation_state", node);
        serviceInterfaces_.push_back(setSimulationStateService_);

        stepSimulationService_ =
            std::make_shared<Service<simulation_interfaces::srv::StepSimulation>>("/step_simulation", node);
        serviceInterfaces_.push_back(stepSimulationService_);
        //
        interactiveMarkerServer_ =
            std::make_shared<interactive_markers::InteractiveMarkerServer>(InteractiveMarkerNamespaceValue, node);

        // Create spawn point marker
        CreateSpawnPointMarker();
    }

    void SimulationWidget::GetSimulationState()
    {
        auto cb = [this](auto response)
        {
            ProduceWarningIfProblem(this, "Get Simulation State", response);
            if (response && response->result.result == simulation_interfaces::msg::Result::RESULT_OK)
            {
                {
                    int stateId = response->state.state;
                    QString stateName;
                    auto it = SimStateIdToName.find(stateId);
                    if (it == SimStateIdToName.end())
                    {
                        stateName = QString::asprintf("Unknown state %d", stateId);
                    }
                    else
                    {
                        stateName = QString::fromStdString(it->second);
                    }
                    ui_->simStateLabel->setText(stateName);
                }
            }
        };
        getSimulationStateService_->call_service_async(cb);
    }

    void SimulationWidget::SetSimulationState()
    {
        simulation_interfaces::srv::SetSimulationState::Request request;
        auto selectedMode = ui_->simStateToSetComboBox->currentText();
        auto it = SimStateNameToId.find(selectedMode.toStdString());
        Q_ASSERT(it != SimStateNameToId.end());
        request.state.state = it->second;

        auto cb = [this](auto response)
        {
            ProduceWarningIfProblem(this, "Set Simulation State", response);
            if (response)
            {
                GetSimulationState(); // Refresh state display
            }
        };
        setSimulationStateService_->call_service_async(cb, request);
    }

    void SimulationWidget::ActionThreadWorker(int steps)
    {
        actionThreadRunning_.store(true);
        // create node for action client
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
            actionThreadProgress_.store(progress);
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
                actionThreadProgress_.store(1.0f);
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
            actionThreadRunning_.store(false);
            return;
        }
        auto goal_handle_result = goal_handle.get();
        auto result_future = client->async_get_result(goal_handle_result);
        if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to get action result");
            actionThreadRunning_.store(false);
            return;
        }
        actionThreadRunning_.store(false);
    }

    void SimulationWidget::StepSimulation()
    {
        int steps = ui_->stepsSpinBox->value();
        if (actionThreadRunning_.load() == true)
        {
            QMessageBox::warning(this, "Action in progress", "An action is already running. Please wait.");
            return;
        }

        if (actionThread_.joinable())
        {
            actionThread_.join();
        }
        actionThreadRunning_.store(true);
        actionThread_ = std::thread(&SimulationWidget::ActionThreadWorker, this, steps);

    }

    void SimulationWidget::ResetSimulation()
    {
        simulation_interfaces::srv::ResetSimulation::Request request;
        auto selectedMode = ui_->resetModeCombo->currentText();
        auto it = ScopeNameToId.find(selectedMode.toStdString());
        Q_ASSERT(it != ScopeNameToId.end());
        request.scope = it->second;

        auto cb = [this](auto response)
        {
            ProduceWarningIfProblem(this, "Reset Simulation", response);
            // Refresh entities and state after reset
            if (response && response->result.result == simulation_interfaces::msg::Result::RESULT_OK)
            {
                GetAllEntities();
                GetSimulationState();
            }
        };
        resetSimulationService_->call_service_async(cb, request);
    }

    void SimulationWidget::GetSimFeatures()
    {
        auto cb = [this](auto response)
        {
            ui_->listCapabilities->clear();
            std::set<int> features;

            ProduceWarningIfProblem(this, "Get Simulation Features", response);
            if (response)
            {
                for (auto& feature : response->features.features)
                {
                    features.emplace(feature);
                }

                for (auto& [featureId, featureName] : FeatureToName)
                {
                    const QString labelSupported = u8"✔️";
                    const QString labelNotSupported = u8"❌";
                    bool isSupported = features.find(featureId) != features.end();
                    QString label =
                        QString(featureName.c_str()) + " : " + (isSupported ? labelSupported : labelNotSupported);
                    QListWidgetItem* item = new QListWidgetItem(label);
                    item->setTextAlignment(Qt::AlignLeft);
                    item->setToolTip(QString::fromStdString(FeatureDescription.at(featureId)));
                    ui_->listCapabilities->addItem(item);
                }
            }
        };
        getSimFeaturesService_->call_service_async(cb);
    }

    void SimulationWidget::StepSimulationService()
    {
        simulation_interfaces::srv::StepSimulation::Request request;
        request.steps = ui_->stepsSpinBox->value();

        auto cb = [this](auto response)
        {
            ProduceWarningIfProblem(this, "Step Simulation", response);
            if (response && response->result.result == simulation_interfaces::msg::Result::RESULT_OK)
            {
                GetSimulationState();
            }
        };
        stepSimulationService_->call_service_async(cb, request);
    }

    void SimulationWidget::DespawnButton()
    {
        simulation_interfaces::srv::DeleteEntity::Request request;
        request.entity = ui_->ComboEntities->currentText().toStdString();

        auto cb = [this](auto response)
        {
            ProduceWarningIfProblem(this, "Despawn", response);
            // Refresh entity list after successful deletion
            if (response)
            {
                GetAllEntities();
            }
        };
        deleteEntityService_->call_service_async(cb, request);
    }

    void SimulationWidget::GetAllEntities()
    {
        auto cb = [this](auto response)
        {
            ProduceWarningIfProblem(this, "GetAllEntities", response);
            // Refresh entity list after successful deletion
            if (response && response->result.result == simulation_interfaces::msg::Result::RESULT_OK)
            {
                ui_->ComboEntities->clear();
                for (const auto& entity : response->entities)
                {
                    ui_->ComboEntities->addItem(QString(entity.c_str()));
                }
            }
        };
        getEntitiesService_->call_service_async(cb);
    }

    void SimulationWidget::GetEntityState(bool silent)
    {
        simulation_interfaces::srv::GetEntityState::Request request;
        request.entity = ui_->ComboEntities->currentText().toStdString();

        auto cb = [this, silent, entity = request.entity](auto response)
        {
            if (!silent)
            {
                ProduceWarningIfProblem(this, "GetEntityState", response);
            }
            // Refresh entity list after successful deletion
            if (response && response->result.result == simulation_interfaces::msg::Result::RESULT_OK)
            {

                // Update UI elements
                ui_->StatePosX->setValue(response->state.pose.position.x);
                ui_->StatePosY->setValue(response->state.pose.position.y);
                ui_->StatePosZ->setValue(response->state.pose.position.z);

                tf2::Quaternion q =
                    tf2::Quaternion(response->state.pose.orientation.x, response->state.pose.orientation.y,
                                    response->state.pose.orientation.z, response->state.pose.orientation.w);

                const auto axis = q.getAxis();
                const auto angle = q.getAngle();
                ui_->RotVector->setText(VectorToQstring(axis));
                ui_->RotAngle->setValue(angle);
                if (response->state.header.frame_id.empty())
                {
                    RCLCPP_ERROR(node_->get_logger(), "Empty frame id");
                }
                else
                {
                    ui_->frameStateLineEdit->setText(QString::fromStdString(response->state.header.frame_id));
                }
                ui_->StateVelX->setValue(response->state.twist.linear.x);
                ui_->StateVelY->setValue(response->state.twist.linear.y);
                ui_->StateVelZ->setValue(response->state.twist.linear.z);
                ui_->StateVelRotX->setValue(response->state.twist.angular.x);
                ui_->StateVelRotY->setValue(response->state.twist.angular.y);
                ui_->StateVelRotZ->setValue(response->state.twist.angular.z);

                // Update interactive marker if available
                if (interactiveMarkerServer_)
                {
                    visualization_msgs::msg::InteractiveMarker interactive_marker;
                    interactive_marker.header.frame_id = ui_->frameStateLineEdit->text().toStdString();
                    interactive_marker.name = "Manipulator";
                    interactive_marker.description = "Manipulate entity " + entity;
                    interactive_marker.pose.position = response->state.pose.position;
                    interactive_marker.pose.orientation = response->state.pose.orientation;
                    interactive_marker.scale = 0.2;

                    AddControlToInteractiveMarker(interactive_marker, true);

                    interactive_markers::InteractiveMarkerServer::FeedbackCallback feedbackCb =
                        [this, entity](const auto& feedback)
                    {
                        const auto& pose = feedback->pose;

                        ui_->StatePosX->setValue(pose.position.x);
                        ui_->StatePosY->setValue(pose.position.y);
                        ui_->StatePosZ->setValue(pose.position.z);

                        tf2::Quaternion q = tf2::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z,
                                                            pose.orientation.w);

                        const auto axis = q.getAxis();
                        const auto angle = q.getAngle();
                        ui_->RotVector->setText(VectorToQstring(axis));
                        ui_->RotAngle->setValue(angle);
                        ui_->frameStateLineEdit->setText(QString::fromStdString(feedback->header.frame_id));

                        simulation_interfaces::srv::SetEntityState::Request request;
                        request.entity = entity;
                        request.state.pose = pose;
                        request.state.header.frame_id = feedback->header.frame_id;
                        setEntityStateService_->call_service_async(nullptr,request);
                    };
                    interactiveMarkerServer_->insert(interactive_marker, feedbackCb);
                    interactiveMarkerServer_->applyChanges();
                }
            }
        };
        getEntityStateService_->call_service_async(cb, request);
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
        request.state.header.frame_id = ui_->frameStateLineEdit->text().toStdString();
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

        auto cb = [this](auto response) { ProduceWarningIfProblem(this, "SetEntityState", response); };
        setEntityStateService_->call_service_async(cb, request);
    }


    void SimulationWidget::SpawnButton()
    {
        simulation_interfaces::srv::SpawnEntity::Request request;
        request.name = ui_->lineEditName->text().toStdString();
        request.uri = ui_->ComboSpawables->currentText().toStdString();
        request.entity_namespace = ui_->lineEditNamespace->text().toStdString();
        request.allow_renaming = ui_->checkBoxAllowRename->isChecked();
        request.initial_pose.header.frame_id = ui_->spawnFrameLineEdit->text().toStdString();
        request.initial_pose.pose.position.x = ui_->doubleSpinBoxX->value();
        request.initial_pose.pose.position.y = ui_->doubleSpinBoxY->value();
        request.initial_pose.pose.position.z = ui_->doubleSpinBoxZ->value();

        auto cb = [this](auto response)
        {
            ProduceWarningIfProblem(this, "SpawnEntity", response);
            if (!response && response->result.result == simulation_interfaces::msg::Result::RESULT_OK)
            {
                QString message = QString::asprintf("Spawned as %s", response->entity_name.c_str());
                QMessageBox::information(this, "Success", message);
                // Refresh entity list after successful spawn
                GetAllEntities();
            }
        };
        spawnEntityService_->call_service_async(cb, request);
    }

    void SimulationWidget::GetSpawnables()
    {
        auto cb = [this](auto response)
        {
            ui_->ComboSpawables->clear();
            ProduceWarningIfProblem(this, "GetSpawnables", response);
            if (response && response->result.result == simulation_interfaces::msg::Result::RESULT_OK)
            {
                QString selectedSpawnable = ui_->ComboSpawables->currentText();

                auto spawnables = response->spawnables;
                std::sort(spawnables.begin(), spawnables.end(),
                          [](const auto& a, const auto& b) { return a.uri < b.uri; });

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
        };
        getSpawnablesService_->call_service_async(cb);
    }

    void SimulationWidget::UpdateServices()
    {

        if (actionThreadRunning_)
        {
            ui_->stepSimButtonAction->setEnabled(false);
            ui_->stepSimServiceButton->setEnabled(false);
            ui_->simProgressBar->setValue(static_cast<int>(this->actionThreadProgress_ * 100));
        }
        else
        {
            ui_->stepSimButtonAction->setEnabled(true);
            ui_->stepSimServiceButton->setEnabled(true);
        }
        for (auto& service : serviceInterfaces_)
        {
            if (service)
            {
                service->check_service_result();
            }
        }
    }

    void SimulationWidget::CreateSpawnPointMarker()
    {
        if (!interactiveMarkerServer_)
        {
            return;
        }

        visualization_msgs::msg::InteractiveMarker spawnMarker;
        spawnMarker.header.frame_id = ui_->spawnFrameLineEdit->text().toStdString();
        spawnMarker.name = "spawn_point";
        spawnMarker.description = "Spawn Point - Drag to set spawn location";
        spawnMarker.scale = 1.0;

        // Set initial position from GUI
        spawnMarker.pose.position.x = ui_->doubleSpinBoxX->value();
        spawnMarker.pose.position.y = ui_->doubleSpinBoxY->value();
        spawnMarker.pose.position.z = ui_->doubleSpinBoxZ->value();
        spawnMarker.pose.orientation.w = 1.0; // Default orientation

        AddControlToInteractiveMarker(spawnMarker, false);

        // Add visual marker (cube)
        visualization_msgs::msg::InteractiveMarkerControl visualControl;
        visualControl.always_visible = true;
        visualControl.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::NONE;

        visualization_msgs::msg::Marker cubeMarker;
        cubeMarker.type = visualization_msgs::msg::Marker::CUBE;
        cubeMarker.scale.x = 0.3;
        cubeMarker.scale.y = 0.3;
        cubeMarker.scale.z = 0.3;
        cubeMarker.color.r = 0.0;
        cubeMarker.color.r = 0.0;
        cubeMarker.color.g = 1.0;
        cubeMarker.color.b = 0.0;
        cubeMarker.color.a = 0.8;

        visualControl.markers.push_back(cubeMarker);
        spawnMarker.controls.push_back(visualControl);

        // Set up feedback callback to update GUI when marker is moved
        interactive_markers::InteractiveMarkerServer::FeedbackCallback feedbackCb = [this](const auto& feedback)
        {
            if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE)
            {
                    const auto& pose = feedback->pose;
                    ui_->spawnFrameLineEdit->setText(QString::fromStdString(feedback->header.frame_id));
                    ui_->doubleSpinBoxX->setValue(pose.position.x);
                    ui_->doubleSpinBoxY->setValue(pose.position.y);
                    ui_->doubleSpinBoxZ->setValue(pose.position.z);
            }
        };

        interactiveMarkerServer_->insert(spawnMarker, feedbackCb);
        interactiveMarkerServer_->applyChanges();
    }

    void SimulationWidget::UpdateSpawnPointMarker()
    {
        if (!interactiveMarkerServer_)
        {
            return;
        }

        // Get current marker
        visualization_msgs::msg::InteractiveMarker spawnMarker;
        if (!interactiveMarkerServer_->get("spawn_point", spawnMarker))
        {
            // Marker doesn't exist, create it
            CreateSpawnPointMarker();
            return;
        }

        // Update position from GUI
        geometry_msgs::msg::Pose newPose;
        std_msgs::msg::Header newHeader;
        newHeader.frame_id = ui_->spawnFrameLineEdit->text().toStdString();
        newPose.position.x = ui_->doubleSpinBoxX->value();
        newPose.position.y = ui_->doubleSpinBoxY->value();
        newPose.position.z = ui_->doubleSpinBoxZ->value();
        newPose.orientation.w = 1.0;

        interactiveMarkerServer_->setPose("spawn_point", newPose, newHeader);
        interactiveMarkerServer_->applyChanges();
    }

} // namespace q_simulation_interfaces

