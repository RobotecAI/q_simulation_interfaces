#include "q_simulation_interfaces/simulation_panel.hpp"
#include "ui_sim_widget.h"
#include <QMessageBox>
#include "service.h"
#include <tf2/LinearMath/Quaternion.h>
#include <QDebug>
#include <rclcpp_action/create_client.hpp>
#include <simulation_interfaces/action/simulate_steps.hpp>
#include "stringToKeys.h"
#include <rviz_common/display_context.hpp>
#include "vector_utils.hpp"


namespace q_simulation_interfaces
{

SimulationPanel::SimulationPanel(QWidget *parent)
        : rviz_common::Panel(parent),
          ui_(new Ui::simWidgetUi) {
    ui_->setupUi(this);
    
    for (const auto& [name, _] : ScopeNameToId)
    {
        ui_->resetModeCombo->addItem(QString::fromStdString(name));
    }
    for (const auto& [name, _] : SimStateNameToId)
    {
        ui_->simStateToSetComboBox->addItem(QString::fromStdString(name));
    }

    connect(ui_->PushButtonRefresh, &QPushButton::clicked, this, &SimulationPanel::GetSpawnables);
    connect(ui_->SpawnButton, &QPushButton::clicked, this, &SimulationPanel::SpawnButton);
    connect(ui_->getAllEntitiesButton, &QPushButton::clicked, this, &SimulationPanel::GetAllEntities);
    connect(ui_->getEntityStateButton, &QPushButton::clicked, this, &SimulationPanel::GetEntityState);
    connect(ui_->setEntityStateButton, &QPushButton::clicked, this, &SimulationPanel::SetEntityState);
    connect(ui_->despawnButton, &QPushButton::clicked, this, &SimulationPanel::DespawnButton);
    connect(ui_->GetSimCapabilites, &QPushButton::clicked, this, &SimulationPanel::GetSimFeatures);
    connect(ui_->resetSimButton, &QPushButton::clicked, this, &SimulationPanel::ResetSimulation);
    connect(ui_->stepSimButtonAction, &QPushButton::clicked, this, &SimulationPanel::StepSimulation);
    connect(ui_->getSimStateBtn, &QPushButton::clicked, this, &SimulationPanel::GetSimulationState);
    connect(ui_->setSimStateButton, &QPushButton::clicked, this, &SimulationPanel::SetSimulationState);
    connect(ui_->stepSimServiceButton, &QPushButton::clicked, this, &SimulationPanel::StepSimulationService);
    connect(ui_->ComboEntities, &QComboBox::currentTextChanged, this, [this](){
        this->GetEntityState(true);
    });
}

SimulationPanel::~SimulationPanel() {
    if (actionThread_.joinable()) {
        actionThread_.join();
    }
    delete ui_;
}

void SimulationPanel::onInitialize()
{
    node_ = rclcpp::Node::make_shared("qt_gui_node");
}

void SimulationPanel::setRosNode(rclcpp::Node::SharedPtr node)
{
    (void)node; // Suppress unused variable warning
}

void SimulationPanel::GetSimulationState()
{
    Service<simulation_interfaces::srv::GetSimulationState> service("/get_simulation_state", node_);
    auto response = service.call_service_sync();
    if (response)
    {
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
}

void SimulationPanel::SetSimulationState()
{
    simulation_interfaces::srv::SetSimulationState::Request request;
    auto selectedMode = ui_->simStateToSetComboBox->currentText();
    auto it = SimStateNameToId.find(selectedMode.toStdString());
    Q_ASSERT(it != SimStateNameToId.end());
    Service<simulation_interfaces::srv::SetSimulationState> service("/set_simulation_state", node_);
    request.state.state = it->second;
    auto response = service.call_service_sync(request);
    if (response)
    {
        GetSimulationState();
    }
}

void SimulationPanel::ActionThreadWorker(int steps) {
    // create node
    auto node = rclcpp::Node::make_shared("qt_gui_action_node");
    using SimulateSteps=simulation_interfaces::action::SimulateSteps;
    auto client = rclcpp_action::create_client<SimulateSteps>(node, "/simulate_steps");

    auto send_goal_options = rclcpp_action::Client<SimulateSteps>::SendGoalOptions();
    auto goal = std::make_shared<SimulateSteps::Goal>();
    goal->steps = steps;
    send_goal_options.feedback_callback =
        [this](rclcpp_action::ClientGoalHandle<SimulateSteps>::SharedPtr goal_handle,
               const std::shared_ptr<const SimulateSteps::Feedback> feedback) {
            float progress = static_cast<float>(feedback->completed_steps) / feedback->remaining_steps;
            ui_->simProgressBar->setValue(static_cast<int>(progress * 100));
        };
    send_goal_options.goal_response_callback = [this](rclcpp_action::ClientGoalHandle<SimulateSteps>::SharedPtr goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by the action server");
        } else {
            RCLCPP_INFO(node_->get_logger(), "Goal accepted by the action server");
        }
    };
    send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<SimulateSteps>::WrappedResult &result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(node_->get_logger(), "Simulation completed successfully");
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Simulation failed");
        }
    };
    auto goal_handle = client->async_send_goal(*goal, send_goal_options);
    if (rclcpp::spin_until_future_complete(node, goal_handle) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to call action");
        return;
    }
    auto goal_handle_result = goal_handle.get();
    auto result_future = client->async_get_result(goal_handle_result);
    if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to get action result");
        return;
    }
}

void SimulationPanel::StepSimulation() {
    int steps = ui_->stepsSpinBox->value();

    if (actionThread_.joinable())
    {
        actionThread_.join();
    }
    actionThread_ = std::thread(&SimulationPanel::ActionThreadWorker, this, steps);
}

void SimulationPanel::ResetSimulation() {
    simulation_interfaces::srv::ResetSimulation::Request request;
    Service<simulation_interfaces::srv::ResetSimulation> service("/reset_simulation", node_);
    auto selectedMode = ui_->resetModeCombo->currentText();
    auto it = ScopeNameToId.find(selectedMode.toStdString());
    Q_ASSERT(it != ScopeNameToId.end());
    request.scope = it->second;
    auto response = service.call_service_sync(request);
}

void SimulationPanel::GetSimFeatures()
{
    Service<simulation_interfaces::srv::GetSimulatorFeatures> service("/get_simulation_features", node_);
    auto response = service.call_service_sync();
    ui_->listCapabilities->clear();
    std::set<int> features;
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
    for (auto &[featrueId, featureName] : FeatureToName)
    {
        const QString labelSupported = u8"✔️";
        const QString labelNotSupported = u8"❌";
        bool isSupported = features.find(featrueId) != features.end();
        QString label = QString(featureName.c_str()) + " : " + (isSupported ? labelSupported : labelNotSupported);
        QListWidgetItem *item = new QListWidgetItem(label);
        item->setTextAlignment(Qt::AlignLeft);
        item->setText(label);
        item->setToolTip(QString::fromStdString(FeatureDescription.at(featrueId)));
        ui_->listCapabilities->addItem(item);
    }
}

void SimulationPanel::StepSimulationService() {
    simulation_interfaces::srv::StepSimulation::Request request;
    request.steps = ui_->stepsSpinBox->value();
    Service<simulation_interfaces::srv::StepSimulation> service("/step_simulation", node_);
    auto response = service.call_service_sync(request);
    if (response) {
        // Handle the response if needed
    }
}

void SimulationPanel::DespawnButton() {
    simulation_interfaces::srv::DeleteEntity::Request request;
    request.entity = ui_->ComboEntities->currentText().toStdString();
    Service<simulation_interfaces::srv::DeleteEntity> service("/delete_entity", node_);
    service.call_service_sync(request);
}

void SimulationPanel::GetAllEntities() {
    Service<simulation_interfaces::srv::GetEntities> service("/get_entities", node_);
    auto response = service.call_service_sync();
    if (response)
    {
        ui_->ComboEntities->clear();
        for (const auto &entity: response->entities) {
            ui_->ComboEntities->addItem(QString(entity.c_str()));
        }
    }
}

void SimulationPanel::GetEntityState(bool silent) {
    simulation_interfaces::srv::GetEntityState::Request request;
    request.entity = ui_->ComboEntities->currentText().toStdString();

    Service<simulation_interfaces::srv::GetEntityState> service("/get_entity_state", node_);
    auto response = service.call_service_sync(request, silent);
    if (response) {
        ui_->StatePosX->setValue(response->state.pose.position.x);
        ui_->StatePosY->setValue(response->state.pose.position.y);
        ui_->StatePosZ->setValue(response->state.pose.position.z);

        tf2::Quaternion q = tf2::Quaternion(
                response->state.pose.orientation.x,
                response->state.pose.orientation.y,
                response->state.pose.orientation.z,
                response->state.pose.orientation.w);

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
    }
}

void SimulationPanel::SetEntityState() {
    simulation_interfaces::srv::SetEntityState::Request request;
    request.entity = ui_->ComboEntities->currentText().toStdString();
    request.state.pose.position.x = ui_->StatePosX->value();
    request.state.pose.position.y = ui_->StatePosY->value();
    request.state.pose.position.z = ui_->StatePosZ->value();

    const QString vectorStr = ui_->RotVector->text();
    const double angle = ui_->RotAngle->value();
    const auto vector = QStringToVector(vectorStr);
    tf2::Quaternion q (vector,angle);

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

    Service<simulation_interfaces::srv::SetEntityState> service("/set_entity_state", node_);
    auto response = service.call_service_sync(request);
}

void SimulationPanel::SpawnButton() {
    simulation_interfaces::srv::SpawnEntity::Request request;
    request.name = ui_->lineEditName->text().toStdString();
    request.uri = ui_->ComboSpawables->currentText().toStdString();
    request.entity_namespace = ui_->lineEditNamespace->text().toStdString();
    request.allow_renaming = ui_->checkBoxAllowRename->isChecked();

    request.initial_pose.pose.position.x = ui_->doubleSpinBoxX->value();
    request.initial_pose.pose.position.y = ui_->doubleSpinBoxY->value();
    request.initial_pose.pose.position.z = ui_->doubleSpinBoxZ->value();
    Service<simulation_interfaces::srv::SpawnEntity> service("/spawn_entity", node_);
    auto response = service.call_service_sync(request);
    if (response && response->result.result == simulation_interfaces::msg::Result::RESULT_OK)
    {
        QString message = QString::asprintf("Spawned as %s", response->entity_name.c_str());
        QMessageBox::information(this, "Success", message);
    }
}

void SimulationPanel::GetSpawnables() {

    Service<simulation_interfaces::srv::GetSpawnables>service("/get_spawnables", node_);
    auto response = service.call_service_sync();
    if (!response) {
        QMessageBox::warning(this, "Error", "Failed to get spawnables");
        return;
    }
    QString selectedSpawnable = ui_->ComboSpawables->currentText();
    ui_->ComboSpawables->clear();

    std::sort(response->spawnables.begin(), response->spawnables.end(),
              [](const auto &a, const auto &b) {
                  return a.uri < b.uri;
              });
    for (const auto &spawnable: response->spawnables) {
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

}  // namespace q_simulation_interfaces

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(q_simulation_interfaces::SimulationPanel, rviz_common::Panel)