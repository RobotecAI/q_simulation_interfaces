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

#include <QLabel>
#include <QMetaType>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <q_simulation_interfaces/service_discovery.h>
#include <rclcpp/rclcpp.hpp>

Q_DECLARE_METATYPE(q_simulation_interfaces::ServiceType)

namespace q_simulation_interfaces
{
    ServiceDiscovery::ServiceDiscovery()
    {
        qRegisterMetaType<q_simulation_interfaces::ServiceType>("q_simulation_interfaces::ServiceType");

        autoDiscoveryTimer_ = new QTimer(this);
        connect(autoDiscoveryTimer_, &QTimer::timeout, this, &ServiceDiscovery::discoverServices);
    }

    ServiceDiscovery::~ServiceDiscovery()
    {
        // Clean up dynamically allocated resources
        for (auto* comboBox : serviceComboBoxes_)
        {
            delete comboBox;
        }
        for (auto* label : serviceLabels_)
        {
            delete label;
        }
        delete autoDiscoveryTimer_;
    }

    void ServiceDiscovery::initializeServiceUI(QWidget* parent)
    {
        parent_ = parent;

        // Find the existing Services tab and its layout
        auto* tabWidget = parent_->findChild<QTabWidget*>("tabWidget");
        if (!tabWidget)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ServiceDiscovery"), "Could not find tabWidget in parent");
            return;
        }

        auto* servicesTab = parent_->findChild<QWidget*>("servicesNames");
        if (!servicesTab)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ServiceDiscovery"), "Could not find servicesNames tab");
            return;
        }

        auto* servicesLayout = servicesTab->findChild<QVBoxLayout*>("servicesLayout");
        if (!servicesLayout)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ServiceDiscovery"), "Could not find servicesLayout");
            return;
        }

        // Find existing buttons
        discoverButton_ = parent_->findChild<QPushButton*>("discoverServicesButton");
        autoDiscoveryCheckBox_ = parent_->findChild<QCheckBox*>("autodiscoverServicesCheckbox");
        autoSelectServicesButton_ = parent_->findChild<QPushButton*>("autoSelectServices");

        if (discoverButton_)
        {
            connect(discoverButton_, &QPushButton::clicked, this, &ServiceDiscovery::discoverServices);
        }

        if (autoDiscoveryCheckBox_)
        {
            connect(autoDiscoveryCheckBox_, &QCheckBox::toggled, this, &ServiceDiscovery::onAutoDiscoveryToggled);
        }

        if (autoSelectServicesButton_)
        {
            connect(autoSelectServicesButton_, &QPushButton::clicked, this, &ServiceDiscovery::autoSelectServices);
        }

        for (const auto& idlType : SUPPORTED_SERVICE_IDL_TYPES)
        {
            createServiceComboBox(idlType, servicesLayout);
        }
    }

    void ServiceDiscovery::initializeServices(rclcpp::Node::SharedPtr node)
    {
        node_ = node;
        discoverServices();
    }

    void ServiceDiscovery::discoverServices()
    {
        if (!node_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ServiceDiscovery"), "Node not initialized");
            return;
        }

        std::map<std::string, std::vector<std::string>> discovered;

        try
        {
            auto serviceNamesAndTypes = node_->get_service_names_and_types();

            std::set<std::string> supportedTypes;
            for (const auto& serviceInfo : SUPPORTED_SERVICE_IDL_TYPES)
            {
                supportedTypes.insert(serviceInfo.type_string);
            }

            for (const auto& [serviceName, serviceTypes] : serviceNamesAndTypes)
            {
                if (serviceName.empty())
                {
                    continue;
                }
            

                for (const auto& serviceType : serviceTypes)
                {
                    if (supportedTypes.find(serviceType) != supportedTypes.end())
                    {
                        discovered[serviceType].push_back(serviceName);
                    }
                }

                // Action discovery - look for _action/send_goal services
                if (serviceName.find("/_action/send_goal") != std::string::npos)
                {
                    // Extract action name by removing "/_action/send_goal"
                    std::string actionName = serviceName.substr(0, serviceName.find("/_action/send_goal"));

                    // Check if this matches any supported action type
                    for (const auto& serviceInfo : SUPPORTED_SERVICE_IDL_TYPES)
                    {
                        if (serviceInfo.is_action)
                        {
                            discovered[serviceInfo.type_string].push_back(actionName);
                        }
                    }
                }
            }

            for (const auto& serviceInfo : SUPPORTED_SERVICE_IDL_TYPES)
            {
                QComboBox* comboBox = serviceComboBoxes_[serviceInfo.type];
                if (comboBox)
                {
                    comboBox->blockSignals(true); // Prevent signals during update

                    // Store current selection
                    QString previousSelection = comboBox->currentText();

                    comboBox->clear();
                    comboBox->addItem("Not selected");

                    auto it = discovered.find(serviceInfo.type_string);
                    int selectedIndex = 0;
                    if (it != discovered.end())
                    {
                        int idx = 1; // Start after "Not selected"
                        for (const auto& serviceName : it->second)
                        {
                            comboBox->addItem(QString::fromStdString(serviceName));
                            if (previousSelection == QString::fromStdString(serviceName))
                            {
                                selectedIndex = idx;
                            }
                            ++idx;
                        }
                    }

                    comboBox->setCurrentIndex(selectedIndex); // Restore selection if possible

                    comboBox->blockSignals(false); // Re-enable signals
                }
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ServiceDiscovery"), "Service discovery failed: %s", e.what());
        }
    }

    void ServiceDiscovery::autoSelectServices()
    {
        // For each service type, if only one service is available, select it automatically
        for (const auto& serviceInfo : SUPPORTED_SERVICE_IDL_TYPES)
        {
            QComboBox* comboBox = serviceComboBoxes_[serviceInfo.type];
            if (comboBox && comboBox->count() == 2) // "Not selected" + 1 available service
            {
                comboBox->setCurrentIndex(1); // Select the only available service
                emit serviceComboBoxChanged(serviceInfo.type, comboBox->currentText());
            }
        }
    }

    void ServiceDiscovery::startServiceDiscovery() { autoDiscoveryTimer_->start(1000); }

    void ServiceDiscovery::stopServiceDiscovery() { autoDiscoveryTimer_->stop(); }

    void ServiceDiscovery::onAutoDiscoveryToggled(bool enabled)
    {
        if (enabled)
        {
            startServiceDiscovery();
        }
        else
        {
            stopServiceDiscovery();
        }
    }

    void ServiceDiscovery::createServiceComboBox(const ServiceInfo& idlType, QVBoxLayout* layout)
    {
        auto* label = new QLabel(parent_);
        label->setText(QString::fromStdString(idlType.friendly_name));
        label->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);

        auto* comboBox = new QComboBox(parent_);
        comboBox->setObjectName(QString::fromStdString(idlType.type_string));
        comboBox->addItem("Not selected");
        comboBox->setCurrentIndex(0);

        comboBox->setProperty("idlType", QVariant::fromValue(idlType.type));
        QObject::connect(comboBox, &QComboBox::currentTextChanged, this, &ServiceDiscovery::onServiceComboBoxChanged);

        serviceComboBoxes_[idlType.type] = comboBox;
        serviceLabels_[idlType.type] = label;

        layout->addWidget(label);
        layout->addWidget(comboBox);
    }

    void ServiceDiscovery::onServiceComboBoxChanged(const QString& text)
    {
        QComboBox* comboBox = qobject_cast<QComboBox*>(sender());
        if (comboBox)
        {
            auto text = comboBox->currentText();
            // Check if "Not selected" is chosen.
            if (comboBox->currentIndex() == 0)
            {
                text = "";
            }
            int idlTypeInt = comboBox->property("idlType").toInt();
            ServiceType idlType = static_cast<ServiceType>(idlTypeInt);
            emit serviceComboBoxChanged(idlType, text);
        }
    }

    void ServiceDiscovery::saveConfig(rviz_common::Config config) const
    {
        for (const auto& serviceInfo : SUPPORTED_SERVICE_IDL_TYPES)
        {
            QComboBox* comboBox = serviceComboBoxes_[serviceInfo.type];
            if (comboBox)
            {
                QString key = QString("service_%1").arg(static_cast<int>(serviceInfo.type));
                config.mapSetValue(key, comboBox->currentText());
            }
        }
    }

    void ServiceDiscovery::loadConfig(const rviz_common::Config& config)
    {
        for (const auto& serviceInfo : SUPPORTED_SERVICE_IDL_TYPES)
        {
            QComboBox* comboBox = serviceComboBoxes_[serviceInfo.type];
            if (comboBox)
            {
                QString key = QString("service_%1").arg(static_cast<int>(serviceInfo.type));
                QString savedSelection;
                if (config.mapGetString(key, &savedSelection))
                {
                    int index = comboBox->findText(savedSelection);
                    if (index != -1)
                    {
                        comboBox->setCurrentIndex(index);
                    }
                }
            }
        }
    }
} // namespace q_simulation_interfaces
