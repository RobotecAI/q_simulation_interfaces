#pragma once
#include <simulation_interfaces/srv/get_spawnables.hpp>
#include <simulation_interfaces/srv/spawn_entity.hpp>
#include <simulation_interfaces/srv/get_entities.hpp>
#include <simulation_interfaces/srv/get_entity_state.hpp>
#include <simulation_interfaces/srv/set_entity_state.hpp>
#include <simulation_interfaces/srv/delete_entity.hpp>
#include <simulation_interfaces/srv/get_simulator_features.hpp>
#include <simulation_interfaces/srv/reset_simulation.hpp>
#include <simulation_interfaces/msg/result.hpp>
#include <simulation_interfaces/srv/get_simulation_state.hpp>
#include <simulation_interfaces/srv/set_simulation_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <type_traits>
#include "stringToKeys.h"

template<typename T>
class Service {
public:
    using Request = typename T::Request;
    using Response = typename T::Response;
    using SharedPtr = std::shared_ptr<T>;

    using Client = rclcpp::Client<T>;
    using ClientSharedPtr = typename Client::SharedPtr;

    Service(const std::string &service_name, rclcpp::Node::SharedPtr node)
            : client_(node->create_client<T>(service_name)),
              node_(node) {
        if (!client_->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_ERROR(node->get_logger(), "Service not available after waiting");
        }
    }

    std::optional<Response> call_service_sync(const Request &request = Request()) {

        if constexpr (std::is_same<simulation_interfaces::srv::GetSimulatorFeatures,T>())
        {
            return call_service_sync_NoCheck(request);
        } else {
            return call_service_sync_Check(request);
        }
    }
private:
    std::optional<Response> call_service_sync_Check(const Request &request) {
        std::shared_ptr <Request> req = std::make_shared<Request>(request);
        auto future = client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(node_, future) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
            return std::nullopt;
        }
        auto response = future.get();
        if (response->result.result != simulation_interfaces::msg::Result::RESULT_OK) {
            RCLCPP_ERROR(node_->get_logger(), "Service call failed: %s", response->result.error_message.c_str());

            const auto error_code = static_cast<int>(response->result.result);
            QString errorType;
            if (auto it = ErrorIdToName.find(error_code); it != ErrorIdToName.end()) {
                errorType = QString::fromStdString(it->second);
            } else {
                errorType = "Error : " + QString::number(error_code) ;
            }

            QMessageBox::warning(nullptr, errorType, QString::fromStdString(response->result.error_message));
            return *response;
        }
        return *response;
    }

    std::optional<Response> call_service_sync_NoCheck(const Request &request = Request() ) {
        std::shared_ptr <Request> req = std::make_shared<Request>(request);
        auto future = client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(node_, future) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
            return std::nullopt;
        }
        auto response = future.get();
        return *response;
    }

private:
    rclcpp::Node::SharedPtr node_;
    ClientSharedPtr client_;
};