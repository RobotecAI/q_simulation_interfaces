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
#include <simulation_interfaces/srv/step_simulation.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <type_traits>
#include <variant>
#include <string>
#include <QMessageBox>
#include "stringToKeys.h"

// Simple expected-like class
template<typename T>
class Expected {
private:
    std::optional<T> value_;
    std::string error_;

public:
    Expected(const T& value) : value_(value) {}
    Expected(T&& value) : value_(std::move(value)) {}
    Expected(const std::string& error) : error_(error) {}

    bool has_value() const { return value_.has_value(); }
    operator bool() const { return has_value(); }
    
    const T& operator*() const { return *value_; }
    const T* operator->() const { return &(*value_); }
    
    const std::string& error() const { return error_; }
};

template<typename T>


class Service {
public:
    using Request = typename T::Request;
    using Response = typename T::Response;
    using SharedPtr = std::shared_ptr<T>;

    using Client = rclcpp::Client<T>;
    using ClientSharedPtr = typename Client::SharedPtr;

    Service(const std::string &service_name, rclcpp::Node::SharedPtr node, double timeout = 1.0)
            : client_(node->create_client<T>(service_name)),
              node_(node), timeout_(std::chrono::milliseconds(static_cast<int>(timeout * 1000))) {
    }

    Expected<Response> call_service_sync(const Request &request = Request(), bool silent = false) {
        if (!client_->wait_for_service( timeout_)) {
            RCLCPP_ERROR(node_->get_logger(), "Service not available after waiting");
            return Expected<Response>{"Service not available after waiting"};
        }
        if constexpr (std::is_same<simulation_interfaces::srv::GetSimulatorFeatures,T>() )
        {
            return call_service_sync_NoCheck(request);
        } else {
            if (silent)
            {
                return call_service_sync_NoCheck(request);
            }
            return call_service_sync_Check(request);
        }
    }
private:
    Expected<Response> call_service_sync_Check(const Request &request){
        std::shared_ptr <Request> req = std::make_shared<Request>(request);
        auto future = client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(node_, future, timeout_) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
            return Expected<Response>{"Failed to call service"};
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
            return Expected<Response>{*response};
        }
        return Expected<Response>{*response};
    }

    Expected<Response> call_service_sync_NoCheck(const Request &request = Request() ) {
        std::shared_ptr <Request> req = std::make_shared<Request>(request);
        auto future = client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(node_, future, timeout_) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
            return Expected<Response>{"Failed to call service"};
        }
        auto response = future.get();
        return Expected<Response>{*response};
    }

private:
    rclcpp::Node::SharedPtr node_;
    ClientSharedPtr client_;
    std::chrono::milliseconds timeout_;
};