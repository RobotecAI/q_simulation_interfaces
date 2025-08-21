#pragma once
#include <QMessageBox>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <simulation_interfaces/msg/result.hpp>
#include <simulation_interfaces/srv/get_simulator_features.hpp>
#include <string>
#include <type_traits>
#include <variant>
#include "string_to_keys.h"

//! Simple expected-like class, where T is the expected type and error is a string.
//! This class is used to handle service responses and errors in a more structured way.
template <typename T>
class Expected
{
    std::optional<T> value_;
    std::string error_;

public:
    //! Constructs the Expected object with a value or an error message.
    Expected(const T& value) : value_(value) {}
    Expected(T&& value) : value_(std::move(value)) {}

    //! Constructs the Expected object with an error message.
    Expected(const std::string& error) : error_(error) {}

    //! Returns the value if it exists, otherwise throws an exception.
    bool has_value() const { return value_.has_value(); }

    //! Returns true if the Expected object contains a value, false otherwise.
    operator bool() const { return has_value(); }

    //! Returns the value. Does not check if the value exists.
    const T& operator*() const
    {
        assert(value_.has_value());
        return *value_;
    }

    //! Returns the value. Does not check if the value exists.
    const T* operator->() const
    {
        assert(value_.has_value());
        return &(*value_);
    }

    //! Returns the error message if it exists, otherwise returns an empty string.
    const std::string& error() const
    {
        assert(!value_.has_value());
        return error_;
    }
};

//! Produces a QT warning message box if the response has an error or if the result is not OK.
//! @param parent - parent widget for the message box
//! @param operation - the operation that was performed, used in the error message
//! @param response - the response from the service call, which can be an Expected<T>
//! @tparam T - the service response type (e.g., simulation_interfaces::srv::GetSimulatorFeatures::Response)
template <typename T>
void ProduceWarningIfProblem(QWidget* parent, const QString operation, const Expected<T>& response)
{
    if (!response.has_value())
    {
        auto w = QString("Failed to %1: %2").arg(operation, QString::fromStdString(response.error()));
        QMessageBox::warning(parent, "Error", w);
        return;
    }

    // only GetSimulatorFeatures::Response does not have a result field, so check if T is not that type
    if constexpr (!std::is_same_v<T, simulation_interfaces::srv::GetSimulatorFeatures::Response>)
    {
        if (response->result.result != simulation_interfaces::msg::Result::RESULT_OK)
        {
            auto w = QString("Operation %1 failed: %2")
                         .arg(operation, QString::fromStdString(response->result.error_message));
            QMessageBox::warning(parent, "Error", w);
        }
    }
}

//! Interface for checking service results.
class ServiceInterface
{
public:
    //! Pokes the service to check if the result is ready.
    //! This method should be called periodically to check if the service call has completed.
    virtual void check_service_result() = 0;
};

//! Template class for a service client that can call a service asynchronously or synchronously.
//! It uses the ROS 2 rclcpp client to send requests and receive responses.
//! @tparam T - the service type, which must be a ROS 2 service type
template <typename T>
class Service : public ServiceInterface
{
public:
    using Request = typename T::Request;
    using Response = typename T::Response;

    using SharedPtr = std::shared_ptr<T>;

    using Client = rclcpp::Client<T>;
    using ClientSharedPtr = typename Client::SharedPtr;
    using FutureAndRequestId = typename Client::FutureAndRequestId;

    //! Callback type for the service call completion.
    //! It gives back to the caller an Expected<Response> object, which can either contain the response or an error
    //! message.
    using CompletionCallback = std::function<void(Expected<Response>)>;

    //! Constructor for the Service class.
    //! @param service_name - the name of the service to call
    //! @param node - the ROS 2 node to use for creating the client
    //! @param timeout - the timeout for the service call in milliseconds, default is 1000 ms
    Service(const std::string& service_name, rclcpp::Node::SharedPtr node, double timeout = 1000) :
        client_(node->create_client<T>(service_name)), node_(node),
        timeout_(std::chrono::milliseconds(static_cast<int>(timeout)))
    {
    }

    virtual ~Service() = default;

    //! Calls the service asynchronously.
    //! @param callback - a callback function to handle the response, can be nullptr
    //! @param request - the request to send, default is an empty request
    void call_service_async(CompletionCallback callback = nullptr, const Request& request = Request())
    {
        RCLCPP_DEBUG(node_->get_logger(), "Calling service %s", client_->get_service_name());

        if (!client_->service_is_ready())
        {
            if (callback)
            {
                callback(Expected<Response>{"Service not available"});
                RCLCPP_ERROR(node_->get_logger(), "Service %s is not available", client_->get_service_name());
            }
            return;
        }
        std::shared_ptr<Request> req = std::make_shared<Request>(request);
        service_result_ = client_->async_send_request(req);
        callback_ = callback;
        service_called_time_ = std::chrono::system_clock::now();
    }

    //! Checks the service result and calls the callback if the result is ready.
    //! This method should be called periodically to check if the service call has completed.
    void check_service_result() override
    {
        if (!service_result_)
        {
            return;
        }
        // check duration
        if (timeout_.count() > 0 && service_called_time_)
        {
            auto duration = std::chrono::system_clock::now() - *service_called_time_;
            if (duration > timeout_)
            {
                RCLCPP_ERROR(node_->get_logger(), "Service call timed out");
                if (callback_)
                {
                    callback_(Expected<Response>("Service call timed out"));
                    callback_ = nullptr;
                }
                service_result_.reset();
                service_called_time_.reset();
                return;
            }
        }

        if (service_result_->wait_for(std::chrono::seconds(0)) == std::future_status::ready)
        {
            auto response = service_result_->future.get();
            RCLCPP_DEBUG(node_->get_logger(), "Service %s response) received", client_->get_service_name());
            if (callback_)
            {
                callback_(Expected(*response));
                callback_ = nullptr;
            }
            service_result_.reset();
            service_called_time_.reset();
        }
        RCLCPP_DEBUG(node_->get_logger(), "Service %s is still processing", client_->get_service_name());
    }

    //! Calls the service synchronously and checks the response.
    //! @note, this method need to be called against ROS 2 node that can be spun.
    //! @param request - the request to send, default is an empty request
    Expected<Response> call_service_sync(const Request& request = Request(), bool silent = false)
    {
        if (!client_->wait_for_service(timeout_))
        {
            RCLCPP_ERROR(node_->get_logger(), "Service not available after waiting");
            return Expected<Response>{"Service not available after waiting"};
        }
        if constexpr (std::is_same<simulation_interfaces::srv::GetSimulatorFeatures, T>())
        {
            return call_service_sync_NoCheck(request);
        }
        else
        {
            if (silent)
            {
                return call_service_sync_NoCheck(request);
            }
            return call_service_sync_Check(request);
        }
    }

private:
    Expected<Response> call_service_sync_Check(const Request& request)
    {
        std::shared_ptr<Request> req = std::make_shared<Request>(request);
        auto future = client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(node_, future, timeout_) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
            return Expected<Response>{"Failed to call service"};
        }
        auto response = future.get();
        if (response->result.result != simulation_interfaces::msg::Result::RESULT_OK)
        {
            RCLCPP_ERROR(node_->get_logger(), "Service call failed: %s", response->result.error_message.c_str());

            const auto error_code = static_cast<int>(response->result.result);
            QString errorType;
            if (auto it = ErrorIdToName.find(error_code); it != ErrorIdToName.end())
            {
                errorType = QString::fromStdString(it->second);
            }
            else
            {
                errorType = "Error : " + QString::number(error_code);
            }

            QMessageBox::warning(nullptr, errorType, QString::fromStdString(response->result.error_message));
            return Expected<Response>{*response};
        }
        return Expected<Response>{*response};
    }

    Expected<Response> call_service_sync_NoCheck(const Request& request = Request())
    {
        std::shared_ptr<Request> req = std::make_shared<Request>(request);
        auto future = client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(node_, future, timeout_) != rclcpp::FutureReturnCode::SUCCESS)
        {
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
    std::optional<FutureAndRequestId> service_result_;
    std::optional<std::chrono::time_point<std::chrono::system_clock>> service_called_time_;
    std::function<void(Expected<Response>)> callback_;
};
