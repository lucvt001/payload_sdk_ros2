#ifndef PSDK_WRAPPER_HPP
#define PSDK_WRAPPER_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <payload_sdk_ros2/application.hpp>
#include <payload_sdk_ros2/liveview.hpp>
#include <payload_sdk_ros2/flight_controller.hpp>

class PSDKWrapper : public rclcpp::Node
{
public:
    PSDKWrapper();
    ~PSDKWrapper();

    // initialize has to be a separate function to avoid "bad_weak_ptr" error when passing the node to LiveViewWrapper
    void initialize();

private:
    bool is_enable_liveview_ = false;
    bool is_enable_flight_control_ = true;

    std::unique_ptr<LiveViewWrapper> liveview_wrapper_;
    std::unique_ptr<FlightControllerWrapper> flight_controller_wrapper_;
};

#endif // PSDK_WRAPPER_HPP