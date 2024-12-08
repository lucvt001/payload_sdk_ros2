#ifndef PSDK_WRAPPER_HPP
#define PSDK_WRAPPER_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <payload-sdk-ros2/application.hpp>
#include <payload-sdk-ros2/liveview.hpp>

class PSDKWrapper : public rclcpp::Node
{
public:
    PSDKWrapper();
    ~PSDKWrapper();

private:
    bool is_enable_liveview_ = false;
    bool is_enable_flight_control_ = false;

    LiveViewWrapper *liveview_wrapper_;
};

#endif // PSDK_WRAPPER_HPP