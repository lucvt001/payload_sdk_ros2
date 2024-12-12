#ifndef FLIGHT_CONTROLLER_HPP
#define FLIGHT_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <module_sample_c/flight_control/test_flight_control.h>
#include <module_sample_c/utils/util_misc.h>
#include "payload_sdk_ros2/log_all.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <payload_sdk_ros2_interfaces/action/take_off.hpp>

class FlightControllerWrapper
{
public:
    explicit FlightControllerWrapper(std::shared_ptr<rclcpp::Node> node);
    ~FlightControllerWrapper();

private:
    std::shared_ptr<rclcpp::Node> node_;

};

#endif // FLIGHT_CONTROLLER_HPP