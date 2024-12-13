#ifndef FLIGHT_CONTROLLER_HPP
#define FLIGHT_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <module_sample_c/flight_control/test_flight_control.h>
#include <module_sample_c/utils/util_misc.h>
#include "payload_sdk_ros2/log_all.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <payload_sdk_ros2_interfaces/action/take_off.hpp>
#include <payload_sdk_ros2_interfaces/action/land.hpp>
#include <payload_sdk_ros2_interfaces/srv/obtain_joystick_authority.hpp>
#include <payload_sdk_ros2_interfaces/srv/release_joystick_authority.hpp>
#include <payload_sdk_ros2_interfaces/srv/set_joystick_mode.hpp>
#include <payload_sdk_ros2_interfaces/msg/joystick_command.hpp>

class FlightControllerWrapper
{
public:
    explicit FlightControllerWrapper(std::shared_ptr<rclcpp::Node> node);
    ~FlightControllerWrapper();

private:
    std::shared_ptr<rclcpp::Node> node_;

    rclcpp_action::Server<payload_sdk_ros2_interfaces::action::TakeOff>::SharedPtr takeoff_action_server_;
    rclcpp_action::Server<payload_sdk_ros2_interfaces::action::Land>::SharedPtr land_action_server_;

    rclcpp::Service<payload_sdk_ros2_interfaces::srv::ObtainJoystickAuthority>::SharedPtr obtain_joystick_authority_service_;
    rclcpp::Service<payload_sdk_ros2_interfaces::srv::ReleaseJoystickAuthority>::SharedPtr release_joystick_authority_service_;
    rclcpp::Service<payload_sdk_ros2_interfaces::srv::SetJoystickMode>::SharedPtr set_joystick_mode_service_;

    rclcpp::Subscription<payload_sdk_ros2_interfaces::msg::JoystickCommand>::SharedPtr joystick_command_subscriber_;

    void execute_takeoff(const std::shared_ptr<rclcpp_action::ServerGoalHandle<payload_sdk_ros2_interfaces::action::TakeOff>> goal_handle);
    void execute_land(const std::shared_ptr<rclcpp_action::ServerGoalHandle<payload_sdk_ros2_interfaces::action::Land>> goal_handle);

    void handle_obtain_joystick_authority(const std::shared_ptr<payload_sdk_ros2_interfaces::srv::ObtainJoystickAuthority::Request> request,
                                          std::shared_ptr<payload_sdk_ros2_interfaces::srv::ObtainJoystickAuthority::Response> response);
    void handle_release_joystick_authority(const std::shared_ptr<payload_sdk_ros2_interfaces::srv::ReleaseJoystickAuthority::Request> request,
                                           std::shared_ptr<payload_sdk_ros2_interfaces::srv::ReleaseJoystickAuthority::Response> response);
    void handle_set_joystick_mode(const std::shared_ptr<payload_sdk_ros2_interfaces::srv::SetJoystickMode::Request> request,
                                        std::shared_ptr<payload_sdk_ros2_interfaces::srv::SetJoystickMode::Response> response);
    void joystick_command_callback(const payload_sdk_ros2_interfaces::msg::JoystickCommand::SharedPtr msg);

};

#endif // FLIGHT_CONTROLLER_HPP