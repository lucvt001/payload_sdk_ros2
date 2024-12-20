#ifndef FLIGHT_CONTROLLER_HPP
#define FLIGHT_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <module_sample_c/flight_control/test_flight_control.h>
#include <module_sample_c/utils/util_misc.h>
#include <psdk_wrapper/log_all.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <psdk_interfaces/action/take_off.hpp>
#include <psdk_interfaces/action/land.hpp>
#include <psdk_interfaces/action/move_to_position.hpp>
#include <psdk_interfaces/srv/obtain_joystick_authority.hpp>
#include <psdk_interfaces/srv/release_joystick_authority.hpp>
#include <psdk_interfaces/srv/set_joystick_mode.hpp>
#include <psdk_interfaces/msg/joystick_command.hpp>
#include <psdk_interfaces/msg/velocity_command.hpp>
#include <future>
#include <chrono>

class FlightControllerWrapper
{
public:
    explicit FlightControllerWrapper(std::shared_ptr<rclcpp::Node> node);
    ~FlightControllerWrapper();

private:
    std::shared_ptr<rclcpp::Node> node_;
    E_DjiFlightControllerJoystickCtrlAuthority current_control_authority_;
    

    rclcpp_action::Server<psdk_interfaces::action::TakeOff>::SharedPtr takeoff_action_server_;
    rclcpp_action::Server<psdk_interfaces::action::Land>::SharedPtr land_action_server_;
    rclcpp_action::Server<psdk_interfaces::action::MoveToPosition>::SharedPtr move_to_position_action_server_;

    rclcpp::Service<psdk_interfaces::srv::ObtainJoystickAuthority>::SharedPtr obtain_joystick_authority_service_;
    rclcpp::Service<psdk_interfaces::srv::ReleaseJoystickAuthority>::SharedPtr release_joystick_authority_service_;
    rclcpp::Service<psdk_interfaces::srv::SetJoystickMode>::SharedPtr set_joystick_mode_service_;

    rclcpp::Subscription<psdk_interfaces::msg::JoystickCommand>::SharedPtr joystick_command_subscriber_;
    rclcpp::Subscription<psdk_interfaces::msg::VelocityCommand>::SharedPtr velocity_command_subscriber_;

    void execute_takeoff(const std::shared_ptr<rclcpp_action::ServerGoalHandle<psdk_interfaces::action::TakeOff>> goal_handle);
    void execute_land(const std::shared_ptr<rclcpp_action::ServerGoalHandle<psdk_interfaces::action::Land>> goal_handle);
    void execute_move_to_position(const std::shared_ptr<rclcpp_action::ServerGoalHandle<psdk_interfaces::action::MoveToPosition>> goal_handle);

    void handle_obtain_joystick_authority(const std::shared_ptr<psdk_interfaces::srv::ObtainJoystickAuthority::Request> request,
                                          std::shared_ptr<psdk_interfaces::srv::ObtainJoystickAuthority::Response> response);
    void handle_release_joystick_authority(const std::shared_ptr<psdk_interfaces::srv::ReleaseJoystickAuthority::Request> request,
                                           std::shared_ptr<psdk_interfaces::srv::ReleaseJoystickAuthority::Response> response);
    void handle_set_joystick_mode(const std::shared_ptr<psdk_interfaces::srv::SetJoystickMode::Request> request,
                                        std::shared_ptr<psdk_interfaces::srv::SetJoystickMode::Response> response);
    void joystick_command_callback(const psdk_interfaces::msg::JoystickCommand::SharedPtr msg);
    void velocity_command_callback(const psdk_interfaces::msg::VelocityCommand::SharedPtr msg);

    static T_DjiReturnCode JoystickCtrlAuthSwitchEventCallbackStatic(T_DjiFlightControllerJoystickCtrlAuthorityEventInfo eventData);
    T_DjiReturnCode JoystickCtrlAuthSwitchEventCallback(T_DjiFlightControllerJoystickCtrlAuthorityEventInfo eventData);
    static FlightControllerWrapper* instance_;
};

#endif // FLIGHT_CONTROLLER_HPP