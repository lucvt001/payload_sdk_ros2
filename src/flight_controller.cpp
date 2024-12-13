#include <payload_sdk_ros2/flight_controller.hpp>

FlightControllerWrapper::FlightControllerWrapper(std::shared_ptr<rclcpp::Node> node)
    : node_(node)
{
    T_DjiReturnCode returnCode = DjiTest_FlightControlInit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        log_error(node_, "Init flight control wrapper failed");
    }
    log_info(node_, "Init flight control wrapper success");


    takeoff_action_server_ = rclcpp_action::create_server<payload_sdk_ros2_interfaces::action::TakeOff>
    (
        node_, "takeoff",
        [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const payload_sdk_ros2_interfaces::action::TakeOff::Goal> goal) {
            RCLCPP_INFO(node_->get_logger(), "Received takeoff goal request");
            (void)uuid;
            // Accept the goal
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<payload_sdk_ros2_interfaces::action::TakeOff>> goal_handle) {
            RCLCPP_INFO(node_->get_logger(), "Received request to cancel takeoff goal");
            // Accept the cancel request
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<payload_sdk_ros2_interfaces::action::TakeOff>> goal_handle) {
            // Execute the goal in a separate thread
            std::thread([this, goal_handle]() {
                execute_takeoff(goal_handle);
            }).detach();
        }
    );

    land_action_server_ = rclcpp_action::create_server<payload_sdk_ros2_interfaces::action::Land>
    (
        node_, "land",
        [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const payload_sdk_ros2_interfaces::action::Land::Goal> goal) {
            RCLCPP_INFO(node_->get_logger(), "Received land goal request");
            (void)uuid;
            // Accept the goal
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<payload_sdk_ros2_interfaces::action::Land>> goal_handle) {
            RCLCPP_INFO(node_->get_logger(), "Received request to cancel land goal");
            // Accept the cancel request
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<payload_sdk_ros2_interfaces::action::Land>> goal_handle) {
            // Execute the goal in a separate thread
            std::thread([this, goal_handle]() {
                execute_land(goal_handle);
            }).detach();
        }
    );

    obtain_joystick_authority_service_ = node_->create_service<payload_sdk_ros2_interfaces::srv::ObtainJoystickAuthority>(
        "obtain_joystick_authority",
        [this](const std::shared_ptr<payload_sdk_ros2_interfaces::srv::ObtainJoystickAuthority::Request> request,
               std::shared_ptr<payload_sdk_ros2_interfaces::srv::ObtainJoystickAuthority::Response> response) {
            handle_obtain_joystick_authority(request, response);
        }
    );

    release_joystick_authority_service_ = node_->create_service<payload_sdk_ros2_interfaces::srv::ReleaseJoystickAuthority>(
        "release_joystick_authority",
        [this](const std::shared_ptr<payload_sdk_ros2_interfaces::srv::ReleaseJoystickAuthority::Request> request,
               std::shared_ptr<payload_sdk_ros2_interfaces::srv::ReleaseJoystickAuthority::Response> response) {
            handle_release_joystick_authority(request, response);
        }
    );

    set_joystick_mode_service_ = node_->create_service<payload_sdk_ros2_interfaces::srv::SetJoystickMode>(
        "set_joystick_mode",
        [this](const std::shared_ptr<payload_sdk_ros2_interfaces::srv::SetJoystickMode::Request> request,
               std::shared_ptr<payload_sdk_ros2_interfaces::srv::SetJoystickMode::Response> response) {
            handle_set_joystick_mode(request, response);
        }
    );

    joystick_command_subscriber_ = node_->create_subscription<payload_sdk_ros2_interfaces::msg::JoystickCommand>(
        "joystick_command",
        10,
        std::bind(&FlightControllerWrapper::joystick_command_callback, this, std::placeholders::_1)
    );
}


void FlightControllerWrapper::execute_takeoff(const std::shared_ptr<rclcpp_action::ServerGoalHandle<payload_sdk_ros2_interfaces::action::TakeOff>> goal_handle)
{
    log_info(node_, "Taking off...");

    auto result = std::make_shared<payload_sdk_ros2_interfaces::action::TakeOff::Result>();

    if (!DjiTest_FlightControlMonitoredTakeoff()) {
        log_error(node_, "Takeoff failed");
        goal_handle->abort(result);
        return;
    }

    log_info(node_, "Takeoff succeeded");
    goal_handle->succeed(result);
}


void FlightControllerWrapper::execute_land(const std::shared_ptr<rclcpp_action::ServerGoalHandle<payload_sdk_ros2_interfaces::action::Land>> goal_handle)
{
    log_info(node_, "Landing...");

    auto result = std::make_shared<payload_sdk_ros2_interfaces::action::Land::Result>();

    if (!DjiTest_FlightControlMonitoredLanding()) {
        log_error(node_, "Land failed");
        goal_handle->abort(result);
        return;
    }

    log_info(node_, "Land succeeded");
    goal_handle->succeed(result);
}

void FlightControllerWrapper::handle_obtain_joystick_authority(const std::shared_ptr<payload_sdk_ros2_interfaces::srv::ObtainJoystickAuthority::Request> request,
                                                               std::shared_ptr<payload_sdk_ros2_interfaces::srv::ObtainJoystickAuthority::Response> response)
{
    T_DjiReturnCode returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        log_error(node_, "Obtain joystick authority failed");
        response->is_success = false;
        return;
    }
    log_info(node_, "Obtained joystick authority");
    response->is_success = true;
}

void FlightControllerWrapper::handle_release_joystick_authority(const std::shared_ptr<payload_sdk_ros2_interfaces::srv::ReleaseJoystickAuthority::Request> request,
                                                                std::shared_ptr<payload_sdk_ros2_interfaces::srv::ReleaseJoystickAuthority::Response> response)
{
    T_DjiReturnCode returnCode = DjiFlightController_ReleaseJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        log_error(node_, "Obtain joystick authority failed");
        response->is_success = false;
        return;
    }
    log_info(node_, "Obtained joystick authority");
    response->is_success = true;
}

void FlightControllerWrapper::handle_set_joystick_mode(const std::shared_ptr<payload_sdk_ros2_interfaces::srv::SetJoystickMode::Request> request,
                                                       std::shared_ptr<payload_sdk_ros2_interfaces::srv::SetJoystickMode::Response> response)
{
    T_DjiFlightControllerJoystickMode joystickMode = {
        static_cast<E_DjiFlightControllerHorizontalControlMode>(request->horizontal_control_mode),
        static_cast<E_DjiFlightControllerVerticalControlMode>(request->vertical_control_mode),
        static_cast<E_DjiFlightControllerYawControlMode>(request->yaw_control_mode),
        static_cast<E_DjiFlightControllerHorizontalCoordinate>(request->horizontal_coordinate),
        static_cast<E_DjiFlightControllerStableControlMode>(request->stable_control_mode)
    };
    DjiFlightController_SetJoystickMode(joystickMode);
    log_info(node_, "Set joystick mode success");
    response->is_success = true;
}

void FlightControllerWrapper::joystick_command_callback(const payload_sdk_ros2_interfaces::msg::JoystickCommand::SharedPtr msg)
{
    T_DjiFlightControllerJoystickCommand joystickCommand = {
        msg->x,
        msg->y,
        msg->z,
        msg->yaw
    };
    DjiFlightController_ExecuteJoystickAction(joystickCommand);
}

FlightControllerWrapper::~FlightControllerWrapper()
{
    T_DjiReturnCode returnCode = DjiTest_FlightControlDeInit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        log_error(node_, "Deinit flight control wrapper failed");
    }
    log_info(node_, "Deinit flight control wrapper success");
}