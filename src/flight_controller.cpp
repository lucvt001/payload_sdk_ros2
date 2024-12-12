#include <payload_sdk_ros2/flight_controller.hpp>

FlightControllerWrapper::FlightControllerWrapper(std::shared_ptr<rclcpp::Node> node)
    : node_(node)
{
    T_DjiReturnCode returnCode = DjiTest_FlightControlInit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        log_error(node_, "Init flight control wrapper failed");
    }
    log_info(node_, "Init flight control wrapper success");
}

FlightControllerWrapper::~FlightControllerWrapper()
{
    T_DjiReturnCode returnCode = DjiTest_FlightControlDeInit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        log_error(node_, "Deinit flight control wrapper failed");
    }
    log_info(node_, "Deinit flight control wrapper success");
}