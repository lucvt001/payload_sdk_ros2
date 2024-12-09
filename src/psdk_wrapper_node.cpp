#include <payload-sdk-ros2/psdk_wrapper_node.hpp>
#include <cstring>

PSDKWrapper::PSDKWrapper() : Node("psdk_wrapper_node")
{
    if (is_enable_liveview_) {
        RCLCPP_INFO(get_logger(), "Enabling Liveview");
        liveview_wrapper_ = new LiveViewWrapper(shared_from_this());
        RCLCPP_INFO(get_logger(), "Liveview enabled");
    }
}

PSDKWrapper::~PSDKWrapper()
{
    std::cout << "Destructor called" << std::endl;
}

int main(int argc, char **argv)
{
    // Initialize DJICore before starting the ROS2 node
    Application application(argc, argv);

    // Other components of DJI will be initialized inside the node
    rclcpp::init(argc, argv);
    auto psdk_wrapper = std::make_shared<PSDKWrapper>();

    rclcpp::spin(psdk_wrapper);
    rclcpp::shutdown();
    return 0;
}

