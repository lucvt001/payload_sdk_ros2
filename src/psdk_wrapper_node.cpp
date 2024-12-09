#include <payload-sdk-ros2/psdk_wrapper_node.hpp>
#include <cstring>

PSDKWrapper::PSDKWrapper() : Node("psdk_wrapper_node")
{
    // Placeholder for now
}

void PSDKWrapper::initialize()
{
    if (is_enable_liveview_) {
        liveview_wrapper_ = std::make_unique<LiveViewWrapper>(this->shared_from_this());
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
    psdk_wrapper->initialize();

    rclcpp::spin(psdk_wrapper);
    rclcpp::shutdown();
    return 0;
}

