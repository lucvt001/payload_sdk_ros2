#include <payload-sdk-ros2/liveview.hpp>

LiveViewWrapper::LiveViewWrapper(const rclcpp::Node::SharedPtr& node)
    : node_(node)
{
    // --------------------------------------------------------------
    // This blocks of code will initialize the Liveview module
    try {
        liveviewSample = new LiveviewSample();
    } catch (...) {
        std::cerr << "Failed to initialize LiveviewSample" << std::endl;
        exit(1);
    }
    // --------------------------------------------------------------

    node_->declare_parameter("camera_index", 0);
    node_->get_parameter("camera_index", camera_index_);

    switch (camera_index_) {
        case '0':
            liveviewSample->StartFpvCameraStream(&DjiUser_ShowRgbImageCallback, &fpvName);
            break;
        case '1':
            liveviewSample->StartMainCameraStream(&DjiUser_ShowRgbImageCallback, &mainName);
            break;
        case '2':
            liveviewSample->StartViceCameraStream(&DjiUser_ShowRgbImageCallback, &viceName);
            break;
        case '3':
            liveviewSample->StartTopCameraStream(&DjiUser_ShowRgbImageCallback, &topName);
            break;
        default:
            cout << "Camera index must be 0 (FPV), 1 (Main), 2 (Vice), or 3 (Top)";
            delete liveviewSample;
            exit(1);
    }

    image_transport::ImageTransport it(node_);
    image_pub_ = it.advertise("camera/image", 1);
}

void LiveViewWrapper::DjiUser_ShowRgbImageCallback(CameraRGBImage img, void *userData)
{
    LiveViewWrapper* live_view_wrapper = static_cast<LiveViewWrapper*>(userData);
    std::string name = string(reinterpret_cast<char *>(userData));
    cv::Mat mat(img.height, img.width, CV_8UC3, img.rawData.data(), img.width * 3);
    cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
    cv::imshow(name, mat);

    sensor_msgs::msg::Image::SharedPtr image = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", mat).toImageMsg();
    live_view_wrapper->image_pub_.publish(image);
}

LiveViewWrapper::~LiveViewWrapper()
{
    switch (camera_index_) {
        case '0':
            liveviewSample->StopFpvCameraStream();
            break;
        case '1':
            liveviewSample->StopMainCameraStream();
            break;
        case '2':
            liveviewSample->StopViceCameraStream();
            break;
        case '3':
            liveviewSample->StopTopCameraStream();
            break;
        default:
            cout << "No camera selected";
            delete liveviewSample;
    }

    delete liveviewSample;
}